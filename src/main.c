
// Builtins
#include <math.h>
#include <stdio.h>

// ESP IDF
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"

// local
#include "actuator.h"
#include "chip_introspection.h"
#include "pid_control.h"
#include "rotary_encoder.h"

// Pin assignments on ESP32 dev board (NodeMCU-32S)
static const gpio_num_t ONBOARD_LED_PIN = 2;
static const gpio_num_t LIMIT_SWITCH_PIN = 16;
static const gpio_num_t ENCODER_PIN_A = 22;
static const gpio_num_t ENCODER_PIN_B = 23;
static const gpio_num_t MOTOR_PWM_PIN = 32;  // row 13
static const gpio_num_t MOTOR_DIR_PIN = 33;  // row 12
static const mcpwm_unit_t MOTOR_PWM_UNIT = MCPWM_UNIT_0;
static const mcpwm_timer_t MOTOR_PWM_TIMER = MCPWM_TIMER_0;

// If desired, calibrate to cm/tick, inches/tick, etc.
// TODO: provide bounce parameters as user input
static const float POSITION_UNITS_PER_ENCODER_UNIT = 1.0;
// static const char* POSITION_UNITS = "encoder pulses";  // unused
static const float ACTUATOR_HEIGHT = 7500.;
static const float REFERENCE_HEIGHT = 0.4 * ACTUATOR_HEIGHT;
static const float BOUNCE_AMPLITUDE = 1500.0;
static const float AMPLITUDE_RAMP_RATE = 0.5;  // pos. units / timestep
static const TickType_t update_period = pdMS_TO_TICKS(10);

static const float BOUNCE_PERIOD = 1.5;  // seconds
static const float BOUNCE_FREQUENCY = 2 * M_PI / BOUNCE_PERIOD;

// For inter-task communication
static xQueueHandle limit_switch_queue;
xSemaphoreHandle master_tick_signal;
xSemaphoreHandle uart_mutex;

static void IRAM_ATTR limit_switch_isr(void* args) {
  int pin_number = (int)args;
  xQueueSendFromISR(limit_switch_queue, &pin_number, NULL);
}

// Parameters passed to FreeRTOS tasks must be either global
// or heap-allocated.
static actuator_t actuator = {.position = 0,
                              .speed = 0,
                              .direction = DIRECTION_UNKNOWN,
                              .encoder_units = 0,
                              .datum = 0,
                              .homed = false};

static pid_control_t pid = {.kp = 0.005,
                            .ki = 0.0,
                            .kd = 0.001,
                            .setpoint = 0.,
                            .input = 0.,
                            .output = 0.,
                            .min_output = -1.0,
                            .max_output = +1.0,
                            .error = 0.,
                            .error_sum = 0.};

void master_timer_callback(TimerHandle_t timer) {
  xSemaphoreGive(master_tick_signal);
}

// IIR filter using exponentially weighted moving average.
// 0 <= alpha < 1 is a smoothing factor (larger -> smoother)
void filter_ema(const float alpha, const float x_meas, float* x_filt) {
  *x_filt = (1.0 - alpha) * x_meas + alpha * (*x_filt);
}

static float bounce_setpoint(const float t_ms, const float amplitude) {
  return REFERENCE_HEIGHT + amplitude * sin(BOUNCE_FREQUENCY * t_ms / 1000.);
}

static void update_actuator_data(int encoder_position,
                                 rotary_encoder_direction_t encoder_direction,
                                 actuator_t* actuator) {
  // Instantaneous speed (current - prev) in position units per timestep
  const float delta = POSITION_UNITS_PER_ENCODER_UNIT *
                      (encoder_position - actuator->encoder_units);

  actuator->encoder_units = encoder_position;

  if (encoder_direction == ROTARY_ENCODER_DIRECTION_CLOCKWISE) {
    actuator->direction = DIRECTION_UP;
  } else if (encoder_direction == ROTARY_ENCODER_DIRECTION_COUNTER_CLOCKWISE) {
    actuator->direction = DIRECTION_DOWN;
  }

  // Update position and speed
  if (actuator->homed) {
    actuator->position = POSITION_UNITS_PER_ENCODER_UNIT *
                         (actuator->encoder_units - actuator->datum);
  }
  filter_ema(0.7, delta, &actuator->speed);
}

// Expecting -1.0 <= PID output <= +1.0
static void go(const float pid_output) {
  const int gpio_level = pid_output < 0 ? DIRECTION_DOWN : DIRECTION_UP;
  const float duty_percent = 100. * fabs(pid_output);

  gpio_set_level(MOTOR_DIR_PIN, gpio_level);
  mcpwm_set_duty(MOTOR_PWM_UNIT, MOTOR_PWM_TIMER, MCPWM_OPR_A, duty_percent);
}

esp_err_t home_actuator(rotary_encoder_info_t* encoder_info,
                        rotary_encoder_state_t* encoder_state,
                        actuator_t* actuator) {
  // TODO: error reporting/handling
  while (!actuator->homed) {
    ESP_ERROR_CHECK(rotary_encoder_get_state(encoder_info, encoder_state));

    update_actuator_data(encoder_state->position, encoder_state->direction,
                         actuator);
    gpio_set_level(MOTOR_DIR_PIN, DIRECTION_DOWN);
    mcpwm_set_duty(MOTOR_PWM_UNIT, MOTOR_PWM_TIMER, MCPWM_OPR_A, 30.0);
    vTaskDelay(update_period);
  }

  // Bring actuator to center
  // TODO: closed-loop control
  gpio_set_level(MOTOR_DIR_PIN, DIRECTION_UP);
  while (actuator->position < REFERENCE_HEIGHT) {
    ESP_ERROR_CHECK(rotary_encoder_get_state(encoder_info, encoder_state));

    update_actuator_data(encoder_state->position, encoder_state->direction,
                         actuator);
    mcpwm_set_duty(MOTOR_PWM_UNIT, MOTOR_PWM_TIMER, MCPWM_OPR_A, 30.0);
    vTaskDelay(update_period);
  }
  ESP_LOGD("HOMING", "Homing complete! At reference point.");

  mcpwm_set_duty(MOTOR_PWM_UNIT, MOTOR_PWM_TIMER, MCPWM_OPR_A, 0.0);

  return ESP_OK;
}

void limit_switch_task(void* params) {
  actuator_t* actuator = (actuator_t*)params;
  int pin_number = 0;
  int count = 0;

  // Configure onboard LED pin
  gpio_pad_select_gpio(ONBOARD_LED_PIN);
  gpio_set_direction(ONBOARD_LED_PIN, GPIO_MODE_OUTPUT);

  // Configure limit switch pin.
  // Limit switch (NO) connects VCC and LIMIT_SWITCH_PIN
  gpio_config_t conf;
  conf.pin_bit_mask = (1ULL << LIMIT_SWITCH_PIN);
  conf.mode = GPIO_MODE_INPUT;
  conf.intr_type = GPIO_INTR_POSEDGE;
  conf.pull_down_en = 1;
  conf.pull_up_en = 0;
  gpio_config(&conf);

  limit_switch_queue = xQueueCreate(1, sizeof(int));

  gpio_isr_handler_add(LIMIT_SWITCH_PIN, limit_switch_isr,
                       (void*)LIMIT_SWITCH_PIN);

  while (true) {
    if (xQueueReceive(limit_switch_queue, &pin_number, portMAX_DELAY)) {
      // Disable limit switch ISR to prevent queuing garbage results during
      // bounce period
      gpio_isr_handler_remove(pin_number);

      // Interrupt handler logic
      if (gpio_get_level(pin_number) == 1) {
        count++;
        gpio_set_level(ONBOARD_LED_PIN, 1);

        // Change actuator state to homed
        bool already_homed = actuator->homed;
        actuator->datum = actuator->encoder_units;
        actuator->homed = true;

        if (xSemaphoreTake(uart_mutex, 100 / portTICK_PERIOD_MS)) {
          ESP_LOGI("LIMIT_SWITCH", "Trigger count = %d. datum = %d. %s", count,
                   actuator->datum, already_homed ? "re-homed" : "homed");
          xSemaphoreGive(uart_mutex);
        }
      }
      vTaskDelay(pdMS_TO_TICKS(100));  // debounce delay

      // Reenable interrupt after debounce period
      gpio_isr_handler_add(pin_number, limit_switch_isr, (void*)pin_number);

      gpio_set_level(ONBOARD_LED_PIN, 0);
    }
  }
}

static void motor_control_task(void* params) {
  actuator_t* actuator = (actuator_t*)params;

  // MCPWM config and initialization
  mcpwm_config_t pwm_config;
  pwm_config.frequency = 20000;  // Hz,
  pwm_config.cmpr_a = 0;         // duty cycle of PWMxA [%]
  pwm_config.cmpr_b = 0;         // duty cycle of PWMxb
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MOTOR_PWM_UNIT, MOTOR_PWM_TIMER, &pwm_config);
  mcpwm_gpio_init(MOTOR_PWM_UNIT, MCPWM0A, MOTOR_PWM_PIN);

  // Motor DIR GPIO pin
  gpio_pad_select_gpio(MOTOR_DIR_PIN);
  gpio_set_direction(MOTOR_DIR_PIN, GPIO_MODE_OUTPUT);

  // Rotary encoder
  // This assumes gpio_install_isr_service(0) was called
  rotary_encoder_info_t encoder_info = {0};
  rotary_encoder_state_t encoder_state = {0};
  ESP_ERROR_CHECK(
      rotary_encoder_init(&encoder_info, ENCODER_PIN_A, ENCODER_PIN_B));

  ESP_ERROR_CHECK(home_actuator(&encoder_info, &encoder_state, actuator));

  int64_t start_time = esp_timer_get_time();  // microseconds

  float amplitude = 0;

  while (true) {
    xSemaphoreTake(master_tick_signal, 2 * update_period);

    // Sensor update
    ESP_ERROR_CHECK(rotary_encoder_get_state(&encoder_info, &encoder_state));
    update_actuator_data(encoder_state.position, encoder_state.direction,
                         actuator);

    const float t_ms = (esp_timer_get_time() - start_time) / 1000.;

    // Ramp towards BOUNCE_AMPLITUDE at the assigned ramp rate
    if (amplitude < BOUNCE_AMPLITUDE) {
      amplitude += AMPLITUDE_RAMP_RATE;
    }
    pid.setpoint = bounce_setpoint(t_ms, amplitude);

    // Compute PID output
    const float dxdt = actuator->speed / pdTICKS_TO_MS(update_period);
    pid_update(actuator->position, 0., 1, &pid, &dxdt);

    go(pid.output);

    if (xSemaphoreTake(uart_mutex, 2 / portTICK_PERIOD_MS)) {
      ESP_LOGI("ACTUATOR", "%.2f %.2f %.2f %.2f %s", actuator->position,
               actuator->speed, pid.setpoint, pid.output,
               actuator->direction == DIRECTION_UP ? "up" : "down");
      xSemaphoreGive(uart_mutex);
    }
  }
}

void app_main() {
  esp_log_level_set("TESTING", ESP_LOG_VERBOSE);  // everything

  master_tick_signal = xSemaphoreCreateBinary();
  uart_mutex = xSemaphoreCreateMutex();

  vTaskDelay(pdMS_TO_TICKS(500));
  print_device_info();
  print_memory();

  ESP_ERROR_CHECK(gpio_install_isr_service(0));

  xTaskCreate(&limit_switch_task, "limit_switch", 2048, &actuator, 5, NULL);
  xTaskCreate(&motor_control_task, "motor_control_task", 2048, &actuator, 2,
              NULL);

  TimerHandle_t master_timer = xTimerCreate("master_timer", update_period, true,
                                            NULL, &master_timer_callback);

  xTimerStart(master_timer, 0);
}

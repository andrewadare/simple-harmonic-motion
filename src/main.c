// TODO: use a mutex semaphore for UART traffic

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
static const gpio_num_t ENCODER_PIN_A = 25;
static const gpio_num_t ENCODER_PIN_B = 26;
static const gpio_num_t MOTOR_PWM_PIN = 32;  // row 13
static const gpio_num_t MOTOR_DIR_PIN = 33;  // row 12
static const mcpwm_unit_t MOTOR_PWM_UNIT = MCPWM_UNIT_0;
static const mcpwm_timer_t MOTOR_PWM_TIMER = MCPWM_TIMER_0;

// If desired, calibrate to cm/tick, inches/tick, etc.
static const float POSITION_UNITS_PER_ENCODER_UNIT = 1.0;
static const char* POSITION_UNITS = "units";  // or "cm", "in", etc.

// Sets the target sample and update period of the overall system
static const TickType_t update_period = pdMS_TO_TICKS(10);

static xQueueHandle limit_switch_queue;

xSemaphoreHandle master_tick_signal;

static void IRAM_ATTR limit_switch_isr(void* args) {
  int pin_number = (int)args;
  xQueueSendFromISR(limit_switch_queue, &pin_number, NULL);
}

// Parameters passed to FreeRTOS tasks must be either global
// or heap-allocated.
static actuator_t actuator = {.position = -1,
                              .speed = 0,
                              .direction = DIRECTION_UNKNOWN,
                              .encoder_units = 0,
                              .datum = 0,
                              .homed = false};

// static pid_control_t pid = {.kp = 0.,
//                             .ki = 0.,
//                             .kd = 0.,
//                             .setpoint = 0.,
//                             .output = 0.,
//                             .min_output = 0.,
//                             .max_output = 0.,
//                             .error_sum = 0.};

void master_timer_callback(TimerHandle_t timer) {
  xSemaphoreGive(master_tick_signal);
}

// IIR filter using exponentially weighted moving average.
// 0 <= alpha < 1 is a smoothing factor (larger -> smoother)
void filter_ema(const float alpha, const float x_meas, float* x_filt) {
  *x_filt = (1.0 - alpha) * x_meas + alpha * (*x_filt);
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

  // ESP_ERROR_CHECK(gpio_install_isr_service(0));

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

        // Home the actuator
        bool already_homed = actuator->homed;
        actuator->datum = actuator->encoder_units;
        actuator->homed = true;

        ESP_LOGI("LIMIT_SWITCH", "Trigger count = %d. datum = %d. %s", count,
                 actuator->datum, already_homed ? "re-homed" : "homed");
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

  // Swap definition of channel A <-> B so count increases upward
  ESP_ERROR_CHECK(rotary_encoder_flip_direction(&encoder_info));

  while (true) {
    xSemaphoreTake(master_tick_signal, 2 * update_period);

    // Query the encoder counter and update the actuator
    ESP_ERROR_CHECK(rotary_encoder_get_state(&encoder_info, &encoder_state));

    // Instantaneous speed in position units per timestep
    // (current - prev)
    const float delta = POSITION_UNITS_PER_ENCODER_UNIT *
                        (encoder_state.position - actuator->encoder_units);

    actuator->encoder_units = encoder_state.position;

    if (encoder_state.direction == ROTARY_ENCODER_DIRECTION_CLOCKWISE) {
      actuator->direction = DIRECTION_UP;
    } else if (encoder_state.direction ==
               ROTARY_ENCODER_DIRECTION_COUNTER_CLOCKWISE) {
      actuator->direction = DIRECTION_DOWN;
    }

    // Update position and speed
    if (actuator->homed) {
      actuator->position = POSITION_UNITS_PER_ENCODER_UNIT *
                           (actuator->encoder_units - actuator->datum);
    }
    filter_ema(0.7, delta, &actuator->speed);

    ESP_LOGI("ENCODER", "%.2f %.2f %s %s", actuator->position, actuator->speed,
             POSITION_UNITS,
             actuator->direction == DIRECTION_UP ? "up" : "down");

    //   for (int i = 0; i < 100; i++) {
    //     // printf("%.2f %d\n", (float)i, direction);
    //     //   motor_go(direction, (float)i);
    //     gpio_set_level(MOTOR_DIR_PIN, direction);
    //     mcpwm_set_duty(MOTOR_PWM_UNIT, MOTOR_PWM_TIMER, MCPWM_OPR_A, i);

    //     vTaskDelay(10 / portTICK_RATE_MS);
    //   }
    //   for (int i = 100; i > 0; i--) {
    //     // printf("%.2f %d\n", (float)i, direction);
    //     gpio_set_level(MOTOR_DIR_PIN, direction);
    //     mcpwm_set_duty(MOTOR_PWM_UNIT, MOTOR_PWM_TIMER, MCPWM_OPR_A, i);

    //     //   motor_go(direction, (float)i);
    //     vTaskDelay(10 / portTICK_RATE_MS);
    //   }
    //   direction = !direction;
  }
}

void app_main() {
  master_tick_signal = xSemaphoreCreateBinary();

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

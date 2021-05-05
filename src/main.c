// TODO: use a mutex semaphore for UART traffic

// Builtins
#include <math.h>
#include <stdio.h>

// ESP IDF
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
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
#include "i2c_comms.h"

// I2C info for AMS AS5600 magnetic position sensor
#define AS5600_ADDR 0x36
uint8_t as5600_registers[2] = {0x0d, 0x0c};  // for low and high bytes

// Pin assignments on ESP32 dev board
static const gpio_num_t onboard_led_pin = 2;
static const gpio_num_t motor_pwm_pin = 4;
static const gpio_num_t limit_switch_pin = 16;
static const gpio_num_t as5600_sda = 18;
static const gpio_num_t as5600_scl = 19;
static const gpio_num_t MOTOR_PWM_PIN = 32;  // row 13
static const gpio_num_t MOTOR_DIR_PIN = 33;  // row 12
static const mcpwm_unit_t MOTOR_PWM_UNIT = MCPWM_UNIT_0;
static const mcpwm_timer_t MOTOR_PWM_TIMER = MCPWM_TIMER_0;

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
static actuator_t actuator = {.pulley = {.position = -1,
                                         .datum = 0,
                                         .rotation_rate = 0,
                                         .rotations = 0,
                                         .radius = 1.5 /*cm*/},
                              .position = -1,
                              .speed = 0,
                              .direction = DIRECTION_UNKNOWN,
                              .homed = false};

void master_timer_callback(TimerHandle_t timer) {
  xSemaphoreGive(master_tick_signal);
}

// IIR filter using exponentially weighted moving average.
// 0 <= alpha < 1 is a smoothing factor (larger -> smoother)
void filter_ema(const float alpha, const float x_meas, float* x_filt) {
  *x_filt = (1.0 - alpha) * x_meas + alpha * (*x_filt);
}

// Return x wrapped into the interval [a, b)
int wrap(const int x, const int a, const int b) {
  if (x < a) {
    return x + b - a;
  }
  if (x >= b) {
    return x - b + a;
  }
  return x;
}

// TODO: corner case logic for datum near 0 or 4095
bool datum_crossed(const int prev_position, actuator_t* a) {
  if (!a->homed) {
    return false;  // no datum yet
  }

  const int d = a->pulley.datum;
  const int x = a->pulley.position;
  const int xp = prev_position;
  // const bool datum_outside = (x >= d && xp > d) || (x <= d && xp < d);

  if (a->pulley.rotation_rate > 0) {
    if (x >= d && d > xp) {
      return true;
    }
    // if (x < xp && datum_outside) {
    //   ESP_LOGW("DATUM_CROSSED", "w > 0, x < xp");
    //   return true;
    // }
  }
  if (a->pulley.rotation_rate < 0) {
    if (x <= d && d < xp) {
      return true;
    }
    // if (x > xp && datum_outside) {
    //   ESP_LOGW("DATUM_CROSSED", "w < 0, x > xp");
    //   return true;
    // }
  }
  return false;
}

// Update actuator structure from a measured 12 bit rotational position
// measurement. The AS5600 magnetic sensor occasionally reports anomalous
// samples. For sufficiently high sample rates, the angular velocity should be
// fairly smooth. If an excursion is detected, the bad measurement is replaced
// with a dead reckoning estimate.
void update_actuator(const int position, actuator_t* a) {
  float delta = 0;  // Change from previous position
  const int prev_position = a->pulley.position;

  if (prev_position >= 0) {  // true except at initialization
    delta = position - prev_position;
    delta = wrap(delta, -2048, 2048);
  }

  // Update the pulley
  static int num_dead_reckon_steps = 0;
  if (fabs(delta - a->pulley.rotation_rate) > 20.0 &&
      num_dead_reckon_steps < 2) {
    ESP_LOGW("update_actuator", "Skip: delta - EMA = %.2f",
             delta - a->pulley.rotation_rate);
    a->pulley.position += round(a->pulley.rotation_rate);
    num_dead_reckon_steps++;
  } else {
    a->pulley.position = position;
    filter_ema(0.6, delta, &a->pulley.rotation_rate);
    num_dead_reckon_steps = 0;
  }

  if (a->pulley.rotation_rate > 0) {
    a->direction = DIRECTION_DOWN;
  } else if (a->pulley.rotation_rate < 0) {
    a->direction = DIRECTION_UP;
  } else {
    a->direction = DIRECTION_UNKNOWN;
  }

  if (datum_crossed(prev_position, a)) {
    if (a->direction == DIRECTION_DOWN) {
      a->pulley.rotations++;
    } else if (a->direction == DIRECTION_UP) {
      a->pulley.rotations--;
    }
    printf("%d rotations. d=%d\n", a->pulley.rotations, a->pulley.datum);
  }

  // Update the carriage vertical position
  const int position_12bit =
      -4096 * a->pulley.rotations -
      wrap(a->pulley.position - a->pulley.datum, 0, 4096);

  a->position = 6.2831853072 / 4096.0 * position_12bit * a->pulley.radius;
}

void sample_as5600_task(void* params) {
  actuator_t* actuator = (actuator_t*)params;
  esp_err_t ret = ESP_FAIL;
  uint16_t position = 0;  // angle encoded as a 12 bit value

  while (true) {
    // TODO: time out after 2 master ticks
    xSemaphoreTake(master_tick_signal, portMAX_DELAY);

    ret = i2c_read_two_bytes(AS5600_ADDR, as5600_registers, &position);
    if (ret == ESP_ERR_TIMEOUT) {
      ESP_LOGE("AS5600", "I2C timeout");
    } else if (ret == ESP_OK) {
      update_actuator(position, actuator);
      if (actuator->homed) {
        ESP_LOGI("AS5600", "%d %.2f %.2f", actuator->pulley.position,
                 actuator->pulley.rotation_rate, actuator->position);
      }
    }
  }
}

void limit_switch_task(void* params) {
  actuator_t* actuator = (actuator_t*)params;
  int pin_number = 0;
  int count = 0;

  while (true) {
    if (xQueueReceive(limit_switch_queue, &pin_number, portMAX_DELAY)) {
      // Disable limit switch ISR to prevent queuing garbage results during
      // bounce period
      gpio_isr_handler_remove(pin_number);

      // Interrupt handler logic
      if (gpio_get_level(pin_number) == 1) {
        count++;
        gpio_set_level(onboard_led_pin, 1);

        // Home the actuator
        bool already_homed = actuator->homed;
        actuator->pulley.datum = actuator->pulley.position;
        actuator->position = 0;
        actuator->pulley.rotations = 0;
        actuator->homed = true;

        ESP_LOGI("LIMIT_SWITCH", "Trigger count = %d. datum = %d. %s", count,
                 actuator->pulley.datum, already_homed ? "re-homed" : "homed");
      }
      vTaskDelay(pdMS_TO_TICKS(100));  // debounce delay

      // Reenable interrupt after debounce period
      gpio_isr_handler_add(pin_number, limit_switch_isr, (void*)pin_number);

      gpio_set_level(onboard_led_pin, 0);
    }
  }
}

void configure_onboard_led() {
  gpio_pad_select_gpio(onboard_led_pin);
  gpio_set_direction(onboard_led_pin, GPIO_MODE_OUTPUT);
}

void configure_limit_switch_pin() {
  // Limit switch (NO) connects VCC and limit_switch_pin
  gpio_config_t conf;
  conf.pin_bit_mask = (1ULL << limit_switch_pin);
  conf.mode = GPIO_MODE_INPUT;
  conf.intr_type = GPIO_INTR_POSEDGE;
  conf.pull_down_en = 1;
  conf.pull_up_en = 0;
  gpio_config(&conf);

  limit_switch_queue = xQueueCreate(1, sizeof(int));
  gpio_install_isr_service(0);
  gpio_isr_handler_add(limit_switch_pin, limit_switch_isr,
                       (void*)limit_switch_pin);
}

void configure_motor_pwm() {
  ledc_timer_config_t timer = {.speed_mode = LEDC_LOW_SPEED_MODE,
                               .duty_resolution = LEDC_TIMER_10_BIT,
                               .timer_num = LEDC_TIMER_0,
                               .freq_hz = 5000,
                               .clk_cfg = LEDC_AUTO_CLK};

  ledc_timer_config(&timer);

  ledc_channel_config_t channel = {.gpio_num = motor_pwm_pin,
                                   .speed_mode = LEDC_LOW_SPEED_MODE,
                                   .channel = LEDC_CHANNEL_0,
                                   .timer_sel = LEDC_TIMER_0,
                                   .duty = 0,
                                   .hpoint = 0};
  ledc_channel_config(&channel);

  // Enable use of functions such as ledc_set_duty_and_update,
  // ledc_set_fade_time_and_start
  ledc_fade_func_install(0);
}

static void mcpwm_task(void* arg) {
  // Config and initialization
  mcpwm_config_t pwm_config;
  pwm_config.frequency = 20000;  // Hz,
  pwm_config.cmpr_a = 0;         // duty cycle of PWMxA [%]
  pwm_config.cmpr_b = 0;         // duty cycle of PWMxb
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MOTOR_PWM_UNIT, MOTOR_PWM_TIMER, &pwm_config);
  mcpwm_gpio_init(MOTOR_PWM_UNIT, MCPWM0A, MOTOR_PWM_PIN);

  gpio_pad_select_gpio(MOTOR_DIR_PIN);
  gpio_set_direction(MOTOR_DIR_PIN, GPIO_MODE_OUTPUT);

  int direction = 0;
  while (true) {
    for (int i = 0; i < 100; i++) {
      // printf("%.2f %d\n", (float)i, direction);
      //   motor_go(direction, (float)i);
      gpio_set_level(MOTOR_DIR_PIN, direction);
      mcpwm_set_duty(MOTOR_PWM_UNIT, MOTOR_PWM_TIMER, MCPWM_OPR_A, i);

      vTaskDelay(10 / portTICK_RATE_MS);
    }
    for (int i = 100; i > 0; i--) {
      // printf("%.2f %d\n", (float)i, direction);
      gpio_set_level(MOTOR_DIR_PIN, direction);
      mcpwm_set_duty(MOTOR_PWM_UNIT, MOTOR_PWM_TIMER, MCPWM_OPR_A, i);

      //   motor_go(direction, (float)i);
      vTaskDelay(10 / portTICK_RATE_MS);
    }
    direction = !direction;
  }
}

void app_main() {
  master_tick_signal = xSemaphoreCreateBinary();

  configure_onboard_led();
  configure_limit_switch_pin();
  configure_motor_pwm();
  configure_i2c_master(I2C_NUM_0, as5600_sda, as5600_scl);

  vTaskDelay(pdMS_TO_TICKS(500));
  print_device_info();
  print_memory();

  xTaskCreate(&limit_switch_task, "limit_switch", 2048, &actuator, 5, NULL);
  xTaskCreate(&sample_as5600_task, "as5600", 2048, &actuator, 3, NULL);
  xTaskCreate(&mcpwm_task, "mcpwm", 2048, NULL, 2, NULL);

  TimerHandle_t master_timer = xTimerCreate("master_timer", update_period, true,
                                            NULL, &master_timer_callback);
  xTimerStart(master_timer, 0);
}

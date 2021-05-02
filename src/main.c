// TODO: use a mutex semaphore for UART traffic

// Builtins
#include <math.h>
#include <stdio.h>

// ESP IDF
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"

// local
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

// Sets the target sample and update period of the overall system
static const TickType_t update_period = pdMS_TO_TICKS(10);

static xQueueHandle limit_switch_queue;

xSemaphoreHandle master_tick_signal;

static void IRAM_ATTR limit_switch_isr(void* args) {
  int pin_number = (int)args;
  xQueueSendFromISR(limit_switch_queue, &pin_number, NULL);
}

typedef enum { MOVING_DOWN, MOVING_UP, DIRECTION_UNKNOWN } actuator_direction_t;

typedef struct {
  int position;         // 0-4095, datum arb. but fixed. Init to -1
  float rotation_rate;  // signed ang. speed in 12-bit units per timestep
  int rotations;        // signed rotation count
  float radius;         // [cm] Half pitch diam. For rot. to lin. conversion
} pulley_t;

typedef struct {
  pulley_t pulley;
  float position;                  // cm above limit switch
  float speed;                     // cm/s, positive upward
  actuator_direction_t direction;  // up/down/stationary
} actuator_t;

void master_timer_callback(TimerHandle_t timer) {
  xSemaphoreGive(master_tick_signal);
}

// IIR filter using exponentially weighted moving average.
// 0 <= alpha < 1 is a smoothing factor (larger -> smoother)
void filter_ema(const float alpha, const float x_meas, float* x_filt) {
  *x_filt = (1.0 - alpha) * x_meas + alpha * (*x_filt);
}

// Return x wrapped into the interval [a, b)
int wrap(const float x, const float a, const float b) {
  if (x < a) {
    return x + b - a;
  }
  if (x >= b) {
    return x - b + a;
  }
  return x;
}

// Update actuator structure from a measured 12 bit rotational position
// measurement. The AS5600 magnetic sensor occasionally reports anomalous
// samples. For sufficiently hight sample rates, the angular velocity should be
// fairly smooth. If an excursion is detected, the bad measurement is replaced
// with a dead reckoning estimate.
void update_actuator(const int position, actuator_t* a) {
  float delta = 0;  // Change from previous position
  static int num_dead_reckon_steps = 0;

  if (a->pulley.position >= 0) {  // true except at initialization
    delta = position - a->pulley.position;
    delta = wrap(delta, -2048, 2048);
  }

  if (fabs(delta - a->pulley.rotation_rate) > 20.0 &&
      num_dead_reckon_steps < 2) {
    ESP_LOGW("update_actuator", "Skip: delta - EMA = %.2f",
             delta - a->pulley.rotation_rate);
    a->pulley.position += a->pulley.rotation_rate;
    num_dead_reckon_steps++;
  } else {
    a->pulley.position = position;
    filter_ema(0.6, delta, &a->pulley.rotation_rate);
    num_dead_reckon_steps = 0;
  }
}

void sample_as5600_task(void* params) {
  esp_err_t ret = ESP_FAIL;
  uint16_t position = 0;  // angle encoded as a 12 bit value

  actuator_t* actuator = (actuator_t*)params;

  while (true) {
    // TODO: time out after 2 master ticks
    xSemaphoreTake(master_tick_signal, portMAX_DELAY);

    ret = i2c_read_two_bytes(AS5600_ADDR, as5600_registers, &position);
    if (ret == ESP_ERR_TIMEOUT) {
      ESP_LOGE("AS5600", "I2C timeout");
    } else if (ret == ESP_OK) {
      update_actuator(position, actuator);
      ESP_LOGI("AS5600", "%d %.2f", actuator->pulley.position,
               actuator->pulley.rotation_rate);
    }
  }
}

void limit_switch_task(void* params) {
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
        ESP_LOGW("LIMIT_SWITCH", "Triggered %d %s since start. GPIO %d = %d",
                 count, count == 1 ? "time" : "times", pin_number,
                 gpio_get_level(pin_number));
        gpio_set_level(onboard_led_pin, 1);
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

void app_main() {
  actuator_t actuator = {.pulley = {.position = -1,
                                    .rotation_rate = 0,
                                    .rotations = 0,
                                    .radius = 1.5 /*cm*/},
                         .position = -1,
                         .speed = 0,
                         .direction = DIRECTION_UNKNOWN};

  master_tick_signal = xSemaphoreCreateBinary();

  configure_onboard_led();
  configure_limit_switch_pin();
  configure_motor_pwm();
  configure_i2c_master(I2C_NUM_0, as5600_sda, as5600_scl);

  vTaskDelay(pdMS_TO_TICKS(500));
  print_device_info();
  print_memory();

  xTaskCreate(&limit_switch_task, "limit_switch_task", 2048, NULL, 5, NULL);
  xTaskCreate(&sample_as5600_task, "sample_as5600_task", 2048, &actuator, 2,
              NULL);

  TimerHandle_t master_timer = xTimerCreate("master_timer", update_period, true,
                                            NULL, &master_timer_callback);
  xTimerStart(master_timer, 0);
}

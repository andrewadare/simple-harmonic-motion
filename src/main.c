// std lib
#include <stdio.h>

// ESP IDF
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

// local
#include "chip_introspection.h"
#include "i2c_comms.h"

// I2C info for AMS AS5600 magnetic position sensor
#define AS5600_ADDR 0x36
uint8_t as5600_registers[2] = {0x0d, 0x0c};  // for low and high bytes

// Pin assignments on ESP32 dev board
static gpio_num_t onboard_led_pin = 2;
static gpio_num_t motor_pwm_pin = 4;
static gpio_num_t limit_switch_pin = 16;
static gpio_num_t as5600_sda = 18;
static gpio_num_t as5600_scl = 19;

static xQueueHandle limit_switch_queue;

static void IRAM_ATTR limit_switch_isr(void* args) {
  int pin_number = (int)args;
  xQueueSendFromISR(limit_switch_queue, &pin_number, NULL);
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
        ESP_LOGW("STATUS", "Limit switch triggered (%d). GPIO %d = %d", count++,
                 pin_number, gpio_get_level(pin_number));
        gpio_set_level(onboard_led_pin, 1);
      }
      vTaskDelay(pdMS_TO_TICKS(100));

      // Reenable
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
  xTaskCreate(limit_switch_task, "limit_switch_task", 2048, NULL, 1, NULL);
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
  configure_onboard_led();
  configure_limit_switch_pin();
  configure_motor_pwm();
  configure_i2c_master(I2C_NUM_0, as5600_sda, as5600_scl);

  vTaskDelay(pdMS_TO_TICKS(500));
  print_device_info();
  print_memory();

  esp_err_t ret = ESP_FAIL;
  uint16_t angle_12bit = 0;
  int led_fade_period = 1000;
  while (true) {
    // TODO move to a dedicated task
    ret = i2c_read_two_bytes(AS5600_ADDR, as5600_registers, &angle_12bit);
    if (ret == ESP_ERR_TIMEOUT) {
      ESP_LOGE("AS5600", "I2C timeout");
    } else if (ret == ESP_OK) {
      ESP_LOGI("AS5600", "%d", angle_12bit);
    }

    // Temporary - will probably use MCPWM instead of LEDC
    ledc_set_fade_time_and_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0,
                                 led_fade_period / 2, LEDC_FADE_WAIT_DONE);
    ledc_set_fade_time_and_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 1024,
                                 led_fade_period / 2, LEDC_FADE_WAIT_DONE);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

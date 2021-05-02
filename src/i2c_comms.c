#include "i2c_comms.h"

#include "esp_log.h"

// Change as needed
static i2c_port_t i2c_port = I2C_NUM_0;

// Internal pullups are 45k on ESP32. Use stronger external pullups for more
// reliable I2C.
void configure_i2c_master(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl) {
  i2c_config_t conf = {.mode = I2C_MODE_MASTER,
                       .sda_io_num = sda,
                       .scl_io_num = scl,
                       .sda_pullup_en = GPIO_PULLUP_DISABLE,
                       .scl_pullup_en = GPIO_PULLUP_DISABLE,
                       .master.clk_speed = 100000};
  i2c_param_config(i2c_num, &conf);
  i2c_driver_install(i2c_num, conf.mode, 0, 0, 0);
}

/**
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
esp_err_t i2c_write(int bus_address, uint8_t* buffer, size_t num_bytes) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (bus_address << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write(cmd, buffer, num_bytes, true);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

/**
 * Commands slave device to fill its buffer then master tries to read it.
 *
 * ____________________________________________________________________________
 * | start | slave_addr+rd_bit+ack | rd n-1 bytes+ack | rd 1 byte+nack | stop |
 * --------|-----------------------|------------------|----------------|------|
 *
 */
esp_err_t i2c_read(int bus_address, uint8_t* buffer, size_t num_bytes) {
  if (num_bytes == 0) {
    return ESP_OK;
  }
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (bus_address << 1) | I2C_MASTER_READ, true);
  if (num_bytes > 1) {
    i2c_master_read(cmd, buffer, num_bytes - 1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(cmd, buffer + num_bytes - 1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

// Convenience function to handle the common task of reading a two-byte value
// from low and high 8 bit registers, then reconstructing the result.
esp_err_t i2c_read_two_bytes(int bus_address, uint8_t* registers,
                             uint16_t* result) {
  uint8_t raw_response[2];
  esp_err_t ret = ESP_FAIL;

  for (int i = 0; i < 2; ++i) {
    ret = i2c_write(bus_address, &registers[i], 1);
    if (ret == !ESP_OK) {
      ESP_LOGE("I2C", "Write failure on byte %d in i2c_read_two_bytes", i);
      return ret;
    }
    ret = i2c_read(bus_address, &raw_response[i], 1);
    if (ret == !ESP_OK) {
      ESP_LOGE("I2C", "Read failure on byte %d in i2c_read_two_bytes", i);
      return ret;
    }
  }
  *result = raw_response[1] << 8 | raw_response[0];
  return ret;
}

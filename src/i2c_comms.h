#ifndef I2C_COMMS_H
#define I2C_COMMS_H

#include "driver/i2c.h"

void configure_i2c_master(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl);

esp_err_t i2c_write(int bus_address, uint8_t* buffer, size_t num_bytes);

esp_err_t i2c_read(int bus_address, uint8_t* buffer, size_t num_bytes);

esp_err_t i2c_read_two_bytes(int bus_address, uint8_t* registers,
                             uint16_t* result);


#endif /* I2C_COMMS_H */

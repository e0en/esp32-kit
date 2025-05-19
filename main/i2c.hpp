#ifndef I2C_HPP
#define I2C_HPP
extern "C" {
#include <driver/gpio.h>
#include <esp_err.h>
}

void init_i2c(gpio_num_t sda_pin, gpio_num_t scl_pin);

esp_err_t i2c_select_register(uint8_t register_address);
esp_err_t i2c_read(uint16_t device_address, uint8_t register_address,
                   uint8_t *target, size_t length);
esp_err_t i2c_write(uint16_t device_address, uint8_t register_address,
                    uint8_t *source, size_t length);
#endif

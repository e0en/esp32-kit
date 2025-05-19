#ifndef I2C_HPP
#define I2C_HPP
extern "C" {
#include <driver/gpio.h>
#include <esp_err.h>
}

void init_i2c(gpio_num_t sda_pin, gpio_num_t scl_pin);
#endif

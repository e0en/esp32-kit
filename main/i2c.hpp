#ifndef I2C_HPP
#define I2C_HPP
extern "C" {
#include <driver/gpio.h>
#include <driver/i2c_types.h>
#include <esp_err.h>
}

i2c_master_bus_handle_t init_i2c(gpio_num_t sda_pin, gpio_num_t scl_pin);
i2c_master_bus_handle_t get_i2c_bus();
i2c_master_dev_handle_t add_i2c_device(i2c_master_bus_handle_t bus_handle,
                                       uint16_t device_address);

esp_err_t i2c_read(i2c_master_dev_handle_t device_handle,
                   uint8_t register_address, uint8_t *target, size_t length);
esp_err_t i2c_write_byte(i2c_master_dev_handle_t device_handle,
                         uint8_t register_address, uint8_t *source);
#endif

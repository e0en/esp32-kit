#include "i2c.hpp"
extern "C" {
#include <driver/i2c.h>
}

const unsigned BAUD_RATE = 400000;

void init_i2c(gpio_num_t sda_pin, gpio_num_t scl_pin) {
  i2c_config_t i2c_config = {.mode = I2C_MODE_MASTER,
                             .sda_io_num = sda_pin,
                             .scl_io_num = scl_pin,
                             .sda_pullup_en = GPIO_PULLUP_ENABLE,
                             .scl_pullup_en = GPIO_PULLUP_ENABLE,
                             .master = {
                                 .clk_speed = BAUD_RATE,
                             }};
  ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_config));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}

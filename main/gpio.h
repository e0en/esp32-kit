#ifndef GPIO_H
#define GPIO_H

#include <driver/gpio.h>
#include <esp_err.h>

esp_err_t set_pin_mode(gpio_num_t pin, gpio_mode_t mode);

#endif

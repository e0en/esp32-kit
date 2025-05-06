#ifndef GPIO_H
#define GPIO_H

extern "C" {
#include <driver/gpio.h>
#include <esp_err.h>
}

esp_err_t set_pin_as_output(gpio_num_t pin);
esp_err_t set_pin_as_input(gpio_num_t pin);
#endif

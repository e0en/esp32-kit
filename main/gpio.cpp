#include "gpio.hpp"
#include "hal/gpio_types.h"
extern "C" {
#include <driver/gpio.h>
}

esp_err_t set_pin_as_output(gpio_num_t pin) {
  uint64_t pin_mask = 1 << pin;
  gpio_config_t config = {pin_mask, GPIO_MODE_OUTPUT, GPIO_PULLUP_ENABLE,
                          GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE};
  return gpio_config(&config);
}

esp_err_t set_pin_as_input(gpio_num_t pin) {
  uint64_t pin_mask = 1 << pin;
  gpio_config_t config = {pin_mask, GPIO_MODE_INPUT, GPIO_PULLUP_DISABLE,
                          GPIO_PULLDOWN_ENABLE, GPIO_INTR_DISABLE};
  return gpio_config(&config);
}

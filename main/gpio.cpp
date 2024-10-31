#include "gpio.hpp"
extern "C" {
#include <driver/gpio.h>
}

esp_err_t set_pin_mode(gpio_num_t pin, gpio_mode_t mode) {
  uint64_t pin_mask = 1 << pin;
  gpio_config_t config = {pin_mask, mode, GPIO_PULLUP_ENABLE,
                          GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE};
  return gpio_config(&config);
}

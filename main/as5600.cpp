#include "as5600.hpp"
#include "i2c.hpp"

extern "C" {
#include <driver/i2c_types.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <hal/gpio_types.h>
}

const float PI = 3.14159265;
const float DEGREE_TO_RADIAN = PI / 180.0;
const float GRAVITY = 9.80665;

const uint8_t REGISTER_CONF_H = 0x07;
const uint8_t REGISTER_CONF_L = 0x08;
const uint8_t REGISTER_STATUS = 0x0B;
const uint8_t REGISTER_RAW_ANGLE_H = 0x0C;
const uint8_t REGISTER_RAW_ANGLE_L = 0x0D;

AS5600::AS5600() {
  i2c_master_bus_handle_t bus_handle = get_i2c_bus();
  device_handle = add_i2c_device(bus_handle, I2C_ADDRESS);

  ESP_LOGI("INIT_AS5600", "INIT");
  ESP_ERROR_CHECK(set_digital_output_mode());
  ESP_LOGI("INIT_AS5600", "set to digital output mode");
  AS5600_MagnetStatus status = read_magnet_status();
  if (status != AS5600_MAGNET_STATUS_OK) {
    ESP_LOGE("INIT_AS5600", "Magnet status error: %d", status);
  } else {
    ESP_LOGI("INIT_AS5600", "Magnet status OK");
  }
}

esp_err_t AS5600::set_digital_output_mode() {
  uint8_t byte;
  i2c_read(device_handle, REGISTER_CONF_H, &byte, 1);
  byte &= ~(3 << 4);
  byte |= (2 << 4);
  return i2c_write_byte(device_handle, REGISTER_CONF_H, &byte);
}

uint16_t AS5600::read_raw_angle() {
  uint8_t bytes[2];
  i2c_read(device_handle, REGISTER_RAW_ANGLE_L, &bytes[0], 1);
  i2c_read(device_handle, REGISTER_RAW_ANGLE_H, &bytes[1], 1);
  return (bytes[1] << 8) + bytes[0];
}

AS5600_MagnetStatus AS5600::read_magnet_status() {
  uint8_t byte;
  i2c_read(device_handle, REGISTER_STATUS, &byte, 1);
  bool is_too_strong = (byte >> 3) % 2;
  bool is_too_weak = (byte >> 4) % 2;
  bool is_detected = (byte >> 5) % 2;

  if (is_too_strong) {
    return AS5600_MAGNET_STATUS_TOO_STRONG;
  } else if (is_too_weak) {
    return AS5600_MAGNET_STATUS_TOO_WEAK;
  } else if (!is_detected) {
    return AS5600_MAGNET_STATUS_UNDETECTED;
  }
  return AS5600_MAGNET_STATUS_OK;
}

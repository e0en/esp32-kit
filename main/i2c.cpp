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

esp_err_t i2c_select_register(uint16_t device_address,
                              uint8_t register_address) {

  esp_err_t error_code;
  i2c_cmd_handle_t cmd;

  cmd = i2c_cmd_link_create();
  ESP_ERROR_CHECK(i2c_master_start(cmd));
  ESP_ERROR_CHECK(
      i2c_master_write_byte(cmd, (device_address << 1) | I2C_MASTER_WRITE, 1));
  ESP_ERROR_CHECK(i2c_master_write_byte(cmd, register_address, 1));
  ESP_ERROR_CHECK(i2c_master_stop(cmd));
  error_code = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return error_code;
}
esp_err_t i2c_read(uint16_t device_address, uint8_t register_address,
                   uint8_t *target, size_t length) {

  esp_err_t error_code;
  i2c_cmd_handle_t cmd;
  uint8_t retry_count = 0;
  while (i2c_select_register(device_address, register_address) != ESP_OK) {
    vTaskDelay(1 / portTICK_PERIOD_MS);
    retry_count++;
    if (retry_count >= 10) {
      return ESP_ERR_INVALID_RESPONSE;
    }
  }

  cmd = i2c_cmd_link_create();
  ESP_ERROR_CHECK(i2c_master_start(cmd));
  ESP_ERROR_CHECK(
      i2c_master_write_byte(cmd, (device_address << 1) | I2C_MASTER_READ, 1));

  if (length > 1) {
    ESP_ERROR_CHECK(i2c_master_read(cmd, target, length - 1, I2C_MASTER_ACK));
  }

  ESP_ERROR_CHECK(
      i2c_master_read_byte(cmd, &target[length - 1], I2C_MASTER_NACK));

  ESP_ERROR_CHECK(i2c_master_stop(cmd));
  error_code = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return error_code;
}
esp_err_t i2c_write(uint16_t device_address, uint8_t register_address,
                    uint8_t *source, size_t length) {
  esp_err_t error_code;
  i2c_cmd_handle_t cmd;

  cmd = i2c_cmd_link_create();
  ESP_ERROR_CHECK(i2c_master_start(cmd));
  ESP_ERROR_CHECK(
      i2c_master_write_byte(cmd, (device_address << 1) | I2C_MASTER_WRITE, 1));
  ESP_ERROR_CHECK(i2c_master_write_byte(cmd, register_address, 1));

  if (length > 1) {
    ESP_ERROR_CHECK(i2c_master_write(cmd, source, length - 1, 0));
  }
  ESP_ERROR_CHECK(i2c_master_write_byte(cmd, source[length - 1], 1));
  ESP_ERROR_CHECK(i2c_master_stop(cmd));
  error_code = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return error_code;
}

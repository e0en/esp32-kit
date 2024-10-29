#include "mpu6050.h"
#include <driver/i2c.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <hal/gpio_types.h>

const uint16_t I2C_ADDRESS = 0x68;
const unsigned BAUD_RATE = 400000;

const uint8_t REGISTER_CONFIG = 0x1A;
const uint8_t REGISTER_GYRO_CONFIG = 0x1B;
const uint8_t REGISTER_ACCEL_CONFIG = 0x1C;
const uint8_t REGISTER_INTR_PIN_CFG = 0x37;
const uint8_t REGISTER_INTR_ENABLE = 0x38;
const uint8_t REGISTER_INTR_STATUS = 0x3A;
const uint8_t REGISTER_ACCEL_XOUT_H = 0x3B;
const uint8_t REGISTER_GYRO_XOUT_H = 0x43;
const uint8_t REGISTER_TEMP_XOUT_H = 0x41;
const uint8_t REGISTER_PWR_MGMT_1 = 0x6B;
const uint8_t REGISTER_WHO_AM_I = 0x75;

const unsigned SLEEP_BIT = 6;

const uint8_t ACCEL_RANGE = 0;
const uint8_t GYRO_RANGE = 0;

esp_err_t mpu6050_select_register(uint8_t register_address) {
  esp_err_t error_code;
  i2c_cmd_handle_t cmd;

  cmd = i2c_cmd_link_create();
  ESP_ERROR_CHECK(i2c_master_start(cmd));
  ESP_ERROR_CHECK(
      i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
  ESP_ERROR_CHECK(i2c_master_write_byte(cmd, register_address, 1));
  ESP_ERROR_CHECK(i2c_master_stop(cmd));
  error_code = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return error_code;
}

esp_err_t mpu6050_read(uint8_t register_address, uint8_t *target,
                       size_t length) {
  esp_err_t error_code;
  i2c_cmd_handle_t cmd;
  uint8_t retry_count = 0;
  while (mpu6050_select_register(register_address) != ESP_OK) {
    vTaskDelay(1 / portTICK_PERIOD_MS);
    retry_count++;
    if (retry_count >= 10) {
      return ESP_ERR_INVALID_RESPONSE;
    }
  }

  cmd = i2c_cmd_link_create();
  ESP_ERROR_CHECK(i2c_master_start(cmd));
  ESP_ERROR_CHECK(
      i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_READ, 1));

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

esp_err_t mpu6050_write(uint8_t register_address, uint8_t *source,
                        size_t length) {
  esp_err_t error_code;
  i2c_cmd_handle_t cmd;

  cmd = i2c_cmd_link_create();
  ESP_ERROR_CHECK(i2c_master_start(cmd));
  ESP_ERROR_CHECK(
      i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
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

esp_err_t mpu6050_whoami() {
  uint8_t byte;
  mpu6050_read(REGISTER_WHO_AM_I, &byte, 1);
  bool is_matched = (byte == I2C_ADDRESS) || (byte == 0xC);
  if (is_matched) {
    return ESP_OK;
  }
  return ESP_ERR_INVALID_STATE;
}

esp_err_t mpu6050_reset() {
  uint8_t byte;
  mpu6050_read(REGISTER_WHO_AM_I, &byte, 1);
  byte |= (1 << 7);
  mpu6050_write(REGISTER_PWR_MGMT_1, &byte, 1);
  return ESP_OK;
}

esp_err_t mpu6050_use_gyrox_clock() {
  uint8_t byte;
  mpu6050_read(REGISTER_PWR_MGMT_1, &byte, 1);
  byte >>= 3;
  byte <<= 3;
  byte |= (uint8_t)1;
  mpu6050_write(REGISTER_PWR_MGMT_1, &byte, 1);
  return ESP_OK;
}

esp_err_t mpu6050_wakeup() {
  uint8_t byte;
  mpu6050_read(REGISTER_PWR_MGMT_1, &byte, 1);
  byte &= ~(1 << SLEEP_BIT);
  mpu6050_write(REGISTER_PWR_MGMT_1, &byte, 1);
  return ESP_OK;
}

uint8_t mpu6050_get_accel_range() {
  uint8_t byte;
  mpu6050_read(REGISTER_ACCEL_CONFIG, &byte, 1);
  return (byte & 24) >> 3;
}

uint8_t mpu6050_get_gyro_range() {
  uint8_t byte;
  mpu6050_read(REGISTER_GYRO_CONFIG, &byte, 1);
  return (byte & 24) >> 3;
}

void mpu6050_set_filter_config(uint8_t flag) {
  uint8_t byte;
  mpu6050_read(REGISTER_CONFIG, &byte, 1);
  byte = ((byte >> 3) << 3) | flag;
  mpu6050_write(REGISTER_CONFIG, &byte, 1);
}

void mpu6050_set_accel_range(uint8_t range) {
  uint8_t byte = range << 3;
  mpu6050_write(REGISTER_ACCEL_CONFIG, &byte, 1);
}

void mpu6050_set_gyro_range(uint8_t range) {
  uint8_t byte = range << 3;
  mpu6050_write(REGISTER_GYRO_CONFIG, &byte, 1);
}

esp_err_t init_mpu6050() {
  mpu6050_use_gyrox_clock();
  ESP_LOGI("INIT_MPU6050", "clock");
  mpu6050_set_accel_range(ACCEL_RANGE);
  mpu6050_set_gyro_range(GYRO_RANGE);
  ESP_LOGI("INIT_MPU6050", "range");
  mpu6050_wakeup();
  ESP_LOGI("INIT_MPU6050", "wakeup");
  return ESP_OK;
}

bool mpu6050_is_data_ready() {
  uint8_t byte;
  mpu6050_read(REGISTER_INTR_STATUS, &byte, 1);
  return byte % 2;
}

void mpu6050_read_accel_gyro(struct int16_3 *accel, struct int16_3 *gyro) {
  uint8_t bytes[14];
  mpu6050_read(REGISTER_ACCEL_XOUT_H, bytes, 14);
  accel->x = (bytes[0] << 8) | bytes[1];
  accel->y = (bytes[2] << 8) | bytes[3];
  accel->z = (bytes[4] << 8) | bytes[5];
  gyro->x = (bytes[8] << 8) | bytes[9];
  gyro->y = (bytes[10] << 8) | bytes[11];
  gyro->z = (bytes[12] << 8) | bytes[13];
}

struct int16_3 mpu6050_read_accel() {
  struct int16_3 result;
  uint8_t bytes[6];
  mpu6050_read(REGISTER_ACCEL_XOUT_H, bytes, 6);
  result.x = (bytes[0] << 8) | bytes[1];
  result.y = (bytes[2] << 8) | bytes[3];
  result.z = (bytes[4] << 8) | bytes[5];
  return result;
}

struct int16_3 mpu6050_read_gyro() {
  struct int16_3 result;
  uint8_t bytes[6];
  mpu6050_read(REGISTER_GYRO_XOUT_H, bytes, 6);
  result.x = (bytes[0] << 8) | bytes[1];
  result.y = (bytes[2] << 8) | bytes[3];
  result.z = (bytes[4] << 8) | bytes[5];
  return result;
}

float mpu6050_read_temperature() {
  uint8_t bytes[2];
  mpu6050_read(REGISTER_TEMP_XOUT_H, bytes, 2);
  int16_t reading = (bytes[0] << 8) | bytes[1];
  return (float)reading / 340.0 + 36.53;
}

struct float_3 accel_to_si_unit(struct int16_3 accel, uint8_t accel_range,
                                struct int16_3 offset) {
  // convert to meters / second^2
  struct float_3 result;
  float coefficient = 2 << accel_range;
  float max_value = coefficient * 9.8;
  float multiplier = max_value / ((int16_t)1 << 15);
  result.x = ((float)accel.x - (float)offset.x) * multiplier;
  result.y = ((float)accel.y - (float)offset.y) * multiplier;
  result.z = ((float)accel.z - (float)offset.z) * multiplier;
  return result;
}

struct float_3 gyro_to_si_unit(struct int16_3 gyro, uint8_t gyro_range,
                               struct int16_3 offset) {
  // convert to degree / second
  struct float_3 result;
  float coefficient = 2 << gyro_range;
  float max_value = coefficient * 250;
  float multiplier = max_value / ((int16_t)1 << 15);
  result.x = ((float)gyro.x - (float)offset.x) * multiplier;
  result.y = ((float)gyro.y - (float)offset.y) * multiplier;
  result.z = ((float)gyro.z - (float)offset.z) * multiplier;
  return result;
}

struct int16_3 mpu6050_calibrate_accel() {
  struct int16_3 accel, offset;
  struct float_3 mean;
  mean.x = 0;
  mean.y = 0;
  mean.z = 0;

  bool is_ready;
  int16_t one_g = 1 << (14 - ACCEL_RANGE);

  size_t sample_count = 0;
  while (sample_count < 200) {
    is_ready = mpu6050_is_data_ready();
    if (is_ready) {
      accel = mpu6050_read_accel();
      mean.x += accel.x;
      mean.y += accel.y;
      mean.z += accel.z;
      sample_count++;
    }
  }
  mean.x /= sample_count;
  mean.y /= sample_count;
  mean.z /= sample_count;
  offset.x = (int16_t)mean.x;
  offset.y = (int16_t)mean.y;
  offset.z = (int16_t)mean.z - one_g;
  return offset;
}

struct int16_3 mpu6050_calibrate_gyro() {
  struct int16_3 gyro, offset;
  struct float_3 mean;
  mean.x = 0;
  mean.y = 0;
  mean.z = 0;

  bool is_ready = false;
  size_t sample_count = 0;
  while (sample_count < 100) {
    is_ready = mpu6050_is_data_ready();
    if (!is_ready) {
      continue;
    }
    gyro = mpu6050_read_gyro();
    mean.x += gyro.x;
    mean.y += gyro.y;
    mean.z += gyro.z;
    sample_count++;
  }
  mean.x /= sample_count;
  mean.y /= sample_count;
  mean.z /= sample_count;
  offset.x = (int16_t)mean.x;
  offset.y = (int16_t)mean.y;
  offset.z = (int16_t)mean.z;
  return offset;
}

void init_i2c(gpio_num_t sda_pin, gpio_num_t scl_pin) {
  i2c_config_t i2c_config = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = sda_pin,
      .scl_io_num = scl_pin,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = BAUD_RATE,
  };
  ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_config));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}

#include "mpu6050.hpp"

extern "C" {
#include <driver/i2c.h>
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

const float ACCEL_COEFFICIENT = GRAVITY / (1 << 15);
const float GYRO_COEFFICIENT = DEGREE_TO_RADIAN / (1 << 15);

const uint16_t I2C_ADDRESS = 0x68;

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
const uint8_t GYRO_RANGE = 2;

esp_err_t MPU6050::select_register(uint8_t register_address) {
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

esp_err_t MPU6050::read(uint8_t register_address, uint8_t *target,
                        size_t length) {
  esp_err_t error_code;
  i2c_cmd_handle_t cmd;
  uint8_t retry_count = 0;
  while (select_register(register_address) != ESP_OK) {
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

esp_err_t MPU6050::write(uint8_t register_address, uint8_t *source,
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

esp_err_t MPU6050::whoami() {
  uint8_t byte;
  read(REGISTER_WHO_AM_I, &byte, 1);
  bool is_matched = (byte == I2C_ADDRESS) || (byte == 0xC);
  if (is_matched) {
    return ESP_OK;
  }
  return ESP_ERR_INVALID_STATE;
}

esp_err_t MPU6050::reset() {
  uint8_t byte;
  read(REGISTER_WHO_AM_I, &byte, 1);
  byte |= (1 << 7);
  write(REGISTER_PWR_MGMT_1, &byte, 1);
  return ESP_OK;
}

esp_err_t MPU6050::use_gyrox_clock() {
  uint8_t byte;
  read(REGISTER_PWR_MGMT_1, &byte, 1);
  byte >>= 3;
  byte <<= 3;
  byte |= (uint8_t)1;
  write(REGISTER_PWR_MGMT_1, &byte, 1);
  return ESP_OK;
}

esp_err_t MPU6050::wakeup() {
  uint8_t byte;
  read(REGISTER_PWR_MGMT_1, &byte, 1);
  byte &= ~(1 << SLEEP_BIT);
  write(REGISTER_PWR_MGMT_1, &byte, 1);
  return ESP_OK;
}

uint8_t MPU6050::get_accel_range() {
  uint8_t byte;
  read(REGISTER_ACCEL_CONFIG, &byte, 1);
  return (byte & 24) >> 3;
}

uint8_t MPU6050::get_gyro_range() {
  uint8_t byte;
  read(REGISTER_GYRO_CONFIG, &byte, 1);
  return (byte & 24) >> 3;
}

void MPU6050::set_filter_config(uint8_t flag) {
  uint8_t byte;
  read(REGISTER_CONFIG, &byte, 1);
  byte = ((byte >> 3) << 3) | flag;
  write(REGISTER_CONFIG, &byte, 1);
}

void MPU6050::set_accel_range(uint8_t range) {
  uint8_t byte = range << 3;
  accel_range = 2 << range;
  write(REGISTER_ACCEL_CONFIG, &byte, 1);
}

void MPU6050::set_gyro_range(uint8_t range) {
  uint8_t byte = range << 3;
  gyro_range = 250 << range;
  write(REGISTER_GYRO_CONFIG, &byte, 1);
}

MPU6050::MPU6050() {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
  i2c_master_stop(cmd);

  esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
  i2c_cmd_link_delete(cmd);
  ESP_ERROR_CHECK(ret);

  ESP_ERROR_CHECK(whoami());
  use_gyrox_clock();
  ESP_LOGI("INIT_MPU6050", "clock");
  set_accel_range(ACCEL_RANGE);
  set_gyro_range(GYRO_RANGE);
  ESP_LOGI("INIT_MPU6050", "range");
  wakeup();
  ESP_LOGI("INIT_MPU6050", "wakeup");
}

bool MPU6050::is_data_ready() {
  uint8_t byte;
  read(REGISTER_INTR_STATUS, &byte, 1);
  return byte % 2;
}

void MPU6050::read_accel_gyro(struct int16_3 *accel, struct int16_3 *gyro) {
  uint8_t bytes[14];
  read(REGISTER_ACCEL_XOUT_H, bytes, 14);
  accel->x = (int16_t)((bytes[0] << 8) | bytes[1]);
  accel->y = (int16_t)((bytes[2] << 8) | bytes[3]);
  accel->z = (int16_t)((bytes[4] << 8) | bytes[5]);
  gyro->x = (int16_t)((bytes[8] << 8) | bytes[9]) - gyro_offset.x;
  gyro->y = (int16_t)((bytes[10] << 8) | bytes[11]) - gyro_offset.y;
  gyro->z = (int16_t)((bytes[12] << 8) | bytes[13]) - gyro_offset.z;
}

void MPU6050::read_accel_gyro_si(struct Vector3 *accel, struct Vector3 *gyro) {
  int16_3 accel_raw, gyro_raw;
  read_accel_gyro(&accel_raw, &gyro_raw);
  accel->x = static_cast<float>(accel_raw.x) * accel_range * ACCEL_COEFFICIENT;
  accel->y = static_cast<float>(accel_raw.y) * accel_range * ACCEL_COEFFICIENT;
  accel->z = static_cast<float>(accel_raw.z) * accel_range * ACCEL_COEFFICIENT;

  gyro->x = static_cast<float>(gyro_raw.x) * gyro_range * GYRO_COEFFICIENT;
  gyro->y = static_cast<float>(gyro_raw.y) * gyro_range * GYRO_COEFFICIENT;
  gyro->z = static_cast<float>(gyro_raw.z) * gyro_range * GYRO_COEFFICIENT;
}

struct int16_3 MPU6050::read_accel() {
  struct int16_3 result;
  uint8_t bytes[6];
  read(REGISTER_ACCEL_XOUT_H, bytes, 6);
  result.x = (bytes[0] << 8) | bytes[1];
  result.y = (bytes[2] << 8) | bytes[3];
  result.z = (bytes[4] << 8) | bytes[5];
  return result;
}

struct int16_3 MPU6050::read_gyro() {
  struct int16_3 result;
  uint8_t bytes[6];
  read(REGISTER_GYRO_XOUT_H, bytes, 6);
  result.x = ((bytes[0] << 8) | bytes[1]);
  result.y = ((bytes[2] << 8) | bytes[3]);
  result.z = ((bytes[4] << 8) | bytes[5]);
  return result;
}

float MPU6050::read_temperature() {
  uint8_t bytes[2];
  read(REGISTER_TEMP_XOUT_H, bytes, 2);
  int16_t reading = (bytes[0] << 8) | bytes[1];
  return (float)reading / 340.0 + 36.53;
}

struct int16_3 MPU6050::calibrate_accel() {
  struct int16_3 accel, offset;
  struct Vector3 mean;
  mean.x = 0;
  mean.y = 0;
  mean.z = 0;

  bool is_ready;
  int16_t one_g = 1 << (14 - ACCEL_RANGE);

  size_t sample_count = 0;
  while (sample_count < 200) {
    is_ready = is_data_ready();
    if (is_ready) {
      accel = read_accel();
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

void MPU6050::calibrate_gyro() {
  struct int16_3 gyro;
  struct Vector3 mean = {0, 0, 0};

  bool is_ready = false;
  size_t sample_count = 0;
  while (sample_count < 300) {
    is_ready = is_data_ready();
    if (!is_ready) {
      continue;
    }
    gyro = read_gyro();
    mean.x += gyro.x;
    mean.y += gyro.y;
    mean.z += gyro.z;
    sample_count++;
  }
  mean.x /= sample_count;
  mean.y /= sample_count;
  mean.z /= sample_count;

  gyro_offset = {
      (int16_t)(mean.x),
      (int16_t)(mean.y),
      (int16_t)(mean.z),
  };
}

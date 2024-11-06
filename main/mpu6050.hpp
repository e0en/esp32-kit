#ifndef MPU6050_H
#define MPU6050_H

extern "C" {
#include <driver/gpio.h>
#include <esp_err.h>
}

struct int16_3 {
  int16_t x;
  int16_t y;
  int16_t z;
};

struct float_3 {
  float x;
  float y;
  float z;
};

class MPU6050 {
public:
  MPU6050();

  uint8_t get_accel_range();
  uint8_t get_gyro_range();
  void set_filter_config(uint8_t flag);

  void set_accel_range(uint8_t range);
  void set_gyro_range(uint8_t range);

  bool is_data_ready();

  void read_accel_gyro(struct int16_3 *accel, struct int16_3 *gyro);
  void read_accel_gyro_si(struct float_3 *accel, struct float_3 *gyro);
  struct int16_3 read_accel();
  struct int16_3 read_gyro();
  float read_temperature();

  struct int16_3 calibrate_accel();
  void calibrate_gyro();

private:
  int16_3 gyro_offset = {0, 0, 0};
  int16_t accel_range = 2;  // unit = g
  int16_t gyro_range = 250; // unit = degree / second

  esp_err_t select_register(uint8_t register_address);
  esp_err_t read(uint8_t register_address, uint8_t *target, size_t length);
  esp_err_t write(uint8_t register_address, uint8_t *source, size_t length);
  esp_err_t whoami();
  esp_err_t reset();
  esp_err_t use_gyrox_clock();
  esp_err_t wakeup();
};

void init_i2c(gpio_num_t sda_pin, gpio_num_t scl_pin);

bool mpu6050_is_data_ready();

#endif

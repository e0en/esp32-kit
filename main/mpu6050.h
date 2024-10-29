#ifndef MPU6050_H
#define MPU6050_H
#include <driver/gpio.h>
#include <esp_err.h>

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

esp_err_t init_mpu6050();
void init_i2c(gpio_num_t sda_pin, gpio_num_t scl_pin);

bool mpu6050_is_data_ready();

struct int16_3 mpu6050_read_gyro();
struct int16_3 mpu6050_read_accel();

#endif

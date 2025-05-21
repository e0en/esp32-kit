#include "i2c.hpp"
#include "driver/i2c_types.h"
#include "hal/i2c_types.h"
#include "soc/clk_tree_defs.h"

extern "C" {
#include <driver/i2c_master.h>
}

const unsigned BAUD_RATE = 400000;
const i2c_port_t I2C_PORT = I2C_NUM_0;

i2c_master_bus_handle_t init_i2c(gpio_num_t sda_pin, gpio_num_t scl_pin) {
  i2c_master_bus_config_t bus_config;
  bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_config.i2c_port = I2C_PORT;
  bus_config.sda_io_num = sda_pin;
  bus_config.scl_io_num = scl_pin;
  bus_config.glitch_ignore_cnt = 7;
  bus_config.trans_queue_depth = 0;
  bus_config.flags = {
      .enable_internal_pullup = true,
      .allow_pd = false,
  };

  i2c_master_bus_handle_t bus_handle;
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
  return bus_handle;
}

i2c_master_bus_handle_t get_i2c_bus() {
  i2c_master_bus_handle_t bus_handle;
  i2c_master_get_bus_handle(I2C_PORT, &bus_handle);
  return bus_handle;
}

i2c_master_dev_handle_t add_i2c_device(i2c_master_bus_handle_t bus_handle,
                                       uint16_t device_address) {
  i2c_device_config_t dev_config;
  dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev_config.device_address = device_address;
  dev_config.scl_speed_hz = BAUD_RATE;
  dev_config.scl_wait_us = 0;
  dev_config.flags = {
      .disable_ack_check = true,
  };

  i2c_master_dev_handle_t dev_handle;
  i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle);
  return dev_handle;
}

bool detect_i2c_device(i2c_master_bus_handle_t bus_handle,
                       uint16_t device_address) {
  auto ret = i2c_master_probe(bus_handle, device_address, 500);
  return ret == ESP_OK;
}

esp_err_t i2c_read(i2c_master_dev_handle_t device_handle,
                   uint8_t register_address, uint8_t *target, size_t length) {
  return i2c_master_transmit_receive(device_handle, &register_address, 1,
                                     target, length, 500);
}

esp_err_t i2c_write_byte(i2c_master_dev_handle_t device_handle,
                         uint8_t register_address, uint8_t *source) {
  uint8_t data[2] = {register_address, *source};
  return i2c_master_transmit(device_handle, data, 2, -1);
}

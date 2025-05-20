#ifndef AS5600_H
#define AS5600_H

extern "C" {
#include <driver/gpio.h>
#include <driver/i2c_types.h>
#include <esp_err.h>
}

enum AS5600_MagnetStatus {
  AS5600_MAGNET_STATUS_OK = 0,
  AS5600_MAGNET_STATUS_UNDETECTED = 1,
  AS5600_MAGNET_STATUS_TOO_STRONG = 2,
  AS5600_MAGNET_STATUS_TOO_WEAK = 3,
};

class AS5600 {
public:
  AS5600();
  uint16_t read_raw_angle();
  AS5600_MagnetStatus read_magnet_status();

private:
  i2c_master_dev_handle_t device_handle;
  esp_err_t set_digital_output_mode();
};

#endif

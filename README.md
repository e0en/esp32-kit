# ESP32 Sensor Kit

A flexible starting point for ESP32-based sensor projects. This repository provides a foundation for developing ESP32 applications that collect sensor data, process it, and transmit it via MQTT over WiFi.

## Overview

This project serves as a starting point for future ESP32-based projects. It provides a modular architecture for:

- Collecting data from various I2C sensors
- Processing sensor data using fusion algorithms
- Transmitting processed data to an MQTT broker
- Providing status information via LED indicators

The codebase is designed to be extendable and adaptable to various sensor-based applications.

## Features

- **Sensor Auto-detection**: Only runs tasks for connected sensors
- **Multi-sensor Support**: Currently supports MPU6050 IMU and AS5600 magnetic angle sensor
- **Sensor Fusion**: Implements complementary filter for IMU data
- **MQTT Communication**: Publishes processed sensor data to configured broker
- **Time Synchronization**: Uses SNTP for accurate timestamps
- **Status Indication**: LEDs indicate system state (connection, errors)
- **Resource Management**: Mutex-based I2C bus sharing between sensors
- **Task-based Architecture**: FreeRTOS tasks for concurrent operation

## Hardware Requirements

- ESP32 microcontroller
- MPU6050 IMU sensor (optional, auto-detected)
- AS5600 magnetic angle sensor (optional, auto-detected)
- Status LEDs (connected to GPIO8 and GPIO10)
- WiFi connectivity
- MQTT broker (external)

## Pin Configuration

- I2C Bus: SDA=GPIO6, SCL=GPIO7
- Status LEDs: Green=GPIO8, Red=GPIO10

## Getting Started

### Prerequisites

1. [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html) development environment
2. ESP32 development board
3. Supported sensors (MPU6050 and/or AS5600)

### Configuration

Create a `main/secret.h` file with your WiFi and MQTT credentials:

```c
#ifndef SECRET_H
#define SECRET_H

#define WIFI_SSID "your_wifi_ssid"
#define WIFI_PASSWORD "your_wifi_password"
#define MQTT_URI "mqtt://your_mqtt_broker_ip"
#define MQTT_USERNAME "your_mqtt_username"
#define MQTT_PASSWORD "your_mqtt_password"

#endif
```

### Building and Flashing

```bash
# Set up ESP-IDF environment (may vary based on your installation)
. $IDF_PATH/export.sh

# Build the project
idf.py build

# Flash to ESP32
idf.py -p [PORT] flash

# Monitor output
idf.py -p [PORT] monitor
```

## Project Structure

- **Sensor Drivers**:
  - `mpu6050.hpp/cpp`: Driver for MPU6050 IMU
  - `as5600.hpp/cpp`: Driver for AS5600 magnetic angle sensor
  - `imu.hpp/cpp`: Sensor fusion algorithms

- **Communication**:
  - `i2c.hpp/cpp`: I2C bus communication
  - `wifi.hpp/cpp`: WiFi connection management
  - `mqtt.hpp/cpp`: MQTT client implementation
  - `sntp.hpp/cpp`: Time synchronization

- **Utilities**:
  - `gpio.hpp/cpp`: GPIO control for LEDs
  - `vector_math.hpp/cpp`: 3D vector and quaternion operations

## Extending the Project

This codebase is designed as a starting point for your own ESP32-based sensor projects. Some ways to extend it:

1. Add support for additional sensors
2. Implement different processing algorithms
3. Add local data storage (SD card, SPIFFS)
4. Create a web interface for configuration
5. Implement different communication protocols

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
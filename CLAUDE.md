# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an ESP32-based sensor platform that collects data from multiple sensors (MPU6050 IMU and AS5600 magnetic angle sensor), processes it, and transmits readings via MQTT over WiFi. The project is built using the Espressif IoT Development Framework (ESP-IDF).

## Build Commands

### Environment Setup
1. Install ESP-IDF following the [official instructions](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html)
2. Set up ESP-IDF environment variables:
   ```
   . $IDF_PATH/export.sh
   ```

### Configuration
Before building, create a `main/secret.h` file (gitignored) with your credentials:
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

### Build and Flash Commands
```bash
# Configure project (only needed once or when changing config)
idf.py menuconfig

# Build the project
idf.py build

# Flash to ESP32
idf.py -p [PORT] flash

# Monitor serial output
idf.py -p [PORT] monitor

# Build, flash and monitor in one command
idf.py -p [PORT] flash monitor
```

## Code Architecture

### Main Components

1. **Sensor Handlers**:
   - `mpu6050.hpp/cpp`: Driver for MPU6050 IMU (accelerometer/gyroscope)
   - `as5600.hpp/cpp`: Driver for AS5600 magnetic angle sensor
   - `imu.hpp/cpp`: Higher-level sensor fusion and processing

2. **Communication**:
   - `i2c.hpp/cpp`: I2C bus communication with sensors
   - `wifi.hpp/cpp`: WiFi connection management
   - `mqtt.hpp/cpp`: MQTT client for publishing sensor data
   - `sntp.hpp/cpp`: Time synchronization for accurate timestamps

3. **Utilities**:
   - `gpio.hpp/cpp`: GPIO control (primarily for status LEDs)
   - `vector_math.hpp/cpp`: 3D vector operations and quaternion math

### Data Flow

1. The main application initializes hardware interfaces and peripheral modules
2. Sensors are auto-detected on the I2C bus at startup
3. FreeRTOS tasks are created for each detected sensor
4. Sensor tasks read data at specific intervals
5. IMU data is processed using sensor fusion algorithms
6. Processed data is formatted and published via MQTT
7. Status LEDs indicate system state (green for connected, red for errors)

### Key Design Patterns

1. **Task-based Architecture**: Uses FreeRTOS tasks for concurrent operation
2. **Auto-detection Pattern**: Code only runs tasks for connected sensors
3. **Mutex-based Resource Sharing**: Prevents I2C bus conflicts
4. **Publisher Pattern**: Sensor data is published to MQTT topics
5. **Error Handling**: Extensive use of ESP-IDF error checking

## Hardware Configuration

- **I2C Bus**: SDA=GPIO6, SCL=GPIO7
- **Status LEDs**: Green=GPIO8, Red=GPIO10
- **Sensors**: MPU6050 (IMU) and AS5600 (magnetic angle) on I2C bus
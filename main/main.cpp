extern "C" {
#include <esp_err.h>
#include <esp_event_base.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <mqtt_client.h>
#include <nvs_flash.h>
}

#include "gpio.hpp"
#include "imu.hpp"
#include "mpu6050.hpp"
#include "mqtt.hpp"
#include "secret.h"
#include "sntp.hpp"
#include "vector_math.hpp"
#include "wifi.hpp"

#define MQTT_TOPIC "/esp32-kit/imu6"

gpio_num_t GREEN_LED_PIN = GPIO_NUM_8;
gpio_num_t RED_LED_PIN = GPIO_NUM_10;

SemaphoreHandle_t imu_mutex = xSemaphoreCreateMutex();
struct Vector3 gyro;
struct Vector3 accel;
struct Quaternion cf;

struct MQTTPayload {
  Vector3 gyro;
  Vector3 accel;
  Quaternion cf;
  int64_t timestamp_microseconds;
};

void initialize() {
  ESP_ERROR_CHECK(set_pin_as_output(GREEN_LED_PIN));
  ESP_ERROR_CHECK(set_pin_as_output(RED_LED_PIN));

  gpio_set_level(GREEN_LED_PIN, 0);
  gpio_set_level(RED_LED_PIN, 0);

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }

  esp_event_loop_create_default();

  esp_netif_init();
  init_wifi(WIFI_SSID, WIFI_PASSWORD);
  ESP_LOGI("INIT", "WiFi");
  init_sntp();
  ESP_LOGI("INIT", "TIME");
  init_i2c(GPIO_NUM_6, GPIO_NUM_7);
  ESP_LOGI("INIT", "I2C");
  ESP_LOGI("INIT", "finished");
  gpio_set_level(GREEN_LED_PIN, 1);
}

void imu_task(void *pvParameters) {
  auto imu = MPU6050();
  imu.calibrate_gyro();
  gyro = {0.0, 0.0, 0.0};
  accel = {0.0, 0.0, 0.0};
  cf = {0.0, 0.0, 0.0, 1.0};

  float tau = 0.98;
  int64_t now = get_timestamp_microseconds();
  int64_t prev = now;
  float dt = 0.0;
  while (1) {
    if (imu.is_data_ready()) {
      now = get_timestamp_microseconds();
      dt = (now - prev) / 1e6;
      if (dt < 1e-4) {
        continue;
      }
      prev = now;
      if (xSemaphoreTake(imu_mutex, portMAX_DELAY) == pdTRUE) {
        imu.read_accel_gyro_si(&accel, &gyro);
        cf = complementary_filter(cf, gyro, accel, dt, tau);
        xSemaphoreGive(imu_mutex);
      }
    }
  }
  vTaskDelete(NULL);
}

void mqtt_task(void *pvParameters) {
  MQTTPayload buffer;
  MQTTClient mqtt(MQTT_URI, "esp32-kit", MQTT_USERNAME, MQTT_PASSWORD);
  ESP_ERROR_CHECK(mqtt.connect());
  int64_t now = 0;
  while (1) {
    now = get_timestamp_microseconds();
    if (xSemaphoreTake(imu_mutex, portMAX_DELAY) == pdTRUE) {
      buffer = {gyro, accel, cf, now};
      xSemaphoreGive(imu_mutex);
    }
    mqtt.publish(MQTT_TOPIC, reinterpret_cast<char *>(&buffer), sizeof(buffer),
                 0);
    ESP_LOGI("MQTT", "Sent (%.3f, %.3f, %.3f: %.3f) %llu", (double)cf.x,
             (double)cf.y, (double)cf.z, (double)cf.w, now);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}

extern "C" void app_main(void) {
  initialize();
  xTaskCreate(imu_task, "imu", 8192, NULL, 5, NULL);
  xTaskCreate(mqtt_task, "mqtt", 8192, NULL, 5, NULL);
}

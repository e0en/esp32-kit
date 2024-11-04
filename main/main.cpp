extern "C" {
#include <esp_err.h>
#include <esp_event_base.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <mqtt_client.h>
#include <nvs_flash.h>
}

#include "gpio.hpp"
#include "mpu6050.hpp"
#include "mqtt.hpp"
#include "secret.h"
#include "wifi.hpp"

#define MQTT_TOPIC "/esp32-kit/imu6"

gpio_num_t GREEN_LED_PIN = GPIO_NUM_8;
gpio_num_t RED_LED_PIN = GPIO_NUM_9;

struct int16_3 gyro;
struct int16_3 accel;

struct MQTTPayload {
  int16_t gx;
  int16_t gy;
  int16_t gz;
  int16_t ax;
  int16_t ay;
  int16_t az;
};

void initialize() {
  ESP_ERROR_CHECK(set_pin_mode(GREEN_LED_PIN, GPIO_MODE_OUTPUT));
  ESP_ERROR_CHECK(set_pin_mode(RED_LED_PIN, GPIO_MODE_OUTPUT));

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
  init_i2c(GPIO_NUM_6, GPIO_NUM_7);
  ESP_LOGI("INIT", "I2C");
  ESP_LOGI("INIT", "finished");
}

void led_task(void *pvParameters) {
  while (1) {
    gpio_set_level(GREEN_LED_PIN, 0);
    gpio_set_level(RED_LED_PIN, 1);
    vTaskDelay(250 / portTICK_PERIOD_MS);
    gpio_set_level(GREEN_LED_PIN, 1);
    gpio_set_level(RED_LED_PIN, 0);
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}

void imu_task(void *pvParameters) {
  auto imu = MPU6050();
  gyro.x = 0;
  gyro.y = 0;
  gyro.z = 0;

  accel.x = 0;
  accel.y = 0;
  accel.z = 0;

  while (1) {
    if (imu.is_data_ready()) {
      imu.read_accel_gyro(&accel, &gyro);
    }
  }
  vTaskDelete(NULL);
}

void mqtt_task(void *pvParameters) {
  MQTTPayload buffer;
  MQTTClient mqtt(MQTT_URI, "esp32-kit", MQTT_USERNAME, MQTT_PASSWORD);
  ESP_ERROR_CHECK(mqtt.connect());
  while (1) {
    buffer = {
        gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z,
    };
    mqtt.publish(MQTT_TOPIC, reinterpret_cast<char *>(&buffer), 0);
    ESP_LOGI("MQTT", "sent %d, %d, %d, %d, %d, %d", gyro.x, gyro.y, gyro.z,
             accel.x, accel.y, accel.z);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}

extern "C" void app_main(void) {
  initialize();
  xTaskCreate(led_task, "led", 8192, NULL, 5, NULL);
  xTaskCreate(imu_task, "imu", 8192, NULL, 5, NULL);
  xTaskCreate(mqtt_task, "mqtt", 8192, NULL, 5, NULL);
}

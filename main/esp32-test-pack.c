#include <esp_err.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <mqtt5_client.h>
#include <nvs_flash.h>

#include "esp_event_base.h"
#include "gpio.h"
#include "mpu6050.h"
#include "mqtt_client.h"
#include "secret.h"
#include "wifi.h"

gpio_num_t GREEN_LED_PIN = GPIO_NUM_8;
gpio_num_t RED_LED_PIN = GPIO_NUM_9;

struct int16_3 gyro;
struct int16_3 accel;

static void event_handler(void *handler_args, esp_event_base_t base,
                          int32_t event_id, void *event_data) {
  esp_mqtt_event_handle_t event = event_data;
  switch ((esp_mqtt_event_id_t)event_id) {
  case MQTT_EVENT_CONNECTED:
    break;
  case MQTT_EVENT_DISCONNECTED:
    break;
  case MQTT_EVENT_PUBLISHED:
    break;
  default:
    break;
  }
}

esp_err_t publish(esp_mqtt5_client_handle_t client, const char *topic,
                  const char *payload) {
  esp_mqtt_client_publish(client, topic, payload, strlen(payload), 0, false);
}

esp_mqtt5_client_handle_t init_mqtt() {
  const esp_mqtt_client_config_t config = {
      .broker = {.address =
                     {
                         .uri = MQTT_URI,
                     }},
      .credentials = {.username = MQTT_USER,
                      .authentication =
                          {
                              .password = MQTT_PASSWORD,
                          }},
      .session = {
          .protocol_ver = MQTT_PROTOCOL_V_5,
      }};
  esp_mqtt5_client_handle_t client = esp_mqtt_client_init(&config);
  esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, event_handler, NULL);
  esp_mqtt_client_start(client);
  return client;
}

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
  init_mpu6050();
  ESP_LOGI("INIT", "finished");

  init_mqtt();
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
  gyro.x = 0;
  gyro.y = 0;
  gyro.z = 0;

  accel.x = 0;
  accel.y = 0;
  accel.z = 0;

  while (1) {
    if (mpu6050_is_data_ready()) {
      gyro = mpu6050_read_gyro();
      accel = mpu6050_read_accel();
    }
  }
  vTaskDelete(NULL);
}

void mqtt_task(void *pvParameters) { vTaskDelete(NULL); }

void app_main(void) {
  initialize();
  xTaskCreate(led_task, "led", 8192, NULL, 5, NULL);
  xTaskCreate(imu_task, "imu", 8192, NULL, 5, NULL);
}

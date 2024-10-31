#include "wifi.hpp"

extern "C" {
#include <esp_event.h>
#include <esp_log.h>
#include <esp_netif_ip_addr.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <string.h>
}

const size_t WIFI_MAX_RETRY = 10;
static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  const char *TAG = "WiFiHandler";
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
    ESP_LOGI(TAG, "station started");
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    if (s_retry_num < WIFI_MAX_RETRY) {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGW(TAG, "retrying connection for %d times", s_retry_num);
    } else {
      xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
    ESP_LOGE(TAG, "connection failed");
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "connected, got IP address = " IPSTR,
             IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

void init_wifi(const char *ssid, const char *password) {
  const char *TAG = "WiFi";

  esp_netif_create_default_wifi_sta();

  s_wifi_event_group = xEventGroupCreate();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                      &wifi_event_handler, NULL,
                                      &instance_any_id);
  esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                      &wifi_event_handler, NULL,
                                      &instance_got_ip);

  wifi_config_t wifi_config = {.sta = {.ssid = {},
                                       .password = {},
                                       .threshold = {
                                           .authmode = WIFI_AUTH_OPEN,
                                       }}};
  unsigned long ssid_len = strlen(ssid);
  unsigned long password_len = strlen(password);
  memcpy(&wifi_config.sta.ssid, (const void *)ssid, ssid_len + 1);
  memcpy(&wifi_config.sta.password, (const void *)password, password_len + 1);

  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
  esp_wifi_start();

  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE, pdFALSE, portMAX_DELAY);

  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", ssid, password);
  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s", ssid, password);
  } else {
    ESP_LOGE(TAG, "UNEXPECTED EVENT");
  }
}

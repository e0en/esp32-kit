#include "mqtt.hpp"
#include <esp_err.h>
#include <esp_timer.h>

MQTTClient::MQTTClient(const char *uri, const char *client_id,
                       const char *username, const char *password) {
  is_connected_ = false;
  const esp_mqtt_client_config_t config = {
      .broker = {.address =
                     {
                         .uri = uri,
                     }},
      .credentials = {.username = username,
                      .client_id = client_id,
                      .authentication =
                          {
                              .password = password,
                          }},
      .session =
          {
              .protocol_ver = MQTT_PROTOCOL_V_3_1_1,
          },
  };

  client_ = esp_mqtt_client_init(&config);
  esp_mqtt_client_register_event(client_, MQTT_EVENT_ANY,
                                 &MQTTClient::event_handler, this);
}

esp_err_t MQTTClient::connect() {
  esp_mqtt_client_start(client_);
  auto t0 = esp_timer_get_time();

  while ((esp_timer_get_time() - t0) < timeout_us_) {
    if (is_connected_) {
      return ESP_OK;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  return ESP_ERR_TIMEOUT;
}

esp_err_t MQTTClient::publish(const char *topic, const char *payload,
                              size_t length, int qos) {
  esp_mqtt_client_publish(client_, topic, payload, length, qos, false);
  return ESP_OK;
}

void MQTTClient::subscribe(const char *topic, MessageHandler callback) {
  for (size_t i = 0; i < MAX_TOPIC_COUNT; i++) {
    if (!handlers_[i].active) {
      strcpy(handlers_[i].topic, topic);
      handlers_[i].handler = callback;
      handlers_[i].active = true;
    }
  }
  esp_mqtt_client_subscribe(client_, topic, 1);
}

void MQTTClient::event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
  auto client = static_cast<MQTTClient *>(handler_args);
  auto event = static_cast<esp_mqtt_event_handle_t>(event_data);

  switch (event_id) {
  case MQTT_EVENT_CONNECTED:
    client->is_connected_ = true;
    break;

  case MQTT_EVENT_DISCONNECTED:
    break;

  case MQTT_EVENT_DATA:
    char *topic = event->topic;
    for (size_t i = 0; i < MAX_TOPIC_COUNT; i++) {
      if (strcmp(client->handlers_[i].topic, topic) == 0) {
        client->handlers_[i].handler(client, event->data, event->data_len);
      }
    }
    break;
  }
}

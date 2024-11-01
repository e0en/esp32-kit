#include "mqtt.hpp"
#include <cstring>

MQTTClient::MQTTClient(const char *uri, const char *username,
                       const char *password) {
  const esp_mqtt_client_config_t config = {
      .broker = {.address =
                     {
                         .uri = uri,
                     }},
      .credentials = {.username = username,
                      .authentication =
                          {
                              .password = password,
                          }},
      .session = {
          .protocol_ver = MQTT_PROTOCOL_V_3_1_1,
      }};

  client_ = esp_mqtt_client_init(&config);
  esp_mqtt_client_register_event(client_, MQTT_EVENT_ANY,
                                 &MQTTClient::event_handler, NULL);
  esp_mqtt_client_start(client_);
}

void MQTTClient::event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
  auto event = static_cast<esp_mqtt_event_handle_t>(event_data);

  switch (event_id) {
  case MQTT_EVENT_CONNECTED:
    if (connect_handler_) {
      connect_handler_();
    }
    if (true) {
      esp_mqtt_client_subscribe(client_, "", 0);
    }
    break;

  case MQTT_EVENT_DISCONNECTED:
    if (disconnect_handler_) {
      disconnect_handler_();
    }
    break;

  case MQTT_EVENT_DATA:
    // Create string_view for topic and data
    auto topic = std::string(event->topic);
    auto data = std::string(event->data);
    message_handler_(event->topic, event->data);
    break;
  }
}

esp_err_t MQTTClient::publish(const char *topic, const char *payload, int qos) {
  esp_mqtt_client_publish(client_, topic, payload, strlen(payload), qos, false);
  return ESP_OK;
}

esp_err_t MQTTClient::subscribe(const char *topic) {
  esp_mqtt_client_subscribe(client_, topic, 1);
  return ESP_OK;
}

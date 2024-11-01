#ifndef MQTT_H
#define MQTT_H

extern "C" {
#include <mqtt_client.h>
}

typedef void (*MessageHandler)(MQTTClient *client, const char *topic,
                               int topic_len, const char *data, int data_len);

using ConnectCallback = std::function<void()>;
using DisconnectCallback = std::function<void()>;

struct TopicHandler {
  char topic[64];
  MessageHandler handler;
  bool active;

  void clear() {
    topic[0] = '\0';
    handler = nullptr;
    active = false;
  }
};

class MQTTClient {
private:
  esp_mqtt_client_handle_t client_;

  staic void event_handler(void *handler_args, esp_event_base_t base,
                           int32_t event_id, void *event_data);

  ConnectCallback connect_handler_;
  DisconnectCallback disconnect_handler_;

public:
  // Define callback types

  MQTTClient(const char *uri, const char *username, const char *password);
  ~MQTTClient() {
    if (client_) {
      esp_mqtt_client_stop(client_);
      esp_mqtt_client_destroy(client_);
    }
  };
  void register_message_handler();
  void register_connect_handler();
  esp_err_t publish(const char *topic, const char *payload, int qos);
  esp_err_t subscribe(const char *topic);
};

#endif

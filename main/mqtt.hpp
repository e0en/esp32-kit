#ifndef MQTT_H
#define MQTT_H

extern "C" {
#include <mqtt_client.h>
}

#define MAX_TOPIC_COUNT 10

class MQTTClient;

typedef void (*ConnectCallback)();
typedef void (*MessageHandler)(MQTTClient *client, const char *data,
                               int data_len);

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
  bool is_connected_;
  int64_t timeout_us_ = 1000000;
  TopicHandler handlers_[MAX_TOPIC_COUNT];
  static void event_handler(void *handler_args, esp_event_base_t base,
                            int32_t event_id, void *event_data);

public:
  MQTTClient(const char *uri, const char *client_id, const char *username,
             const char *password);
  ~MQTTClient() {
    if (client_) {
      esp_mqtt_client_stop(client_);
      esp_mqtt_client_destroy(client_);
    }
  };
  esp_err_t connect();
  esp_err_t publish(const char *topic, const char *payload, size_t length,
                    int qos);
  void subscribe(const char *topic, MessageHandler callback);
};

#endif

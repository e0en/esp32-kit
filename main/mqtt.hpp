#ifndef MQTT_H
#define MQTT_H

extern "C" {
#include <mqtt_client.h>
}

class MQTTClient {
  MQTTClient(const char *uri, const char *username, const char *password);
  esp_err_t publish(const char *topic, const char *payload, int qos);

private:
  esp_mqtt_client_handle_t client;
};

#endif

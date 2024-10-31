#ifndef ESP32_WIFI_H
#define ESP32_WIFI_H

extern "C" {
#include <esp_event.h>
}

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

void init_wifi(const char *ssid, const char *password);

#endif

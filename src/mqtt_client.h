#pragma once

#include <WiFiClient.h>

void mqtt_setup(WiFiClient& wifi_client);
void mqtt_task(void* pvParameters);

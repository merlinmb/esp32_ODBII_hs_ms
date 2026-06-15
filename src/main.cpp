#include <Arduino.h>
#include <WiFi.h>
#include "credentials.h"
#include "obd_data.h"
#include "can_hs.h"
#include "can_ms.h"
#include "web_server.h"
#include "mqtt_client.h"
#include "sleep_manager.h"
#include "display.h"

// Globals defined here, declared extern in obd_data.h
VehicleData g_vehicle = {
    .rpm               = NAN,
    .speed_kmh         = NAN,
    .coolant_temp_c    = NAN,
    .intake_air_temp_c = NAN,
    .maf_g_per_s       = NAN,
    .throttle_pct      = NAN,
    .battery_voltage   = NAN,
    .oil_temp_c        = NAN,
    .fuel_level_pct    = NAN,
};
SemaphoreHandle_t g_data_mutex = nullptr;

static WiFiClient s_wifi_client;

static void wifi_connect() {
    Serial.printf("[WiFi] connecting to %s ...\n", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    uint8_t attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print('.');
        attempts++;
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[WiFi] connected, IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("[WiFi] failed to connect — continuing without network");
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[Boot] Ford Focus OBD-II logger starting");

    g_data_mutex = xSemaphoreCreateMutex();
    diag_log_init();

    sleep_manager_init();

    wifi_connect();
    web_server_begin();
    mqtt_setup(s_wifi_client);
    display_init();

    // Both CAN tasks on Core 1 — Core 0 idle task is WDT-monitored and must not be starved.
    // MCP2515 SPI transfers block with no timeout, so can_hs must not run on Core 0.
    xTaskCreatePinnedToCore(
        can_hs_task, "can_hs", 6144, nullptr, 5, nullptr, 1
    );
    xTaskCreatePinnedToCore(
        can_ms_task, "can_ms", 6144, nullptr, 5, nullptr, 1
    );

    Serial.println("[Boot] tasks started — open http://" + WiFi.localIP().toString());
}

void loop() {
    mqtt_loop();
    display_loop();
    sleep_manager_check();

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WiFi] reconnecting...");
        WiFi.reconnect();
        delay(500);
        return;
    }

    delay(10);
}

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include "credentials.h"
#include "obd_data.h"
#include "can_hs.h"
#include "can_ms.h"
#include "web_server.h"
#include "mqtt_client.h"
#include "sleep_manager.h"
#include "display.h"
#include "esp_sleep.h"

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

static WiFiClient s_mqtt_wifi_client;

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

    esp_sleep_wakeup_cause_t wakeup = esp_sleep_get_wakeup_cause();
    if (wakeup == ESP_SLEEP_WAKEUP_EXT1) {
        Serial.println("\n[Boot] wakeup from deep sleep (CAN activity on GPIO8)");
    } else if (wakeup != ESP_SLEEP_WAKEUP_UNDEFINED) {
        Serial.printf("\n[Boot] wakeup cause: %d\n", (int)wakeup);
    }

    Serial.println("\n[Boot] Ford Focus OBD-II logger starting");

    g_data_mutex = xSemaphoreCreateMutex();
    diag_log_init();

    sleep_manager_init();

    SPI.begin(SPI_SCLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, MCP2515_CS_PIN);

    wifi_connect();
    web_server_begin();
    mqtt_setup(s_mqtt_wifi_client);
    display_init();

    // HS-CAN stays on Core 1 because MCP2515 SPI transfers can block.
    xTaskCreatePinnedToCore(
        can_hs_task, "can_hs", 6144, nullptr, 4, nullptr, 1
    );
    // MS-CAN uses timed TWAI receives, so it can live on Core 0 and free Core 1
    // for the Arduino loop and async web server work.
    xTaskCreatePinnedToCore(
        can_ms_task, "can_ms", 6144, nullptr, 4, nullptr, 0
    );
    // MQTT on Core 0 — TCP connect blocks; keep it off Core 1 and out of loop().
    xTaskCreatePinnedToCore(
        mqtt_task, "mqtt", 4096, nullptr, 2, nullptr, 0
    );

    Serial.println("[Boot] tasks started — open http://" + WiFi.localIP().toString());
}

void loop() {
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

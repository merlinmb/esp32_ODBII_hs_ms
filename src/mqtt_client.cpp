#include "mqtt_client.h"
#include "obd_data.h"
#include "credentials.h"
#include <PubSubClient.h>
#include <WiFi.h>
#include <Arduino.h>

static PubSubClient s_mqtt;
static unsigned long s_last_publish = 0;

void mqtt_setup(WiFiClient& wifi_client) {
    s_mqtt.setClient(wifi_client);
    s_mqtt.setServer(MQTT_BROKER, MQTT_PORT);
    s_mqtt.setBufferSize(640);
}

static unsigned long s_last_reconnect_attempt = 0;
#define MQTT_RECONNECT_INTERVAL_MS 10000

static void reconnect() {
    if (s_mqtt.connected()) return;
    if (WiFi.status() != WL_CONNECTED) return;

    unsigned long now = millis();
    if (now - s_last_reconnect_attempt < MQTT_RECONNECT_INTERVAL_MS) return;
    s_last_reconnect_attempt = now;

    Serial.printf("[MQTT] connecting to %s:%d ...\n", MQTT_BROKER, MQTT_PORT);
    if (s_mqtt.connect(MQTT_CLIENT_ID)) {
        Serial.println("[MQTT] connected");
    } else {
        Serial.printf("[MQTT] failed, rc=%d — will retry in %d s\n",
                      s_mqtt.state(), MQTT_RECONNECT_INTERVAL_MS / 1000);
    }
}

static void publish_data() {
    VehicleData snap;
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        snap = g_vehicle;
        xSemaphoreGive(g_data_mutex);
    }

    char buf[512];
    snprintf(buf, sizeof(buf),
        "{"
        "\"rpm\":%.1f,"
        "\"speed_kmh\":%.1f,"
        "\"coolant_temp_c\":%.1f,"
        "\"intake_air_temp_c\":%.1f,"
        "\"maf_g_per_s\":%.2f,"
        "\"throttle_pct\":%.1f,"
        "\"battery_voltage\":%.2f,"
        "\"oil_temp_c\":%.1f,"
        "\"dtc_present\":%s,"
        "\"dtc_count\":%u,"
        "\"fuel_level_pct\":%.1f,"
        "\"hs_connected\":%s,"
        "\"ms_connected\":%s"
        "}",
        snap.rpm, snap.speed_kmh, snap.coolant_temp_c,
        snap.intake_air_temp_c, snap.maf_g_per_s,
        snap.throttle_pct, snap.battery_voltage, snap.oil_temp_c,
        snap.dtc_present ? "true" : "false", snap.dtc_count,
        snap.fuel_level_pct,
        snap.hs_connected ? "true" : "false",
        snap.ms_connected ? "true" : "false"
    );

    if (s_mqtt.publish(MQTT_TOPIC, buf)) {
        Serial.printf("[MQTT] published to %s\n", MQTT_TOPIC);
    } else {
        Serial.println("[MQTT] publish failed");
    }
}

void mqtt_task(void* /*pvParameters*/) {
    for (;;) {
        reconnect();
        if (s_mqtt.connected()) {
            s_mqtt.loop();
            unsigned long now = millis();
            if (now - s_last_publish >= MQTT_PUBLISH_INTERVAL_MS) {
                s_last_publish = now;
                publish_data();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

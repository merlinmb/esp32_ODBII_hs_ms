#include "can_ms.h"
#include "obd_data.h"
#include "credentials.h"
#include <Arduino.h>
#include "driver/twai.h"

// Ford MS-CAN broadcast IDs (125 kbps, no request needed)
#define MS_ID_FUEL_LEVEL    0x420

// Consecutive receive-timeout threshold before declaring disconnected
#define MS_SILENCE_THRESHOLD_MS 5000

static twai_handle_t s_ms_handle = nullptr;

static bool install_ms_can() {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_MS_TX_PIN,
        (gpio_num_t)CAN_MS_RX_PIN,
        TWAI_MODE_LISTEN_ONLY
    );
    g.controller_id = 1;
    g.rx_queue_len  = 32;
    g.tx_queue_len  = 0;

    twai_timing_config_t t = TWAI_TIMING_CONFIG_125KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install_v2(&g, &t, &f, &s_ms_handle);
    if (err != ESP_OK) {
        diag_log("[MS-CAN] driver install failed: %s", esp_err_to_name(err));
        return false;
    }
    err = twai_start_v2(s_ms_handle);
    if (err != ESP_OK) {
        diag_log("[MS-CAN] start failed: %s", esp_err_to_name(err));
        twai_driver_uninstall_v2(s_ms_handle);
        s_ms_handle = nullptr;
        return false;
    }
    diag_log("[MS-CAN] started OK (125 kbps, listen-only)");
    return true;
}

static void uninstall_ms_can() {
    if (s_ms_handle) {
        twai_stop_v2(s_ms_handle);
        twai_driver_uninstall_v2(s_ms_handle);
        s_ms_handle = nullptr;
    }
}

void can_ms_task(void* /*pvParameters*/) {
    diag_log("[MS-CAN] task started, connecting...");

    while (!install_ms_can()) {
        diag_log("[MS-CAN] retrying in 3 s...");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }

    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_vehicle.ms_connected = true;
        g_vehicle.reconnect_ms = false;
        xSemaphoreGive(g_data_mutex);
    }

    twai_message_t msg;
    unsigned long last_rx_ms = millis();
    bool silence_logged = false;

    for (;;) {
        // Handle reconnect request from web UI
        bool want_reconnect = false;
        if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            want_reconnect = g_vehicle.reconnect_ms;
            xSemaphoreGive(g_data_mutex);
        }
        if (want_reconnect) {
            diag_log("[MS-CAN] reconnect requested — restarting driver...");
            uninstall_ms_can();
            if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_vehicle.ms_connected = false;
                g_vehicle.reconnect_ms = false;
                xSemaphoreGive(g_data_mutex);
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            while (!install_ms_can()) {
                diag_log("[MS-CAN] reconnect failed, retrying in 3 s...");
                vTaskDelay(pdMS_TO_TICKS(3000));
            }
            if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_vehicle.ms_connected = true;
                xSemaphoreGive(g_data_mutex);
            }
            last_rx_ms = millis();
            silence_logged = false;
            diag_log("[MS-CAN] reconnect succeeded");
        }

        if (twai_receive_v2(s_ms_handle, &msg, pdMS_TO_TICKS(100)) != ESP_OK) {
            // Check for prolonged silence
            if (!silence_logged && (millis() - last_rx_ms) > MS_SILENCE_THRESHOLD_MS) {
                diag_log("[MS-CAN] no frames received for %lu ms — bus silent?",
                         millis() - last_rx_ms);
                if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    g_vehicle.ms_connected = false;
                    xSemaphoreGive(g_data_mutex);
                }
                silence_logged = true;
            }
            continue;
        }

        // Frame received
        if (silence_logged) {
            diag_log("[MS-CAN] frames resumed (ID 0x%03lX)", (unsigned long)msg.identifier);
            if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_vehicle.ms_connected = true;
                xSemaphoreGive(g_data_mutex);
            }
            silence_logged = false;
        }
        last_rx_ms = millis();

        switch (msg.identifier) {
            case MS_ID_FUEL_LEVEL:
                if (msg.data_length_code >= 1) {
                    float pct = msg.data[0] / 2.55f;
                    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        g_vehicle.fuel_level_pct    = pct;
                        g_vehicle.last_ms_update_ms = millis();
                        xSemaphoreGive(g_data_mutex);
                    }
                }
                break;

            default:
                break;
        }
    }
}

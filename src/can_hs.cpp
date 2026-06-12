#include "can_hs.h"
#include "obd_data.h"
#include "credentials.h"
#include <Arduino.h>
#include "driver/twai.h"

// OBD-II Mode 01 PID definitions
#define PID_ENGINE_LOAD     0x04
#define PID_COOLANT_TEMP    0x05
#define PID_RPM             0x0C
#define PID_SPEED           0x0D
#define PID_INTAKE_AIR_TEMP 0x0F
#define PID_MAF             0x10
#define PID_THROTTLE        0x11
#define PID_BATTERY_VOLTAGE 0x42
#define PID_OIL_TEMP        0x5C
#define PID_DTC_STATUS      0x01

#define OBD_REQUEST_ID          0x7DF
#define OBD_RESPONSE_ID         0x7E8
#define OBD_RESPONSE_TIMEOUT_MS 25

// Consecutive timeout threshold before declaring disconnected
#define HS_FAIL_THRESHOLD 8

static twai_handle_t s_hs_handle = nullptr;

static bool install_hs_can() {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_HS_TX_PIN,
        (gpio_num_t)CAN_HS_RX_PIN,
        TWAI_MODE_NORMAL
    );
    g.controller_id = 0;
    g.rx_queue_len  = 16;
    g.tx_queue_len  = 8;

    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install_v2(&g, &t, &f, &s_hs_handle);
    if (err != ESP_OK) {
        diag_log("[HS-CAN] driver install failed: %s", esp_err_to_name(err));
        return false;
    }
    err = twai_start_v2(s_hs_handle);
    if (err != ESP_OK) {
        diag_log("[HS-CAN] start failed: %s", esp_err_to_name(err));
        twai_driver_uninstall_v2(s_hs_handle);
        s_hs_handle = nullptr;
        return false;
    }
    diag_log("[HS-CAN] started OK (500 kbps)");
    return true;
}

static void uninstall_hs_can() {
    if (s_hs_handle) {
        twai_stop_v2(s_hs_handle);
        twai_driver_uninstall_v2(s_hs_handle);
        s_hs_handle = nullptr;
    }
}

static bool send_pid_request(uint8_t pid) {
    twai_message_t msg = {};
    msg.identifier       = OBD_REQUEST_ID;
    msg.extd             = 0;
    msg.data_length_code = 8;
    msg.data[0] = 0x02;
    msg.data[1] = 0x01;
    msg.data[2] = pid;
    return twai_transmit_v2(s_hs_handle, &msg, pdMS_TO_TICKS(10)) == ESP_OK;
}

static bool recv_pid_response(uint8_t pid, uint8_t* out_a, uint8_t* out_b) {
    twai_message_t msg;
    unsigned long deadline = millis() + OBD_RESPONSE_TIMEOUT_MS;
    while (millis() < deadline) {
        if (twai_receive_v2(s_hs_handle, &msg, pdMS_TO_TICKS(5)) == ESP_OK) {
            if (msg.identifier == OBD_RESPONSE_ID &&
                msg.data_length_code >= 4 &&
                msg.data[1] == 0x41 &&
                msg.data[2] == pid) {
                *out_a = msg.data[3];
                *out_b = (msg.data_length_code >= 5) ? msg.data[4] : 0;
                return true;
            }
        }
    }
    return false;
}

static bool query_pid(uint8_t pid, uint8_t* a, uint8_t* b) {
    if (!send_pid_request(pid)) return false;
    return recv_pid_response(pid, a, b);
}

static void poll_dtc_status() {
    uint8_t a, b;
    if (!query_pid(PID_DTC_STATUS, &a, &b)) return;
    bool mil    = (a & 0x80) != 0;
    uint8_t cnt = (a & 0x7F);
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_vehicle.dtc_present = mil;
        g_vehicle.dtc_count   = cnt;
        xSemaphoreGive(g_data_mutex);
    }
    if (mil) diag_log("[HS-CAN] MIL on — %u DTC(s) present", cnt);
}

void can_hs_task(void* /*pvParameters*/) {
    diag_log("[HS-CAN] task started, connecting...");

    while (!install_hs_can()) {
        diag_log("[HS-CAN] retrying in 3 s...");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }

    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_vehicle.hs_connected = true;
        g_vehicle.reconnect_hs = false;
        xSemaphoreGive(g_data_mutex);
    }

    uint8_t a, b;
    uint32_t dtc_cycle   = 0;
    uint8_t  fail_streak = 0;

    for (;;) {
        // Handle reconnect request from web UI
        bool want_reconnect = false;
        if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            want_reconnect = g_vehicle.reconnect_hs;
            xSemaphoreGive(g_data_mutex);
        }
        if (want_reconnect) {
            diag_log("[HS-CAN] reconnect requested — restarting driver...");
            uninstall_hs_can();
            if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_vehicle.hs_connected = false;
                g_vehicle.reconnect_hs = false;
                xSemaphoreGive(g_data_mutex);
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            while (!install_hs_can()) {
                diag_log("[HS-CAN] reconnect failed, retrying in 3 s...");
                vTaskDelay(pdMS_TO_TICKS(3000));
            }
            if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_vehicle.hs_connected = true;
                xSemaphoreGive(g_data_mutex);
            }
            fail_streak = 0;
            diag_log("[HS-CAN] reconnect succeeded");
        }

        // --- RPM ---
        if (query_pid(PID_RPM, &a, &b)) {
            float v = ((uint16_t)a * 256 + b) / 4.0f;
            if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_vehicle.rpm = v;
                g_vehicle.last_hs_update_ms = millis();
                xSemaphoreGive(g_data_mutex);
            }
            if (fail_streak >= HS_FAIL_THRESHOLD) {
                diag_log("[HS-CAN] connection recovered (RPM responding)");
                if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    g_vehicle.hs_connected = true;
                    xSemaphoreGive(g_data_mutex);
                }
            }
            fail_streak = 0;
        } else {
            fail_streak++;
            if (fail_streak == HS_FAIL_THRESHOLD) {
                diag_log("[HS-CAN] no response from ECU — %u consecutive timeouts", fail_streak);
                if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    g_vehicle.hs_connected = false;
                    xSemaphoreGive(g_data_mutex);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));

        // --- Speed ---
        if (query_pid(PID_SPEED, &a, &b)) {
            if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_vehicle.speed_kmh = a;
                xSemaphoreGive(g_data_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));

        // --- Coolant temp ---
        if (query_pid(PID_COOLANT_TEMP, &a, &b)) {
            if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_vehicle.coolant_temp_c = (float)a - 40.0f;
                xSemaphoreGive(g_data_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));

        // --- Intake air temp ---
        if (query_pid(PID_INTAKE_AIR_TEMP, &a, &b)) {
            if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_vehicle.intake_air_temp_c = (float)a - 40.0f;
                xSemaphoreGive(g_data_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));

        // --- MAF ---
        if (query_pid(PID_MAF, &a, &b)) {
            if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_vehicle.maf_g_per_s = ((uint16_t)a * 256 + b) / 100.0f;
                xSemaphoreGive(g_data_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));

        // --- Throttle ---
        if (query_pid(PID_THROTTLE, &a, &b)) {
            if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_vehicle.throttle_pct = a * 100.0f / 255.0f;
                xSemaphoreGive(g_data_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));

        // --- Battery voltage ---
        if (query_pid(PID_BATTERY_VOLTAGE, &a, &b)) {
            if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_vehicle.battery_voltage = ((uint16_t)a * 256 + b) / 1000.0f;
                xSemaphoreGive(g_data_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));

        // --- Oil temp ---
        if (query_pid(PID_OIL_TEMP, &a, &b)) {
            if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_vehicle.oil_temp_c = (float)a - 40.0f;
                xSemaphoreGive(g_data_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));

        // --- DTC check every 20 cycles (~20 s) ---
        if (++dtc_cycle >= 20) {
            dtc_cycle = 0;
            poll_dtc_status();
        }
    }
}

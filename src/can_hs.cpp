#include "can_hs.h"
#include "obd_data.h"
#include "credentials.h"
#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <esp_task_wdt.h>

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

#define HS_FAIL_THRESHOLD 8

static MCP2515 s_mcp(MCP2515_CS_PIN);

static bool install_hs_can() {
    SPI.begin(SPI_SCLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, MCP2515_CS_PIN);

    // reset() can hang if MCP2515 is absent — guard with a brief yield first
    // Hardware reset the MCP2515
    pinMode(MCP2515_RST_PIN, OUTPUT);
    digitalWrite(MCP2515_RST_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(10));
    digitalWrite(MCP2515_RST_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));

    vTaskDelay(pdMS_TO_TICKS(10));
    s_mcp.reset();
    vTaskDelay(pdMS_TO_TICKS(10));

    if (s_mcp.setBitrate(CAN_500KBPS) != MCP2515::ERROR_OK) {
        diag_log("[HS-CAN] setBitrate failed");
        SPI.end();
        return false;
    }
    if (s_mcp.setNormalMode() != MCP2515::ERROR_OK) {
        diag_log("[HS-CAN] setNormalMode failed");
        SPI.end();
        return false;
    }
    diag_log("[HS-CAN] MCP2515 started OK (500 kbps)");
    return true;
}

static bool send_pid_request(uint8_t pid) {
    struct can_frame req = {};
    req.can_id  = OBD_REQUEST_ID;
    req.can_dlc = 8;
    req.data[0] = 0x02;
    req.data[1] = 0x01;
    req.data[2] = pid;
    return s_mcp.sendMessage(&req) == MCP2515::ERROR_OK;
}

static bool recv_pid_response(uint8_t pid, uint8_t* out_a, uint8_t* out_b) {
    struct can_frame resp;
    unsigned long deadline = millis() + OBD_RESPONSE_TIMEOUT_MS;
    while (millis() < deadline) {
        if (s_mcp.readMessage(&resp) == MCP2515::ERROR_OK) {
            if (resp.can_id == OBD_RESPONSE_ID &&
                resp.can_dlc >= 4 &&
                resp.data[1] == 0x41 &&
                resp.data[2] == pid) {
                *out_a = resp.data[3];
                *out_b = (resp.can_dlc >= 5) ? resp.data[4] : 0;
                return true;
            }
        }
        vTaskDelay(1);
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
    diag_log("[HS-CAN] task started, connecting via MCP2515...");

    // Do NOT subscribe to task WDT — MCP2515 SPI transfers block with no timeout
    // and will always trigger it when the bus is absent or slow to respond.
    esp_task_wdt_delete(NULL);

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
        bool want_reconnect = false;
        if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            want_reconnect = g_vehicle.reconnect_hs;
            xSemaphoreGive(g_data_mutex);
        }
        if (want_reconnect) {
            diag_log("[HS-CAN] reconnect requested — resetting MCP2515...");
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

#pragma once
#include <Arduino.h>
#include <freertos/semphr.h>

struct VehicleData {
    // HS-CAN — OBD-II standard PIDs
    float rpm;
    float speed_kmh;
    float coolant_temp_c;
    float intake_air_temp_c;
    float maf_g_per_s;
    float throttle_pct;
    float battery_voltage;
    float oil_temp_c;
    bool  dtc_present;
    uint8_t dtc_count;

    // MS-CAN — Ford broadcast
    float fuel_level_pct;

    // Bus health
    unsigned long last_hs_update_ms;
    unsigned long last_ms_update_ms;
    bool hs_connected;
    bool ms_connected;

    // Reconnect request flags (set by web handler, cleared by CAN tasks)
    volatile bool reconnect_hs;
    volatile bool reconnect_ms;
};

extern VehicleData g_vehicle;
extern SemaphoreHandle_t g_data_mutex;

// Diagnostic log — ring buffer of timestamped messages
#define DIAG_LOG_SIZE 32
#define DIAG_MSG_LEN  80

struct DiagEntry {
    unsigned long ts_ms;
    char msg[DIAG_MSG_LEN];
};

extern DiagEntry g_diag_log[DIAG_LOG_SIZE];
extern volatile uint16_t g_diag_head;  // monotonic write counter; index = head % DIAG_LOG_SIZE
extern SemaphoreHandle_t g_diag_mutex;

void diag_log(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void diag_log_init();

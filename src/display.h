#pragma once

#include <stdint.h>

struct EvTelemetry {
    uint8_t speed_kmh;
    uint8_t battery_soc_pct;
    int16_t power_tenths_kw;
    bool speed_valid;
    bool soc_valid;
    bool power_valid;
};

void display_init();
void display_update(const EvTelemetry &telemetry, bool ble_connected);
void display_shutdown();

#include "display.h"
#include "obd_data.h"
#include "credentials.h"
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <WiFi.h>
#include <math.h>

static U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(
    U8G2_R0, U8X8_PIN_NONE, DISPLAY_SCL, DISPLAY_SDA);

static bool s_display_ok = false;
static uint8_t s_frame = 0;
static uint32_t s_last_switch = 0;

// ---- value formatting -------------------------------------------------------

static void fmt_int(char *buf, size_t n, float v, const char *unit) {
    if (v < 0) { snprintf(buf, n, "--"); return; }
    snprintf(buf, n, "%d%s", (int)roundf(v), unit);
}

static void fmt_f1(char *buf, size_t n, float v, const char *unit) {
    if (v < 0) { snprintf(buf, n, "--"); return; }
    snprintf(buf, n, "%.1f%s", v, unit);
}

// ---- shared draw helpers ---------------------------------------------------

static void draw_title(const char *title) {
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(0, 8, title);
    u8g2.drawHLine(0, 10, 128);
}

static void draw_cell(uint8_t x, uint8_t top, const char *label, const char *value) {
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(x, top + 8, label);
    u8g2.setFont(u8g2_font_ncenB14_tf);
    u8g2.drawStr(x + 10, top + 23, value);
}

// ---- frame 0: ENGINE -------------------------------------------------------

static void draw_frame_engine(const VehicleData &d) {
    char b0[12], b1[12], b2[12], b3[12];
    fmt_int(b0, 12, d.rpm, "");
    fmt_int(b1, 12, d.speed_kmh, "");
    fmt_int(b2, 12, d.throttle_pct, "%");
    fmt_int(b3, 12, d.oil_temp_c, "\xb0\x43");
    draw_title("ENGINE");
    draw_cell(0,  11, "rpm",      b0);
    draw_cell(65, 11, "speed",    b1);
    draw_cell(0,  37, "throttle", b2);
    draw_cell(65, 37, "oil",      b3);
}

// ---- frame 1: SENSORS ------------------------------------------------------

static void draw_frame_sensors(const VehicleData &d) {
    char b0[12], b1[12], b2[12], b3[12];
    fmt_int(b0, 12, d.coolant_temp_c,    "\xb0\x43");
    fmt_int(b1, 12, d.intake_air_temp_c, "\xb0\x43");
    fmt_int(b2, 12, d.fuel_level_pct,    "%");
    fmt_f1 (b3, 12, d.battery_voltage,   "V");
    draw_title("SENSORS");
    draw_cell(0,  11, "coolant", b0);
    draw_cell(65, 11, "intake",  b1);
    draw_cell(0,  37, "fuel",    b2);
    draw_cell(65, 37, "battery", b3);
}

// ---- frame 2: STATUS -------------------------------------------------------

static void draw_status_row(uint8_t y, const char *label, bool ok, const char *detail) {
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(0, y, label);
    if (ok) u8g2.drawDisc(28, y - 3, 3);
    else    u8g2.drawCircle(28, y - 3, 3);
    u8g2.drawStr(35, y, detail);
}

static void draw_frame_status(const VehicleData &d) {
    bool wifi_ok = (WiFi.status() == WL_CONNECTED);
    draw_title("STATUS");
    draw_status_row(21, "HS-CAN", d.hs_connected, d.hs_connected ? "OK"  : "offline");
    draw_status_row(34, "MS-CAN", d.ms_connected, d.ms_connected ? "OK"  : "offline");
    draw_status_row(47, "WiFi",   wifi_ok,         wifi_ok        ? WiFi.SSID().c_str() : "---");
    u8g2.setFont(u8g2_font_5x7_tf);
    String ip = wifi_ok ? WiFi.localIP().toString() : String("---");
    u8g2.drawStr(0, 61, ip.c_str());
}

// ---- redraw ----------------------------------------------------------------

static void redraw(const VehicleData &snap) {
    u8g2.clearBuffer();
    switch (s_frame) {
        case 0: draw_frame_engine(snap);  break;
        case 1: draw_frame_sensors(snap); break;
        case 2: draw_frame_status(snap);  break;
    }
    u8g2.sendBuffer();
}

// ---- public API ------------------------------------------------------------

void display_init() {
    s_display_ok = u8g2.begin();
    if (s_display_ok) {
        u8g2.clearDisplay();
        s_last_switch = millis() - DISPLAY_INTERVAL_MS;
        diag_log("[DISP] SH1106 found on SDA=%d SCL=%d", DISPLAY_SDA, DISPLAY_SCL);
    } else {
        diag_log("[DISP] no display detected — running headless");
    }
}

void display_loop() {
    if (!s_display_ok) return;

    uint32_t now = millis();
    if ((now - s_last_switch) < (uint32_t)DISPLAY_INTERVAL_MS) return;
    s_last_switch = now;

    VehicleData snap;
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        snap = g_vehicle;
        xSemaphoreGive(g_data_mutex);
    } else {
        return;
    }

    redraw(snap);
    s_frame = (s_frame + 1) % 3;
}

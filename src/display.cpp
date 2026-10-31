#include "display.h"
#include "credentials.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Arduino.h>
#include <Wire.h>

static Adafruit_SH1106G display(
    OLED_SCREEN_WIDTH, OLED_SCREEN_HEIGHT, &Wire, OLED_RESET_PIN,
    100000, 100000);  // 100 kHz, as the previous working U8g2 setup used
static bool display_ready = false;

void display_init() {
    Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
    // begin() does not check for an ACK, so probe first to stay truly headless.
    Wire.beginTransmission(0x3C);
    display_ready = Wire.endTransmission() == 0 && display.begin(0x3C, true);
    if (!display_ready) {
        Serial.println("[OLED] initialization failed; continuing headless");
        return;
    }
    display.clearDisplay();
    display.display();
    Serial.printf("[OLED] ready on SDA=%d SCL=%d\n", OLED_SDA_PIN, OLED_SCL_PIN);
}

void display_update(const EvTelemetry &telemetry, bool ble_connected) {
    if (!display_ready) return;

    display.clearDisplay();
    display.setTextColor(SH110X_WHITE);

    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("SYS: READY");
    display.setCursor(75, 0);
    display.print(ble_connected ? "BLE: CON" : "BLE: ADV");
    display.drawFastHLine(0, 10, OLED_SCREEN_WIDTH, SH110X_WHITE);

    display.setTextSize(3);
    display.setCursor(20, 18);
    if (telemetry.speed_valid) {
        display.printf("%03u", static_cast<unsigned>(telemetry.speed_kmh));
    } else {
        display.print("---");
    }
    display.setTextSize(1);
    display.setCursor(80, 32);
    display.print("km/h");

    display.drawFastHLine(0, 48, OLED_SCREEN_WIDTH, SH110X_WHITE);
    display.setCursor(0, 54);
    if (telemetry.soc_valid) {
        display.printf("SOC: %u%%",
                       static_cast<unsigned>(telemetry.battery_soc_pct));
    } else {
        display.print("SOC: --");
    }

    display.setCursor(70, 54);
    if (!telemetry.power_valid) {
        display.print("PWR: --");
    } else if (telemetry.power_tenths_kw < 0) {
        display.printf("REGEN: %.1fkW",
                       -telemetry.power_tenths_kw / 10.0f);
    } else {
        display.printf("PWR: %.1fkW",
                       telemetry.power_tenths_kw / 10.0f);
    }

    display.display();
}

void display_shutdown() {
    if (!display_ready) return;
    display.clearDisplay();
    display.display();
    display.oled_command(SH110X_DISPLAYOFF);
}

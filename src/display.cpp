#include "display.h"
#include "credentials.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Wire.h>

static Adafruit_SSD1306 display(
    OLED_SCREEN_WIDTH, OLED_SCREEN_HEIGHT, &Wire, OLED_RESET_PIN);
static bool display_ready = false;

void display_init() {
    Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
    display_ready = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
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
    display.setTextColor(SSD1306_WHITE);

    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("SYS: READY");
    display.setCursor(75, 0);
    display.print(ble_connected ? "BLE: CON" : "BLE: ADV");
    display.drawFastHLine(0, 10, OLED_SCREEN_WIDTH, SSD1306_WHITE);

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

    display.drawFastHLine(0, 48, OLED_SCREEN_WIDTH, SSD1306_WHITE);
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
    display.ssd1306_command(SSD1306_DISPLAYOFF);
}

#include "sleep_manager.h"
#include "obd_data.h"
#include "display.h"
#include "credentials.h"
#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include "esp_sleep.h"

// MCP2515 SPI pin assignments (T-2CAN schematic)
#define MCP_CS_PIN    10
#define MCP_RST_PIN    9
#define MCP_MOSI_PIN  11
#define MCP_SCLK_PIN  12
#define MCP_MISO_PIN  13

// MCP2515 register addresses
#define MCP_CANINTE   0x2B
#define MCP_CANINTF   0x2C
#define MCP_WRITE     0x02
#define MCP_RESET     0xC0

static unsigned long s_last_activity_ms = 0;

static void mcp_write_register(uint8_t reg, uint8_t val) {
    digitalWrite(MCP_CS_PIN, LOW);
    SPI.transfer(MCP_WRITE);
    SPI.transfer(reg);
    SPI.transfer(val);
    digitalWrite(MCP_CS_PIN, HIGH);
}

static void mcp_reset() {
    digitalWrite(MCP_CS_PIN, LOW);
    SPI.transfer(MCP_RESET);
    digitalWrite(MCP_CS_PIN, HIGH);
    delay(10);
}

static void mcp2515_enable_wake() {
    SPI.begin(MCP_SCLK_PIN, MCP_MISO_PIN, MCP_MOSI_PIN, MCP_CS_PIN);
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

    pinMode(MCP_CS_PIN, OUTPUT);
    digitalWrite(MCP_CS_PIN, HIGH);

    mcp_reset();

    // Enable RX buffer 0 and 1 interrupts so INT goes LOW on any received frame
    mcp_write_register(MCP_CANINTE, 0x03); // RX0IE | RX1IE

    // Clear any pending interrupt flags
    mcp_write_register(MCP_CANINTF, 0x00);

    SPI.endTransaction();
}

void sleep_manager_init() {
    s_last_activity_ms = millis();
    esp_sleep_enable_ext0_wakeup((gpio_num_t)MCP2515_INT_PIN, 0);
    diag_log("[Sleep] init — idle timeout %lu ms, wake on GPIO%d LOW",
             (unsigned long)SLEEP_IDLE_TIMEOUT_MS, MCP2515_INT_PIN);
}

void sleep_manager_check() {
    unsigned long hs_last, ms_last;
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        hs_last = g_vehicle.last_hs_update_ms;
        ms_last  = g_vehicle.last_ms_update_ms;
        xSemaphoreGive(g_data_mutex);
    } else {
        return;
    }

    unsigned long most_recent = (hs_last > ms_last) ? hs_last : ms_last;
    if (most_recent > s_last_activity_ms) {
        s_last_activity_ms = most_recent;
    }

    if ((millis() - s_last_activity_ms) < (unsigned long)SLEEP_IDLE_TIMEOUT_MS) {
        return;
    }

    diag_log("[Sleep] %lu ms idle — entering deep sleep", (unsigned long)SLEEP_IDLE_TIMEOUT_MS);
    delay(100);

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);

    display_shutdown();

    mcp2515_enable_wake();

    Serial.println("[Sleep] deep sleep now — wake on CAN activity (GPIO8 LOW)");
    Serial.flush();

    esp_deep_sleep_start();
}

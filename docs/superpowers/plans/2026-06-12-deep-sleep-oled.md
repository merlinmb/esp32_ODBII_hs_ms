# Deep Sleep + OLED Display Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add MCP2515-interrupt-triggered deep sleep (5-minute CAN idle timeout) and an optional SH1106 OLED display showing live vehicle data, to the ESP32-S3 T-2CAN OBD-II logger.

**Architecture:** The idle watchdog runs in `loop()` on Core 1 — it reads the existing `last_hs_update_ms` / `last_ms_update_ms` timestamps from `VehicleData`, tears down WiFi+MQTT, then calls `esp_deep_sleep_start()`. Wake is via `esp_sleep_enable_ext0_wakeup(GPIO8, LOW)` — GPIO8 is the MCP2515 INT line on the T-2CAN board. The OLED (SH1106 128×64, U8g2, I2C) is initialised with a graceful-absent check: if `u8g2.begin()` returns false the display is silently disabled for the session.

**Tech Stack:** ESP-IDF deep sleep API (`esp_sleep.h`), Arduino SPI (for MCP2515 wake config), U8g2 library (SH1106 I2C driver), existing FreeRTOS/TWAI/PubSubClient stack.

---

## File Map

| File | Action | Purpose |
|------|--------|---------|
| `include/credentials.h.example` | Modify | Add `CAN_INT_PIN`, `SLEEP_IDLE_TIMEOUT_MS`, `DISPLAY_SDA`, `DISPLAY_SCL`, `DISPLAY_INTERVAL_MS` |
| `include/credentials.h` | Modify | Same additions (actual config file) |
| `src/sleep_manager.h` | Create | `sleep_manager_init()`, `sleep_manager_check()` declarations |
| `src/sleep_manager.cpp` | Create | Idle-timeout logic, MCP2515 wake config via SPI, deep sleep call |
| `src/display.h` | Create | `display_init()`, `display_loop()` declarations |
| `src/display.cpp` | Create | U8g2 SH1106 driver, 3 rotating frames, graceful-absent guard |
| `src/main.cpp` | Modify | Call `sleep_manager_init()` in `setup()`, call `sleep_manager_check()` and `display_loop()` in `loop()` |
| `platformio.ini` | Modify | Add U8g2 library dependency |

---

## Task 1: Add new constants to credentials files

**Files:**
- Modify: `include/credentials.h.example`
- Modify: `include/credentials.h`

- [ ] **Step 1: Add constants to `credentials.h.example`**

Open `include/credentials.h.example` and append after the last `#define`:

```cpp
// Deep sleep
#define CAN_INT_PIN              8       // MCP2515 INT pin — T-2CAN schematic GPIO8
#define SLEEP_IDLE_TIMEOUT_MS    300000  // 5 minutes; both CAN buses must be silent

// OLED display (SH1106 128x64, I2C) — set to 255 to disable
#define DISPLAY_SDA              1       // T-2CAN QWIIC SDA
#define DISPLAY_SCL              2       // T-2CAN QWIIC SCL
#define DISPLAY_INTERVAL_MS      2000    // ms between frame rotations
```

- [ ] **Step 2: Apply the same additions to `include/credentials.h`**

Add the identical block to the real credentials file.

- [ ] **Step 3: Commit**

```
git add include/credentials.h.example include/credentials.h
git commit -m "config: add CAN_INT_PIN, sleep timeout, and OLED pin constants"
```

---

## Task 2: Add U8g2 to platformio.ini

**Files:**
- Modify: `platformio.ini`

- [ ] **Step 1: Add U8g2 to lib_deps**

In `platformio.ini`, extend `lib_deps`:

```ini
lib_deps =
    https://github.com/me-no-dev/ESPAsyncWebServer.git
    https://github.com/me-no-dev/AsyncTCP.git
    knolleary/PubSubClient @ ^2.8
    olikraus/U8g2 @ ^2.35.19
```

- [ ] **Step 2: Verify build resolves the library**

```
pio lib install
```

Expected: U8g2 downloaded into `.pio/libdeps/lilygo-t2-can/U8g2/`.

- [ ] **Step 3: Commit**

```
git add platformio.ini
git commit -m "deps: add U8g2 library for SH1106 OLED"
```

---

## Task 3: Create `sleep_manager.h` and `sleep_manager.cpp`

**Files:**
- Create: `src/sleep_manager.h`
- Create: `src/sleep_manager.cpp`

**Context:** The MCP2515 on the T-2CAN board is on SPI with CS=GPIO10, RST=GPIO9, MOSI=GPIO11, SCLK=GPIO12, MISO=GPIO13. To ensure the MCP2515 asserts INT LOW when a CAN frame arrives while the ESP32-S3 is in deep sleep, we must configure its interrupt enable register (`CANINTE`) with `RXnBF` bits set. We do this via a short SPI transaction immediately before calling `esp_deep_sleep_start()`. The MCP2515 remains powered from the OBD2 connector (pin 16 = battery +12 V → board regulator) during sleep.

- [ ] **Step 1: Create `src/sleep_manager.h`**

```cpp
#pragma once

void sleep_manager_init();
void sleep_manager_check();
```

- [ ] **Step 2: Create `src/sleep_manager.cpp`**

```cpp
#include "sleep_manager.h"
#include "obd_data.h"
#include "credentials.h"
#include <Arduino.h>
#include <SPI.h>
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
#define MCP_BIT_MODIFY 0x05
#define MCP_WRITE      0x02
#define MCP_RESET      0xC0

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
    // Bring up SPI just long enough to configure the MCP2515 wake interrupt
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
    // Leave SPI running — deep sleep will power it down
}

void sleep_manager_init() {
    // Record boot time as initial activity so timeout runs from first start
    s_last_activity_ms = millis();
    // Configure wake source — GPIO8 LOW = MCP2515 INT asserted
    esp_sleep_enable_ext0_wakeup((gpio_num_t)CAN_INT_PIN, 0);
    diag_log("[Sleep] init — idle timeout %lu ms, wake on GPIO%d LOW",
             (unsigned long)SLEEP_IDLE_TIMEOUT_MS, CAN_INT_PIN);
}

void sleep_manager_check() {
    // Update activity timestamp if either CAN bus received data recently
    unsigned long hs_last, ms_last;
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        hs_last = g_vehicle.last_hs_update_ms;
        ms_last  = g_vehicle.last_ms_update_ms;
        xSemaphoreGive(g_data_mutex);
    } else {
        return; // Can't read — skip this check cycle
    }

    unsigned long most_recent = (hs_last > ms_last) ? hs_last : ms_last;
    if (most_recent > s_last_activity_ms) {
        s_last_activity_ms = most_recent;
    }

    if ((millis() - s_last_activity_ms) < SLEEP_IDLE_TIMEOUT_MS) {
        return; // Still active
    }

    // --- Both buses silent for SLEEP_IDLE_TIMEOUT_MS — go to sleep ---
    diag_log("[Sleep] %lu ms idle — entering deep sleep", (unsigned long)SLEEP_IDLE_TIMEOUT_MS);

    // Give the log a moment to flush to serial
    delay(100);

    // Tear down network cleanly
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);

    // Configure MCP2515 to assert INT on next received frame
    mcp2515_enable_wake();

    Serial.println("[Sleep] deep sleep now — wake on CAN activity (GPIO8 LOW)");
    Serial.flush();

    esp_deep_sleep_start();
    // Never returns
}
```

- [ ] **Step 3: Verify it compiles (no link yet)**

```
pio run -e lilygo-t2-can
```

Expected: compiles without errors. `sleep_manager_check` and `sleep_manager_init` are defined but not yet called — linker may warn about unused symbols, that's fine.

- [ ] **Step 4: Commit**

```
git add src/sleep_manager.h src/sleep_manager.cpp
git commit -m "feat: add sleep_manager with MCP2515 wake config and idle timeout"
```

---

## Task 4: Create `display.h` and `display.cpp`

**Files:**
- Create: `src/display.h`
- Create: `src/display.cpp`

**Context:** The U8g2 `begin()` call does an I2C probe. If the display is absent it returns `false` — we store that in `s_display_ok` and early-return on every subsequent draw call. No exception, no hang. The three frames rotate every `DISPLAY_INTERVAL_MS`. The VehicleData snapshot is taken under mutex at the start of each draw cycle.

- [ ] **Step 1: Create `src/display.h`**

```cpp
#pragma once

void display_init();
void display_loop();
```

- [ ] **Step 2: Create `src/display.cpp`**

```cpp
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
        s_last_switch = millis() - DISPLAY_INTERVAL_MS; // draw on first loop call
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

    // Snapshot VehicleData under mutex
    VehicleData snap;
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        snap = g_vehicle;
        xSemaphoreGive(g_data_mutex);
    } else {
        return; // Skip this frame rather than draw stale data
    }

    redraw(snap);
    s_frame = (s_frame + 1) % 3;
}
```

- [ ] **Step 3: Build to check for compile errors**

```
pio run -e lilygo-t2-can
```

Expected: compiles cleanly. Functions are defined but not yet called from `main.cpp`.

- [ ] **Step 4: Commit**

```
git add src/display.h src/display.cpp
git commit -m "feat: add OLED display module with graceful absent-screen handling"
```

---

## Task 5: Wire everything into `main.cpp`

**Files:**
- Modify: `src/main.cpp`

- [ ] **Step 1: Add includes and call `sleep_manager_init()` + `display_init()` in `setup()`**

The new `main.cpp` in full:

```cpp
#include <Arduino.h>
#include <WiFi.h>
#include "credentials.h"
#include "obd_data.h"
#include "can_hs.h"
#include "can_ms.h"
#include "web_server.h"
#include "mqtt_client.h"
#include "sleep_manager.h"
#include "display.h"

// Globals defined here, declared extern in obd_data.h
VehicleData g_vehicle = {};
SemaphoreHandle_t g_data_mutex = nullptr;

static WiFiClient s_wifi_client;

static void wifi_connect() {
    Serial.printf("[WiFi] connecting to %s ...\n", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    uint8_t attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print('.');
        attempts++;
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[WiFi] connected, IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("[WiFi] failed to connect — continuing without network");
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[Boot] Ford Focus OBD-II logger starting");

    g_data_mutex = xSemaphoreCreateMutex();
    diag_log_init();

    sleep_manager_init();

    wifi_connect();
    web_server_begin();
    mqtt_setup(s_wifi_client);
    display_init();

    // HS-CAN task pinned to Core 0
    xTaskCreatePinnedToCore(
        can_hs_task, "can_hs", 4096, nullptr, 5, nullptr, 0
    );

    // MS-CAN task pinned to Core 0
    xTaskCreatePinnedToCore(
        can_ms_task, "can_ms", 4096, nullptr, 5, nullptr, 0
    );

    Serial.println("[Boot] tasks started — open http://" + WiFi.localIP().toString());
}

void loop() {
    // WiFi watchdog — reconnect if dropped
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WiFi] reconnecting...");
        WiFi.reconnect();
        delay(5000);
        return;
    }

    mqtt_loop();
    display_loop();
    sleep_manager_check();
    delay(10);
}
```

- [ ] **Step 2: Full build**

```
pio run -e lilygo-t2-can
```

Expected: clean build, no errors.

- [ ] **Step 3: Flash and observe serial output**

```
pio run -e lilygo-t2-can --target upload && pio device monitor -p COM3 -b 115200
```

Expected serial lines at boot:
```
[Boot] Ford Focus OBD-II logger starting
[Sleep] init — idle timeout 300000 ms, wake on GPIO8 LOW
[WiFi] connecting to ...
[DISP] SH1106 found on SDA=1 SCL=2     ← if display wired
  OR
[DISP] no display detected — running headless   ← if no display
[HS-CAN] started OK (500 kbps)
[MS-CAN] started OK (125 kbps, listen-only)
```

- [ ] **Step 4: Verify sleep fires after idle**

Disconnect the OBD2 connector (so no CAN frames arrive). After 5 minutes, expected serial output:

```
[Sleep] 300000 ms idle — entering deep sleep
[Sleep] deep sleep now — wake on CAN activity (GPIO8 LOW)
```

Then the device goes silent on serial. Reconnect OBD2 — the MCP2515 INT line should go LOW, waking the ESP32-S3 and triggering a fresh `setup()` boot sequence on serial.

- [ ] **Step 5: Commit**

```
git add src/main.cpp
git commit -m "feat: wire sleep_manager and display into main setup/loop"
```

---

## Task 6: Update design spec

**Files:**
- Modify: `docs/superpowers/specs/2026-06-11-lilygo-t2can-obd2-design.md`

- [ ] **Step 1: Add deep sleep and OLED sections to the spec**

Append after the "Out of Scope" section:

```markdown
---

## Deep Sleep

- Idle timeout: `SLEEP_IDLE_TIMEOUT_MS` (default 300 000 ms / 5 min)
- Both `last_hs_update_ms` and `last_ms_update_ms` must be stale before sleep triggers
- Wake source: `esp_sleep_enable_ext0_wakeup(CAN_INT_PIN, LOW)` — GPIO8 = MCP2515 INT
- Pre-sleep: WiFi disconnected, MCP2515 CANINTE configured via SPI (RX0IE | RX1IE set, CANINTF cleared)
- On wake: full reboot, `setup()` runs normally

## OLED Display

- Hardware: SH1106 128×64, I2C, T-2CAN QWIIC connector (SDA=GPIO1, SCL=GPIO2)
- Library: U8g2
- Graceful absent: `display_init()` stores `u8g2.begin()` result; all draws are no-ops if display absent
- 3 rotating frames (interval `DISPLAY_INTERVAL_MS`, default 2 s):
  - Frame 0 ENGINE: RPM, Speed, Throttle, Oil Temp
  - Frame 1 SENSORS: Coolant °C, IAT °C, Fuel %, Battery V
  - Frame 2 STATUS: HS-CAN health, MS-CAN health, WiFi SSID, IP address
```

- [ ] **Step 2: Commit**

```
git add docs/superpowers/specs/2026-06-11-lilygo-t2can-obd2-design.md
git commit -m "docs: update spec with deep sleep and OLED design"
```

---

## Self-Review

**Spec coverage:**
- [x] Deep sleep on 5-min CAN idle — Task 3 (`sleep_manager_check`)
- [x] Wake on MCP2515 INT GPIO8 LOW — Task 3 (`sleep_manager_init` + `mcp2515_enable_wake`)
- [x] `CAN_INT_PIN` configurable in `credentials.h` — Task 1
- [x] OLED SH1106 U8g2 ported from ZUS bridge — Task 4
- [x] Graceful absent-screen — Task 4 (`s_display_ok` guard)
- [x] STATUS frame uses HS/MS CAN health (not Bluetooth) — Task 4
- [x] I2C pins reserved, not clashing with sleep GPIO8 — GPIO1/2 for I2C, GPIO8 for INT
- [x] U8g2 dependency — Task 2
- [x] `display_loop()` and `sleep_manager_check()` called from `loop()` — Task 5

**Placeholder scan:** None found — all steps contain full code.

**Type consistency:**
- `VehicleData` field names (`rpm`, `speed_kmh`, `coolant_temp_c`, `intake_air_temp_c`, `throttle_pct`, `oil_temp_c`, `fuel_level_pct`, `battery_voltage`, `hs_connected`, `ms_connected`, `last_hs_update_ms`, `last_ms_update_ms`) match `src/obd_data.h` exactly.
- `display_loop()` takes no arguments (snapshot taken internally under mutex) — matches `display.h` declaration.
- `sleep_manager_init()` / `sleep_manager_check()` — match `sleep_manager.h` declarations.

---
name: lilygo-t2can-obd2-ford-focus-2008
description: Dual CAN OBD-II logger for Ford Focus 2008 using Lilygo T2-CAN, with web dashboard and MQTT publishing
metadata:
  type: project
---

# Lilygo T2-CAN OBD-II Logger — Design Spec

**Date:** 2026-06-11  
**Target vehicle:** Ford Focus 2008  
**Hardware:** Lilygo T2-CAN (ESP32-S3, dual TWAI controllers)

---

## Goals

- Read both HS-CAN (500 kbps) and MS-CAN (125 kbps) buses simultaneously
- Decode comprehensive OBD-II PIDs: RPM, speed, coolant temp, IAT, MAF, throttle, battery voltage, oil temp, DTCs (HS); fuel level (MS)
- Host a lightweight web dashboard showing live values (auto-refresh every 5 s)
- Publish all values as JSON to a Mosquitto MQTT broker every 30 s (configurable)
- All credentials (WiFi, MQTT, pins, intervals) in `credentials.h` — no runtime config UI

---

## Architecture

```
credentials.h          — all config: WiFi, MQTT, GPIO pins, intervals
src/
  main.cpp             — setup(), loop(), FreeRTOS task launch
  obd_data.h           — shared VehicleData struct + SemaphoreHandle_t mutex
  can_hs.h/.cpp        — FreeRTOS task: CAN0/TWAI0, HS-CAN OBD-II request/response
  can_ms.h/.cpp        — FreeRTOS task: CAN1/TWAI1, MS-CAN broadcast listener
  web_server.h/.cpp    — ESPAsyncWebServer; HTML in PROGMEM; /data JSON endpoint
  mqtt_client.h/.cpp   — PubSubClient wrapper; publishes VehicleData JSON on interval
```

### Runtime task layout

| Task | Core | Stack | Responsibility |
|------|------|-------|----------------|
| `task_can_hs` | 0 | 4096 | Poll HS-CAN PIDs sequentially, decode, write to g_vehicle |
| `task_can_ms` | 0 | 4096 | Listen for MS-CAN broadcast frames, decode, write to g_vehicle |
| `loop()` | 1 | default | WiFi keepalive, MQTT reconnect + publish, yield to async web server |

The async web server runs on its own internal FreeRTOS task (managed by ESPAsyncWebServer).

---

## Data Model

```cpp
struct VehicleData {
    // HS-CAN — OBD-II standard PIDs
    float rpm;               // PID 0x0C
    float speed_kmh;         // PID 0x0D
    float coolant_temp_c;    // PID 0x05
    float intake_air_temp_c; // PID 0x0F
    float maf_g_per_s;       // PID 0x10
    float throttle_pct;      // PID 0x11
    float battery_voltage;   // PID 0x42
    float oil_temp_c;        // PID 0x5C
    bool  dtc_present;       // Mode 0x01 PID 0x01
    uint8_t dtc_count;

    // MS-CAN — Ford broadcast
    float fuel_level_pct;    // CAN ID 0x420, byte 0

    // Health
    unsigned long last_hs_update_ms;
    unsigned long last_ms_update_ms;
    bool hs_connected;
    bool ms_connected;
};
```

Access is always guarded by `g_data_mutex` (FreeRTOS mutex).

---

## CAN Bus Protocol

### HS-CAN (CAN0, 500 kbps) — OBD-II request/response

- Send request frame to `0x7DF` (functional addressing, all ECUs)
- Frame: `[0x02, 0x01, PID, 0x00, 0x00, 0x00, 0x00, 0x00]`
- ECU responds on `0x7E8` within 25 ms timeout
- Response frame: `[len, 0x41, PID, A, B, ...]`
- PIDs polled in round-robin, one per 100 ms cycle

### MS-CAN (CAN1, 125 kbps) — Ford broadcast listener

- No request needed; ECU broadcasts data continuously
- Listen for CAN ID `0x420` → fuel level: `byte[0] / 2.55` → %
- Additional Ford IDs can be added to `can_ms.cpp` as discovered

---

## Web Dashboard

- Served on port 80, route `/`
- HTML + CSS + JS stored as `const char[]` in PROGMEM (~8 KB)
- Dark theme, CSS grid of value cards with colour-coded status (green/amber/red)
- JS fetches `/data` every 5 s and updates DOM — no page reload
- `/data` endpoint returns JSON snapshot of `VehicleData`

---

## MQTT

- Library: PubSubClient
- Connects to broker defined in `credentials.h`
- Publishes full JSON payload to configured topic every `MQTT_PUBLISH_INTERVAL_MS`
- Auto-reconnects on drop; skips publish cycle if not connected (no buffering)
- Payload: flat JSON object, all VehicleData fields

---

## Configuration (`credentials.h`)

```cpp
#define WIFI_SSID              "your_ssid"
#define WIFI_PASSWORD          "your_password"
#define MQTT_BROKER            "192.168.1.100"
#define MQTT_PORT              1883
#define MQTT_TOPIC             "car/focus/obd"
#define MQTT_PUBLISH_INTERVAL_MS  30000
#define CAN_HS_TX_PIN          4
#define CAN_HS_RX_PIN          5
#define CAN_MS_TX_PIN          6
#define CAN_MS_RX_PIN          7
```

---

## Dependencies (PlatformIO)

- `ESP Async WebServer` — async HTTP server
- `AsyncTCP` — required by ESPAsyncWebServer on ESP32
- `PubSubClient` — MQTT client
- ESP-IDF TWAI driver (built-in, v2 multi-controller API)

---

## Out of Scope

- Runtime WiFi/MQTT configuration (credentials are compile-time)
- OTA firmware updates
- Local data logging / SD card
- CAN bus sniffing / raw frame capture UI

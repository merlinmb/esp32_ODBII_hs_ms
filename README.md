# Ford Focus 2008 — Dual CAN OBD-II Logger

ESP32-S3 firmware for the **Lilygo T2-CAN** board. Reads both the High-Speed (HS) and Medium-Speed (MS) CAN buses from a Ford Focus 2008 OBD-II connector, hosts a live web dashboard, and publishes data to a Mosquitto MQTT broker.

---

## What It Does

| Feature | Detail |
|---------|--------|
| HS-CAN (500 kbps) | RPM, speed, coolant temp, IAT, MAF, throttle %, battery voltage, oil temp, DTCs |
| MS-CAN (125 kbps) | Fuel level (Ford broadcast, CAN ID 0x420) |
| Web dashboard | Dark-theme card grid, auto-refreshes every 5 s, served on port 80 |
| MQTT | Publishes full JSON payload every 30 s (configurable) |
| Config | All credentials and pin assignments in `credentials.h` |

---

## Hardware

### Lilygo T2-CAN

The board exposes two independent CAN transceivers connected to the ESP32-S3's dual TWAI controllers. Verify your exact board revision's pinout against the schematic in the Lilygo GitHub repo before flashing.

**Default pin assignments (adjust in `credentials.h` if different):**

| Signal | GPIO |
|--------|------|
| CAN0 TX (HS-CAN) | 4 |
| CAN0 RX (HS-CAN) | 5 |
| CAN1 TX (MS-CAN) | 6 |
| CAN1 RX (MS-CAN) | 7 |

---

## OBD-II Wiring

Use a standard OBD-II breakout cable or connector. Wire as follows:

```
OBD-II Connector          Lilygo T2-CAN
─────────────────         ─────────────
Pin  6  HS-CAN H  ───────  CAN0 CANH
Pin 14  HS-CAN L  ───────  CAN0 CANL
Pin  3  MS-CAN H  ───────  CAN1 CANH  (Ford-specific)
Pin 11  MS-CAN L  ───────  CAN1 CANL  (Ford-specific)
Pin 16  12 V      ───────  VIN  (or external 5 V reg)
Pin  4  Chassis GND ─────  GND
Pin  5  Signal GND  ─────  GND
```

> **Note:** OBD-II pins 3 and 11 are Ford-proprietary MS-CAN. They are not present on non-Ford vehicles.

> **Termination:** The vehicle's CAN buses are already terminated at the ECU end. Do **not** add external 120 Ω resistors — the T2-CAN transceivers have on-board termination jumpers; leave them **open** (or cut the trace) when connecting to an in-vehicle bus.

---

## Ford Focus 2008 CAN Bus Notes

### HS-CAN (500 kbps)
Carries powertrain data. Accessed via standard OBD-II Mode 01 PIDs using functional address `0x7DF`. ECU responds on `0x7E8`.

### MS-CAN (125 kbps)
Carries body/comfort data (fuel gauge, HVAC, instrument cluster). Ford ECUs broadcast data continuously — no request frames needed. The firmware listens passively.

**Known MS-CAN broadcast IDs:**

| CAN ID | Content | Byte | Formula |
|--------|---------|------|---------|
| 0x420 | Fuel level | 0 | `byte / 2.55` → % |

Additional IDs can be added in `src/can_ms.cpp` as you discover them with a CAN sniffer.

---

## Software Setup

### Prerequisites

- [PlatformIO](https://platformio.org/) (VS Code extension or CLI)
- USB cable to COM3

### 1. Clone / open the project

Open the `esp32_ODBII_hs_ms` folder in VS Code with the PlatformIO extension installed.

### 2. Create credentials

```bash
cp credentials.h.example credentials.h
```

Edit `credentials.h` and fill in your values:

```cpp
#define WIFI_SSID              "MyNetwork"
#define WIFI_PASSWORD          "MyPassword"
#define MQTT_BROKER            "192.168.1.100"
#define MQTT_PORT              1883
#define MQTT_TOPIC             "car/focus/obd"
#define MQTT_PUBLISH_INTERVAL_MS  30000   // 30 s
#define CAN_HS_TX_PIN          4
#define CAN_HS_RX_PIN          5
#define CAN_MS_TX_PIN          6
#define CAN_MS_RX_PIN          7
```

### 3. Build and flash

```bash
pio run --target upload
```

Or use the PlatformIO toolbar in VS Code (→ Upload).

### 4. Monitor serial output

```bash
pio device monitor
```

The device prints WiFi connection status, CAN bus state, and each MQTT publish to the serial console at 115200 baud.

---

## Web Dashboard

Once connected to WiFi, open a browser to the device's IP address (printed on serial):

```
http://192.168.x.x/
```

The dashboard auto-refreshes every 5 seconds. Values turn amber when stale (> 10 s since last CAN update) and red when the bus is disconnected.

The raw JSON data is also available at:

```
http://192.168.x.x/data
```

---

## MQTT Payload

Published as a flat JSON object to the configured topic:

```json
{
  "rpm": 820.0,
  "speed_kmh": 0.0,
  "coolant_temp_c": 87.5,
  "intake_air_temp_c": 22.0,
  "maf_g_per_s": 3.1,
  "throttle_pct": 0.0,
  "battery_voltage": 14.1,
  "oil_temp_c": 91.0,
  "dtc_present": false,
  "dtc_count": 0,
  "fuel_level_pct": 62.4,
  "hs_connected": true,
  "ms_connected": true
}
```

---

## Project Structure

```
credentials.h          — your local config (gitignored)
credentials.h.example  — template committed to repo
platformio.ini         — build config, COM3, board
src/
  main.cpp             — setup, loop, FreeRTOS task launch
  obd_data.h           — shared VehicleData struct + mutex
  can_hs.h / .cpp      — HS-CAN FreeRTOS task (OBD-II request/response)
  can_ms.h / .cpp      — MS-CAN FreeRTOS task (Ford broadcast listener)
  web_server.h / .cpp  — async HTTP server, HTML in PROGMEM
  mqtt_client.h / .cpp — MQTT publish wrapper
docs/
  superpowers/specs/   — design documentation
```

---

## Troubleshooting

| Symptom | Likely cause |
|---------|-------------|
| CAN bus install fails | Wrong GPIO pins — check `credentials.h` against your board schematic |
| HS-CAN shows no data | Bus speed mismatch or missing termination — verify 500 kbps |
| MS-CAN fuel stays 0 | CAN ID 0x420 not present on your trim — sniff the bus to find the correct ID |
| WiFi not connecting | Check SSID/password in `credentials.h`; device must be on 2.4 GHz |
| MQTT not publishing | Check broker IP, port 1883 open, no auth required (or add auth to `mqtt_client.cpp`) |
| Web page not loading | Find IP in serial monitor; ensure device and browser on same subnet |

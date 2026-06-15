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

### Lilygo T2-CAN V1.0

The T2-CAN V1.0 uses a **hybrid CAN architecture** — not two native TWAI controllers as the board name might imply:

| Bus | Controller | Interface | GPIO |
|-----|-----------|-----------|------|
| CAN A — HS-CAN (500 kbps) | MCP2515 (external SPI) | SPI | CS=10, SCLK=12, MOSI=11, MISO=13 |
| CAN B — MS-CAN (125 kbps) | ESP32-S3 native TWAI | Direct | TX=7, RX=6 |

The MCP2515 also has a hardware reset (GPIO 9) and interrupt (GPIO 8) line. Pin definitions are in `include/credentials.h` and match the [official LilyGo T2-CAN pin_config.h](https://github.com/Xinyuan-LilyGO/T-2Can/blob/main/libraries/private_library/pin_config.h).

### I2C OLED Display (SH1106 128×64)

Connect via the QWIIC/I2C connector on the board.

**Default pin assignments (adjust in `credentials.h` if different):**

| Signal | GPIO |
|--------|------|
| I2C SDA | 17 |
| I2C SCL | 18 |

---

## OBD-II Wiring

The connector used for this build is an [OBD-II 16-pin breakout cable (Amazon UK)](https://www.amazon.co.uk/dp/B0F2HTKXRH?ref=ppx_yo2ov_dt_b_fed_asin_title), which exposes individual colour-coded wires for each pin.

### Wire colour reference (cable pin-out)

| OBD-II Pin | Wire Colour | Standard Function |
|:---:|---|---|
| 1 | Black | Manufacturer-defined |
| 2 | Brown | Manufacturer-defined |
| 3 | Red | MS-CAN High (Ford) |
| 4 | Orange | Chassis Ground |
| 5 | Yellow | Signal Ground |
| 6 | Green | HS-CAN High |
| 7 | Blue | K-Line / ISO 9141 |
| 8 | Purple | Manufacturer-defined |
| 9 | Grey | Manufacturer-defined |
| 10 | White | Manufacturer-defined |
| 11 | Pink | MS-CAN Low (Ford) |
| 12 | Light Green | Manufacturer-defined |
| 13 | Black/White | Manufacturer-defined |
| 14 | Brown/White | HS-CAN Low |
| 15 | Red/White | Manufacturer-defined |
| 16 | Green/White | Battery +12 V |

### T2-CAN connector wiring

#### CAN Bus A — HS-CAN (500 kbps, OBD-II standard)

| T2-CAN Pin | Signal | OBD-II Pin | Wire Colour |
|---|---|:---:|---|
| CANHA | CAN High | 6 | Green |
| CANLA | CAN Low | 14 | Brown/White |
| SGNDA | Signal Ground | 5 | Yellow |
| DGNDA | Digital Ground | 4 | Orange |

#### CAN Bus B — MS-CAN (125 kbps, Ford Medium Speed)

| T2-CAN Pin | Signal | OBD-II Pin | Wire Colour |
|---|---|:---:|---|
| CANHB | CAN High | 3 | Red |
| CANLB | CAN Low | 11 | Pink |
| SGNDB | Signal Ground | 5 | Yellow (shared with Bus A) |
| DGNDB | Digital Ground | 4 | Orange (shared with Bus A) |

#### Power

| T2-CAN Pin | Signal | OBD-II Pin | Wire Colour |
|---|---|:---:|---|
| VIN | +12 V battery | 16 | Green/White |
| GND | Power Ground | 4 | Orange |

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

## Ford Focus MK2 (2007–2008, UK) — Model-Specific Notes

This section documents the specific CAN bus architecture of the **Ford Focus MK2 C170 platform** (2005–2008, UK market) and confirms compatibility with this firmware.

### Dual CAN Bus Architecture

The Focus MK2 is one of Ford's earliest models to use a dual-bus CAN architecture. This is why a standard ELM327 dongle cannot access body modules without a hardware HS/MS-CAN toggle switch (as sold by ELMConfig/FORScan adapters) — a standard OBD-II dongle only sees pins 6 and 14 (HS-CAN).

| Bus | OBD-II Pins | Speed | Purpose |
|-----|------------|-------|---------|
| HS-CAN | Pin 6 (High), Pin 14 (Low) | 500 kbps | Powertrain, ABS, transmission |
| MS-CAN | Pin 3 (High), Pin 11 (Low) | 125 kbps | Body, comfort, instrument cluster |

> Pins 3 and 11 are **Ford-proprietary** — they carry no signal on non-Ford vehicles.

### Which Modules Live on Each Bus

**HS-CAN (500 kbps) — accessed by this firmware via MCP2515:**

| Module | Abbrev | What it provides |
|--------|--------|-----------------|
| Powertrain Control Module | PCM | RPM, speed, temps, MAF, throttle, DTCs |
| Transmission Control Module | TCM | Gear, torque |
| Anti-lock Braking System | ABS/HEC | Wheel speeds |
| Electric Power Assisted Steering | EHPAS | Steering load |
| Body Control Module | BCM | Some body signals |

**MS-CAN (125 kbps) — listened to passively by this firmware via TWAI:**

| Module | Abbrev | What it provides |
|--------|--------|-----------------|
| Generic Electronic Module | GEM | Lighting, wipers, central locking |
| Driver/Passenger Door Modules | DDM / PDM | Window, mirror control |
| Electronic Automatic Temp Control | EATC | Climate |
| Restraints Control Module | RCM | Airbag |
| Instrument Cluster | IC | Fuel gauge, odometer, warnings |
| Keyless Vehicle Module | KVM | Remote locking |
| Parking Aid Module | PAM | Sensors |

### OBD-II Diagnostic Protocol (HS-CAN)

The Focus MK2 UK uses **ISO 15765-4** (CAN 11-bit, 500 kbps) for OBD-II diagnostics. All standard Mode 01 PIDs are supported:

| PID | Parameter | Formula |
|-----|-----------|---------|
| 0x0C | Engine RPM | `((A×256)+B) / 4` |
| 0x0D | Vehicle speed | `A` km/h |
| 0x05 | Coolant temperature | `A − 40` °C |
| 0x0F | Intake air temperature | `A − 40` °C |
| 0x10 | MAF air flow | `((A×256)+B) / 100` g/s |
| 0x11 | Throttle position | `A × 100 / 255` % |
| 0x42 | Battery voltage | `((A×256)+B) / 1000` V |
| 0x5C | Oil temperature | `A − 40` °C |
| 0x01 | DTC status / MIL | Bit 7 of A = MIL on/off; bits 0–6 = DTC count |

Request address: `0x7DF` (functional broadcast to all ECUs)
Response address: `0x7E8` (PCM — engine ECU)

### MS-CAN Fuel Level (Unverified — Needs In-Car Confirmation)

The instrument cluster on the MS-CAN bus broadcasts fuel level as a periodic frame. CAN ID `0x420`, byte 0, scaled as `byte / 2.55` → % is **widely cited in Ford reverse-engineering communities** for this platform, but has not been independently verified against a production UK MK2.

**To verify or find the correct ID on your car:**

1. Enable serial logging and watch MS-CAN output — the firmware logs all unrecognised frame IDs at debug level
2. With a nearly-full tank, note which IDs are present
3. Remove ~5 litres, re-check — the ID whose byte 0 value changed is the fuel level frame
4. Update `MS_ID_FUEL_LEVEL` in `src/can_ms.cpp` if the ID differs

### Termination

Do **not** add 120 Ω termination resistors. The vehicle bus is already terminated at both ends (ECU and instrument cluster). The T2-CAN board has optional termination jumpers — leave them **open** when connected to an in-vehicle bus, otherwise you'll create a 60 Ω net impedance and cause bus errors.

### Known Limitations on This Platform

- **Oil temperature (PID 0x5C):** May not be supported on all MK2 engine variants. The 1.6 Duratec petrol does not have an oil temp sensor — PID queries will time out silently.
- **Battery voltage (PID 0x42):** Returns ECU supply voltage, not alternator voltage directly. Values are typically 13.8–14.4 V at idle with engine running.
- **MAF (PID 0x10):** Not present on diesel variants (use MAP instead). Queries will time out on TDCi engines.
- **MS-CAN availability:** The MS-CAN bus is only active with ignition on (position II). It goes silent within seconds of key-off, which will trigger the MS-CAN silence timeout in the firmware.

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

// T2-CAN V1.0 — MCP2515 SPI pins for HS-CAN
#define MCP2515_CS_PIN         10
#define MCP2515_RST_PIN        9
#define MCP2515_INT_PIN        8
#define SPI_SCLK_PIN           12
#define SPI_MOSI_PIN           11
#define SPI_MISO_PIN           13

// T2-CAN V1.0 — native TWAI pins for MS-CAN
#define CAN_MS_TX_PIN          7
#define CAN_MS_RX_PIN          6
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

# Volvo EX30 — EV OLED and BLE Telemetry

ESP32-S3 firmware for the Lilygo T-2Can. It passively listens to the native
TWAI controller on CAN B at 500 kbps, shows EV telemetry on the QWIIC OLED,
and exposes readings over BLE.

The CAN controller is configured in **listen-only mode**. This firmware does
not send CAN frames or issue UDS requests.

## Hardware and OBD-II Wiring

Connect the OBD-II cable to the T-2Can CAN B terminal block:

| OBD-II pin | Function | T-2Can terminal | GPIO |
|---|---|---|---:|
| 16 | +12 V battery | VIN | — |
| 4 / 5 | Ground | GND | — |
| 6 | HS-CAN High | CANHB | — |
| 14 | HS-CAN Low | CANLB | — |

Leave the board's 120 Ω termination jumpers **open** when connected to a
vehicle bus. Confirm the board revision and wiring against its schematic
before connecting it to a vehicle.

The native TWAI pin mapping is TX=GPIO 7 and RX=GPIO 6. The OLED is connected
through QWIIC using SDA=GPIO 17 and SCL=GPIO 18.

## Telemetry Decoding

The defaults in `include/credentials.h.example` use the example mappings from
the project request:

| Signal | Example CAN ID(s) | Byte and decoding |
|---|---|---|
| Speed | `0x201`, `0x128` | byte 1 × `EV_SPEED_SCALE_KMH` |
| Battery SoC | `0x318` | byte 0, accepted only when 0–100 |
| Power / regen | Disabled by default | Configure a verified frame ID and scale |

These are candidate layouts, not confirmed Volvo EX30 signal definitions.
Verify the IDs, byte positions, scaling, and signedness on the target vehicle
before relying on the readings. The power decoder expects a signed,
little-endian 16-bit value; positive is power draw and negative is regen.
Until a verified power frame is configured, the OLED and BLE report power as
unavailable rather than showing a fabricated value.

## BLE Service

The device advertises as `EX30_CAN_Gateway`. The service and characteristic
UUIDs are in `include/credentials.h.example`:

| Characteristic | Value |
|---|---|
| Speed | 1-byte unsigned km/h |
| SoC | 1-byte unsigned percent |
| Power | signed 16-bit little-endian tenths of kW |

Characteristics support reads and notifications. Values remain unavailable
until a valid matching CAN frame has been received.

## Build and Flash

1. Install PlatformIO and connect the board over USB.
2. Create the local configuration:

   ```sh
   cp include/credentials.h.example include/credentials.h
   ```

3. Verify CAN signal mappings in `include/credentials.h`.
4. Build and upload:

   ```sh
   pio run
   pio run --target upload
   ```

5. View serial output at 115200 baud:

   ```sh
   pio device monitor
   ```

The OLED shows speed, battery SoC, and power/regen when valid. If no CAN
frames arrive for five minutes, the firmware turns off the display and enters
deep sleep, waking when CAN RX goes low.

## Project Files

```text
include/credentials.h.example  Pin, UUID, and signal mapping template
platformio.ini                 PlatformIO configuration
src/main.cpp                   BLE, listen-only TWAI, telemetry, sleep
src/display.cpp                EV OLED output
```

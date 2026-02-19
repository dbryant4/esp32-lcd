# ESP32 LCD Arduino Sketches

This repo is intentionally cleaned down to the two active Arduino sketches:

- `arduino/JC4827W543_LVGLv9/JC4827W543_LVGLv9.ino`
- `arduino/I2C_Pin_Scanner_BME280/I2C_Pin_Scanner_BME280.ino`

## Table of Contents

- [Structure](#structure)
- [Firmware Dashboard Sketch](#firmware-dashboard-sketch)
- [Wi-Fi Provisioning (ESPHome-Style)](#wi-fi-provisioning-esphome-style)
- [Web UI and API](#web-ui-and-api)
- [Historical Data Storage](#historical-data-storage)
- [Firmware Versioning](#firmware-versioning)
- [I2C Scanner Sketch](#i2c-scanner-sketch)

## Structure

- `arduino/JC4827W543_LVGLv9/` - main LVGL dashboard firmware
- `arduino/I2C_Pin_Scanner_BME280/` - I2C/BME280 scanner firmware
- `scripts/fw-arduino.sh` - build/upload script for dashboard sketch
- `scripts/scan-i2c-pins.sh` - build/upload/monitor script for scanner sketch
- `scripts/version.sh` - semantic version management helper

## Firmware Dashboard Sketch

Uses ESP32S3 `huge_app` partition scheme via `scripts/fw-arduino.sh` so larger firmware images can compile/upload.

```bash
# install toolchain + libraries
./scripts/fw-arduino.sh deps

# build
./scripts/fw-arduino.sh build

# build + upload
./scripts/fw-arduino.sh upload --port /dev/cu.usbmodem101
```

## Wi-Fi Provisioning (ESPHome-Style)

The dashboard sketch now provisions Wi-Fi with an ESPHome-style recovery flow:

- Saved STA credentials are loaded from ESP32 `Preferences` (NVS) on boot.
- If no credentials exist or STA connect times out, the device starts a fallback AP.
- Fallback AP defaults:
  - SSID: `ESP32LCD-<chipid>`
  - Password: random easy-to-type words plus 2 digits (example: `maple-river-42`)
  - Captive portal + provisioning page: `http://192.168.4.1`
- Submitting credentials in the portal stores them in NVS and immediately retries STA mode.
- In AP mode, AP SSID/password/portal address are shown on the debug screen.
- When connected to STA Wi-Fi, time is synced via NTP and shown on the main screen.

Validation checklist:

1. Flash firmware and open Serial Monitor at `115200`.
2. On first boot, confirm logs indicate AP provisioning mode.
3. Connect to the fallback AP and submit router credentials at `http://192.168.4.1`.
4. Confirm device reconnects in STA mode and logs local IP.
5. Reboot and verify the device reconnects without entering AP mode.
6. Power off router/AP temporarily and confirm reconnect or fallback behavior without UI lockups.

## Web UI and API

When connected (STA or AP mode), the web UI at `http://<device-ip>/` shows:

- **Current settings**: Wi-Fi, timezone, brightness
- **Live readings**: Temperature, humidity, pressure (refreshed every 5 seconds)
- **7-day history**: Table of recent samples and a CSV download link

**API endpoints** (JSON):

- `GET /api/readings` — Current sensor values (temp_c, temp_f, humidity_pct, pressure_hpa, timestamp, valid)
- `GET /api/history` — 7-day history; use `?res=hourly` for 168 points
- `GET /api/status` — Settings plus current readings
- `GET /history.csv` — Download full 7-day history as CSV

History is stored at 5-minute resolution with NTP timestamps. Gaps (device off) are marked with `valid: false` in the API and omitted from the CSV.

## Historical Data Storage

Historical sensor data is stored entirely on the ESP32:

- **RAM (ring buffer)**: 2016 slots in a circular buffer (~20 KB). Each slot holds: Unix timestamp, temperature (0.1°C), humidity (%), pressure (hPa), and a valid flag. Data is written every 5 minutes as an average of the 1-second BME280 readings.
- **Flash (NVS)**: The ring buffer is persisted to non-volatile storage (Preferences namespace `history`) every 30 minutes. On boot, the buffer is restored so history survives power cycles.
- **Gap handling**: When the device was off, missed 5-minute intervals are filled with gap entries (`valid: false`) so the timeline stays correct. Gaps are not written to the CSV download.
- **Resolution**: 7 days at 5-minute intervals (12 samples/hour × 24 hours × 7 days = 2016 slots).

## Firmware Versioning

- Semantic version source of truth is in `arduino/JC4827W543_LVGLv9/JC4827W543_LVGLv9.ino`:
  - `FW_VERSION_MAJOR`
  - `FW_VERSION_MINOR`
  - `FW_VERSION_PATCH`
- Current version: `1.2.0`.
- The debug screen shows `Firmware: v<major.minor.patch>`.

```bash
# print current version
./scripts/version.sh show

# set explicit version
./scripts/version.sh set 1.1.0

# bump semantic part
./scripts/version.sh bump patch
./scripts/version.sh bump minor
./scripts/version.sh bump major

# chat workflow helper: minor++, patch=0
./scripts/version.sh new-chat
```

## I2C Scanner Sketch

```bash
# build + upload scanner
./scripts/scan-i2c-pins.sh upload --port /dev/cu.usbmodem101

# monitor scanner output
./scripts/scan-i2c-pins.sh monitor --port /dev/cu.usbmodem101
```

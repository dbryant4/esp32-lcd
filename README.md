# ESP32 LCD Arduino Sketches

This repo is intentionally cleaned down to the two active Arduino sketches:

- `arduino/JC4827W543_LVGLv9/JC4827W543_LVGLv9.ino`
- `arduino/I2C_Pin_Scanner_BME280/I2C_Pin_Scanner_BME280.ino`

## Table of Contents

- [Structure](#structure)
- [Firmware Dashboard Sketch](#firmware-dashboard-sketch)
- [Wi-Fi Provisioning (ESPHome-Style)](#wi-fi-provisioning-esphome-style)
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

## Firmware Versioning

- Semantic version source of truth is in `arduino/JC4827W543_LVGLv9/JC4827W543_LVGLv9.ino`:
  - `FW_VERSION_MAJOR`
  - `FW_VERSION_MINOR`
  - `FW_VERSION_PATCH`
- Current version: `1.1.0`.
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

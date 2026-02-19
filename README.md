# ESP32 LCD Arduino Sketches

This repo is intentionally cleaned down to the two active Arduino sketches:

- `arduino/JC4827W543_LVGLv9/JC4827W543_LVGLv9.ino`
- `arduino/I2C_Pin_Scanner_BME280/I2C_Pin_Scanner_BME280.ino`

## Table of Contents

- [Structure](#structure)
- [Firmware Dashboard Sketch](#firmware-dashboard-sketch)
- [I2C Scanner Sketch](#i2c-scanner-sketch)

## Structure

- `arduino/JC4827W543_LVGLv9/` - main LVGL dashboard firmware
- `arduino/I2C_Pin_Scanner_BME280/` - I2C/BME280 scanner firmware
- `scripts/fw-arduino.sh` - build/upload script for dashboard sketch
- `scripts/scan-i2c-pins.sh` - build/upload/monitor script for scanner sketch

## Firmware Dashboard Sketch

```bash
# install toolchain + libraries
./scripts/fw-arduino.sh deps

# build
./scripts/fw-arduino.sh build

# build + upload
./scripts/fw-arduino.sh upload --port /dev/cu.usbmodem101
```

## I2C Scanner Sketch

```bash
# build + upload scanner
./scripts/scan-i2c-pins.sh upload --port /dev/cu.usbmodem101

# monitor scanner output
./scripts/scan-i2c-pins.sh monitor --port /dev/cu.usbmodem101
```

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SKETCH_PATH="$PROJECT_ROOT/arduino/I2C_Pin_Scanner_BME280/I2C_Pin_Scanner_BME280.ino"
BUILD_DIR="$PROJECT_ROOT/build-arduino-i2c-scan"
FQBN="esp32:esp32:esp32s3"
PORT="/dev/cu.usbmodem101"

ACTION="${1:-upload}"
shift || true

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port)
      PORT="${2:?--port requires a value}"
      shift 2
      ;;
    *)
      echo "Unknown option: $1" >&2
      exit 1
      ;;
  esac
done

if [[ ! -f "$SKETCH_PATH" ]]; then
  echo "Sketch not found: $SKETCH_PATH" >&2
  exit 1
fi

ensure_core() {
  arduino-cli config init >/dev/null 2>&1 || true
  arduino-cli config set board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
  arduino-cli core update-index
  arduino-cli core install esp32:esp32
  arduino-cli lib install "GFX Library for Arduino"
  arduino-cli lib install "Dev Device Pins"
  arduino-cli lib install "Adafruit BME280 Library"
  arduino-cli lib install "Adafruit Unified Sensor"
}

compile_only() {
  mkdir -p "$BUILD_DIR"
  arduino-cli compile --fqbn "$FQBN" --build-path "$BUILD_DIR" "$SKETCH_PATH"
}

upload_only() {
  arduino-cli upload -p "$PORT" --fqbn "$FQBN" --input-dir "$BUILD_DIR" "$SKETCH_PATH"
}

monitor_only() {
  arduino-cli monitor -p "$PORT" -c baudrate=115200
}

case "$ACTION" in
  build)
    ensure_core
    compile_only
    ;;
  upload)
    ensure_core
    compile_only
    upload_only
    ;;
  monitor)
    monitor_only
    ;;
  *)
    echo "Usage: $0 [build|upload|monitor] [--port /dev/cu.usbmodemXXX]" >&2
    exit 1
    ;;
esac


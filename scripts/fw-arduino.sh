#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SKETCH_PATH="$PROJECT_ROOT/arduino/JC4827W543_LVGLv9/JC4827W543_LVGLv9.ino"
BUILD_DIR="$PROJECT_ROOT/build-arduino"
FQBN="esp32:esp32:esp32s3:PartitionScheme=huge_app"
PORT="/dev/cu.usbmodem101"

if [[ ! -f "$SKETCH_PATH" ]]; then
  echo "Sketch not found: $SKETCH_PATH" >&2
  exit 1
fi

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

install_deps() {
  arduino-cli config init >/dev/null 2>&1 || true
  arduino-cli config set board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
  arduino-cli core update-index
  arduino-cli core install esp32:esp32

  arduino-cli lib install "lvgl"
  arduino-cli lib install "GFX Library for Arduino"
  arduino-cli lib install "Dev Device Pins"
  arduino-cli lib install "TAMC_GT911"
  arduino-cli lib install "Adafruit BME280 Library"
  arduino-cli lib install "Adafruit Unified Sensor"
}

compile_only() {
  rm -rf "$BUILD_DIR"
  arduino-cli compile --fqbn "$FQBN" --build-path "$BUILD_DIR" "$SKETCH_PATH"
}

upload_firmware() {
  compile_only
  arduino-cli upload -p "$PORT" --fqbn "$FQBN" --input-dir "$BUILD_DIR" "$SKETCH_PATH"
}

case "$ACTION" in
  deps)
    install_deps
    ;;
  build)
    compile_only
    ;;
  upload)
    install_deps
    upload_firmware
    ;;
  *)
    echo "Usage: $0 [deps|build|upload] [--port /dev/cu.usbmodemXXX]" >&2
    exit 1
    ;;
esac


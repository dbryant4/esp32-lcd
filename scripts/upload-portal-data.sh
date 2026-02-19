#!/usr/bin/env bash
# Upload portal HTML/JS from data/ to ESP32 SPIFFS (huge_app partition).
# Run after firmware upload. Requires: Arduino ESP32 package, esptool.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DATA_DIR="$PROJECT_ROOT/arduino/JC4827W543_LVGLv9/data"
PORT="${PORT:-/dev/cu.usbmodem101}"
# huge_app partition: spiffs at 0x310000, size 0xE0000 (896KB)
SPIFFS_OFFSET=0x310000
SPIFFS_SIZE=0xE0000
MKSPIFFS="${ARDUINO15:-$HOME/Library/Arduino15}/packages/esp32/tools/mkspiffs/0.2.3/mkspiffs"

if [[ ! -d "$DATA_DIR" ]]; then
  echo "Data dir not found: $DATA_DIR" >&2
  exit 1
fi

if [[ ! -f "$MKSPIFFS" ]]; then
  echo "mkspiffs not found. Install Arduino ESP32 package: ./scripts/fw-arduino.sh deps" >&2
  exit 1
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port)
      PORT="${2:?--port requires value}"
      shift 2
      ;;
    *)
      echo "Unknown option: $1" >&2
      exit 1
      ;;
  esac
done

echo "Creating SPIFFS image from $DATA_DIR..."
SPIFFS_BIN="$PROJECT_ROOT/build-arduino/spiffs.bin"
mkdir -p "$(dirname "$SPIFFS_BIN")"
"$MKSPIFFS" -c "$DATA_DIR" -b 4096 -p 256 -s "$SPIFFS_SIZE" "$SPIFFS_BIN"

echo "Flashing SPIFFS to $PORT at offset $SPIFFS_OFFSET..."
esptool.py --chip esp32s3 -p "$PORT" write_flash "$SPIFFS_OFFSET" "$SPIFFS_BIN"

echo "Portal data uploaded."

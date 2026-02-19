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

release_build() {
  compile_only
  local bin_src="$BUILD_DIR/JC4827W543_LVGLv9.ino.merged.bin"
  local major minor patch version
  major="$(sed -n 's/^#define FW_VERSION_MAJOR[[:space:]]*\([0-9]*\).*/\1/p' "$SKETCH_PATH" | head -1)"
  minor="$(sed -n 's/^#define FW_VERSION_MINOR[[:space:]]*\([0-9]*\).*/\1/p' "$SKETCH_PATH" | head -1)"
  patch="$(sed -n 's/^#define FW_VERSION_PATCH[[:space:]]*\([0-9]*\).*/\1/p' "$SKETCH_PATH" | head -1)"
  version="${major:-0}.${minor:-0}.${patch:-0}"
  local bin_dest="$PROJECT_ROOT/JC4827W543_LVGLv9-${version}.bin"
  cp "$bin_src" "$bin_dest"
  echo "Release binary: $bin_dest"
}

case "$ACTION" in
  deps)
    install_deps
    ;;
  build)
    compile_only
    ;;
  release)
    release_build
    ;;
  upload)
    install_deps
    upload_firmware
    ;;
  *)
    echo "Usage: $0 [deps|build|release|upload] [--port /dev/cu.usbmodemXXX]" >&2
    exit 1
    ;;
esac


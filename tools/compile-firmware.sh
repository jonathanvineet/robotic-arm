#!/bin/sh
# Compile-check the firmware for ESP8266 (NodeMCU) and ESP32 with arduino-cli.
# Usage: sh tools/compile-firmware.sh [esp8266|esp32|all]
set -e
CLI="${ARDUINO_CLI:-$(command -v arduino-cli || echo "/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli")}"
DIR="$(cd "$(dirname "$0")/.." && pwd)/firmware/RoboArm"
target="${1:-all}"
if [ "$target" = esp8266 ] || [ "$target" = all ]; then
  echo "== ESP8266 (NodeMCU 1.0)"; "$CLI" compile --fqbn esp8266:esp8266:nodemcuv2 "$DIR" 2>&1 | tail -4
fi
if [ "$target" = esp32 ] || [ "$target" = all ]; then
  echo "== ESP32 Dev Module"; "$CLI" compile --fqbn esp32:esp32:esp32 "$DIR" 2>&1 | tail -4
fi

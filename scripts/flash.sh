#!/usr/bin/env bash
# Build the car firmware and flash it onto the ESP32 DevKit over USB serial,
# then open its log at 115200 baud.
#
# Usage: scripts/flash.sh [-p] [PORT]
#   -p     pristine build (clean first)
#   PORT   serial port, e.g. /dev/ttyUSB0 (default: auto-detected by PlatformIO)
#
# Uses pio from PATH, or the one installed by the PlatformIO IDE extension.
# The fob is a XIAO nRF52840: flash it from the nrf52840/main branch.
set -euo pipefail

PRISTINE=0
if [ "${1:-}" = "-p" ]; then
	PRISTINE=1
	shift
fi
PORT=()
[ -n "${1:-}" ] && PORT=(--upload-port "$1" --monitor-port "$1")

ROOT=$(cd "$(dirname "$0")/.." && pwd)
[ -f "$ROOT/common/keys.h" ] || { echo "common/keys.h is missing: run scripts/gen_keys.py first" >&2; exit 1; }

PIO=$(command -v pio || echo "$HOME/.platformio/penv/bin/pio")
[ -x "$PIO" ] || { echo "PlatformIO not found; install it or put pio on PATH" >&2; exit 1; }

[ "$PRISTINE" = 1 ] && "$PIO" run -d "$ROOT/car" -t clean
"$PIO" run -d "$ROOT/car" -t upload -t monitor "${PORT[@]}"

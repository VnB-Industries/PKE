#!/usr/bin/env bash
# Build one app and flash it onto a XIAO nRF52840 through its UF2 bootloader.
#
# Usage: scripts/flash.sh fob|car [-p]
#   -p   pristine build
#
# Runs from any shell. When west is not on PATH, the nRF Connect toolchain is
# loaded from $NCS_TOOLCHAIN (default: the newest ~/ncs/toolchains/*), and the
# SDK is taken from $NCS (default: the newest ~/ncs/v*).
set -euo pipefail

APP=${1:-}
case "$APP" in
fob | car) ;;
*)
	echo "usage: $0 fob|car [-p]" >&2
	exit 2
	;;
esac
PRISTINE=()
[ "${2:-}" = "-p" ] && PRISTINE=(-p)

ROOT=$(cd "$(dirname "$0")/.." && pwd)
NCS=${NCS:-$(ls -d "$HOME"/ncs/v* 2>/dev/null | sort -V | tail -1)}
[ -d "$NCS" ] || { echo "nRF Connect SDK not found; set NCS" >&2; exit 1; }

if ! command -v west >/dev/null; then
	TC=${NCS_TOOLCHAIN:-$(ls -dt "$HOME"/ncs/toolchains/*/ 2>/dev/null | head -1)}
	[ -f "$TC/environment.json" ] || { echo "nRF Connect toolchain not found; set NCS_TOOLCHAIN" >&2; exit 1; }
	# Same environment as "nrfutil sdk-manager toolchain launch --shell".
	eval "$(python3 - "$TC" <<'EOF'
import json, os, shlex, sys
tc = sys.argv[1].rstrip("/")
for var in json.load(open(os.path.join(tc, "environment.json")))["env_vars"]:
    if var["type"] == "relative_paths":
        value = ":".join(os.path.join(tc, p) for p in var["values"])
    elif var["type"] == "relative_path":
        value = os.path.join(tc, var["value"])
    else:
        value = var["value"]
    if var.get("existing_value_treatment") == "prepend_to":
        value += ":" + os.environ.get(var["key"], "")
    print(f"export {var['key']}={shlex.quote(value)}")
EOF
)"
fi

# west only finds its build and flash commands from inside the SDK workspace.
cd "$NCS"
west build "${PRISTINE[@]}" -b xiao_ble/nrf52840 "$ROOT/$APP" -d "$ROOT/build/$APP"

echo
echo ">>> Double-tap reset on the $APP board (only that one)."
INFO=
for _ in $(seq 60); do
	INFO=$(compgen -G "/media/$USER/*/INFO_UF2.TXT" || compgen -G "/run/media/$USER/*/INFO_UF2.TXT" || true)
	[ -n "$INFO" ] && break
	sleep 1
done
[ -n "$INFO" ] || { echo "no UF2 bootloader drive found" >&2; exit 1; }
# Plain cp instead of west flash: the bootloader reboots and unmounts the drive as soon as
# the image is written, and west's uf2 runner then fails on a chmod of the vanished file.
DRIVE=$(dirname "$(head -1 <<<"$INFO")")
cp "$ROOT/build/$APP/$APP/zephyr/zephyr.uf2" "$DRIVE/" 2>/dev/null || [ ! -d "$DRIVE" ]
echo ">>> Flashed $APP via $DRIVE"

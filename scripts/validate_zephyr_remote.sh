#!/usr/bin/env bash
set -euo pipefail

remote_host="${REMOTE_HOST:-w530}"
remote_root="${REMOTE_ROOT:-/tmp/attitudeMathLibrary-validation}"
board="${BOARD:-nrf5340dk_nrf5340_cpuapp}"
jlink_serial="${JLINK_SERIAL:-960169267}"
ncs_env="${NCS_ENV:-$HOME/ncs/toolchains/1f9b40e71a/env.sh}"
zephyr_base="${ZEPHYR_BASE_REMOTE:-$HOME/ncs/v2.4.0/zephyr}"
serial_interface="${SERIAL_INTERFACE:-if04}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

rsync -az \
    --exclude='.git' \
    --exclude='build*' \
    "$project_root/" "$remote_host:$remote_root/"

ssh "$remote_host" bash -s -- \
    "$remote_root" "$board" "$jlink_serial" "$ncs_env" "$zephyr_base" "$serial_interface" <<'REMOTE'
set -euo pipefail

remote_root="$1"
board="$2"
jlink_serial="$3"
ncs_env="$4"
zephyr_base="$5"
serial_interface="$6"
build_dir="$remote_root/build_$board"

set +u
source "$ncs_env" >/dev/null 2>&1
set -u
export ZEPHYR_BASE="$zephyr_base"

west build -p always -b "$board" "$remote_root/samples/zephyr" -d "$build_dir"

if [[ "$board" != nrf5340dk_nrf5340_cpuapp ]]; then
    echo "ATTITUDE_VALIDATION BUILD PASS board=$board"
    exit 0
fi

serial_device="/dev/serial/by-id/usb-SEGGER_J-Link_000${jlink_serial}-${serial_interface}"
if [[ ! -e "$serial_device" ]]; then
    echo "Missing serial device: $serial_device" >&2
    exit 1
fi

west flash -d "$build_dir" --dev-id "$jlink_serial"
stty -F "$serial_device" 115200 raw -echo

capture_file="$(mktemp)"
trap 'rm -f "$capture_file"' EXIT
(sleep 0.3; nrfjprog --reset --family NRF53 --coprocessor CP_APPLICATION \
    --snr "$jlink_serial" >/dev/null) &
timeout 5s stdbuf -o0 cat "$serial_device" | tr -d '\r' | tee "$capture_file" || true
wait

grep -Fq 'ATTITUDE_VALIDATION PASS' "$capture_file"
REMOTE

#!/usr/bin/env bash
# flash_and_watch.sh - flash a BTF and capture the boot trace.
#
#   tools/flash_and_watch.sh images/01-feature-DEBUG.BTF [/dev/cu.usbserial-10]
#
# Put the radio in bootloader mode first: hold the bottom two side buttons
# while powering on. Then run this, and power-cycle when prompted.
#
# Exists because two separate mistakes cost a whole debugging session:
#
#   1. A monitor left running in another window kept the serial port open.
#      Uploads then failed with "Wrong data length" at a different block each
#      time, which reads exactly like a radio-side protocol fault. This script
#      kills any monitor before it uploads.
#
#   2. The monitor's output was piped without flushing, so a few hundred bytes
#      of boot trace sat in Python's 8 KB block buffer and never appeared. Four
#      empty captures led to "this board has no debug UART". It had one.
#      Fixed in firmware_upload.py; `python3 -u` here belts-and-braces it.
#
# The trace is written to a log AND shown live, so a hang leaves evidence.

set -u

BTF="${1:?usage: flash_and_watch.sh <file.BTF> [port]}"
PORT="${2:-/dev/cu.usbserial-10}"
HERE="$(cd "$(dirname "$0")" && pwd)"
LOG="${HERE}/../logs/boot-$(basename "${BTF%.BTF}").log"

mkdir -p "$(dirname "$LOG")"

if [ ! -f "$BTF" ]; then
    echo "no such image: $BTF" >&2
    exit 1
fi

echo "==> releasing the serial port"
pkill -f "firmware_upload.py monitor" 2>/dev/null || true
sleep 1
if command -v lsof >/dev/null && lsof "$PORT" >/dev/null 2>&1; then
    echo "WARNING: something still has $PORT open:" >&2
    lsof "$PORT" >&2
    echo "Close it before continuing, or the upload will drop bytes." >&2
    exit 1
fi

# Two ways into the bootloader. Try the soft one first: any firmware carrying
# the update listener hands itself over when it sees the handshake, so no side
# buttons and no battery pull. Fall back to assuming the radio is already in
# bootloader mode, which is what the side-button entry gives you.
echo "==> checking for bootloader on $PORT"
if python3 "$HERE/firmware_upload.py" probe "$PORT" 2>&1 | grep -q "bootloader mode"; then
    echo "    already in bootloader mode"
    MODE="--ptt"
else
    echo "    not in bootloader; trying the soft handshake"
    MODE=""
fi

echo "==> uploading $(basename "$BTF")"
if ! python3 "$HERE/firmware_upload.py" upload "$PORT" "$BTF" $MODE; then
    echo >&2
    echo "Upload failed." >&2
    echo "If the running firmware is too broken to answer the handshake, use the" >&2
    echo "guaranteed path: hold the bottom two side buttons while powering on," >&2
    echo "then run this again." >&2
    exit 1
fi

echo
echo "==> POWER-CYCLE THE RADIO NOW"
echo "==> capturing to $LOG   (Ctrl-C to stop)"
echo

# -u as well as the in-tool flush: this output is the whole point of the run.
python3 -u "$HERE/firmware_upload.py" monitor "$PORT" 2>&1 | tee "$LOG"

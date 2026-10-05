#!/usr/bin/env bash
# Build and flash the standalone RSSI meter, self-service (no buttons).
#
# Usage: tools/flash_rssi_meter.sh [port] [--no-build] [--manual]
#   port:      default /dev/cu.usbmodem1101
#   --no-build skip the rebuild, flash the existing build_rssi_meter_4mb/
#              The image header must match the board flash size: an 8MB-stamped
#              app on this 4MB devkit asserts in esp_flash_spi_init at boot and
#              crash-loops, which also tears down the JTAG debug module.
#   --manual   the board is ALREADY in download mode (button sequence done)
#              - skip ROM entry entirely.
#
# ROM entry, in order of preference:
#   1. X on USB: the meter arms the external auto-download circuit
#      (diode+cap+MOSFET on GPIO10/BOOT, docs/rssi-meter.md) and restarts
#      into download mode. Deterministic, no host-reset dependency.
#   2. esptool usb_reset (DTR/RTS): works when the host USB stack is
#      healthy; flaky on some Macs.
#   3. Manual: hold BOOT, tap RESET, hold BOOT ~2 s, let go, rerun with
#      --manual.
#
# Flash finish is always --after watchdog_reset (the chip's own ROM
# restarts itself, identical to a power cycle): the new image boots and
# streams immediately. Never finish with hard_reset here.
set -euo pipefail
cd "$(dirname "$0")/.."

# Port auto-detect: a board with a real USB-to-UART bridge enumerates as
# /dev/cu.usbserial-*; the built-in USB-Serial/JTAG enumerates as
# /dev/cu.usbmodem*. Prefer the bridge, since that is the path with reliably
# wired DTR/RTS (BOOT and EN) and the one RotorHazard expects for a race timer.
PORT="${ESPTOOL_PORT_OVERRIDE:-}"
if [ -z "$PORT" ]; then
  for p in /dev/cu.usbserial-* /dev/cu.usbmodem*; do
    if [ -e "$p" ]; then PORT="$p"; break; fi
  done
fi
[ -n "$PORT" ] || PORT="/dev/cu.usbserial-110"
NO_BUILD=0
MANUAL=0
for arg in "$@"; do
  case "$arg" in
    --no-build) NO_BUILD=1 ;;
    --manual)   MANUAL=1 ;;
    *)          PORT="$arg" ;;
  esac
done

if [ "$NO_BUILD" -eq 0 ]; then
  echo ">> building meter image"
  bash -c ". ~/esp/esp-idf/export.sh > /dev/null 2>&1 && \
    idf.py -B build_rssi_meter_4mb \
      -D SDKCONFIG=sdkconfig.rssi-meter-4mb \
      -D \"SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.rssi-meter-4mb.defaults\" \
      build"
fi

flash() {
  echo ">> flashing (watchdog_reset finish)"
  bash -c ". ~/esp/esp-idf/export.sh > /dev/null 2>&1 && \
    python -m esptool --chip esp32c5 --port $PORT --baud 460800 \
      --before $1 --after watchdog_reset write_flash \
      --flash-mode dio --flash-freq 80m \
      0x2000 build_rssi_meter_4mb/bootloader/bootloader.bin \
      0x8000 build_rssi_meter_4mb/partition_table/partition-table.bin \
      0x10000 build_rssi_meter_4mb/c5vrx3.bin"
}

if [ "$MANUAL" -eq 1 ]; then
  echo ">> ROM entry: manual (assumed)"
  flash no_reset
elif [ -e "$PORT" ] && case "$PORT" in /dev/cu.usbserial-*) true ;; *) false ;; esac; then
  # A real USB-to-UART bridge has DTR and RTS physically wired to BOOT and EN,
  # so a normal reset reliably lands in download mode and no external circuit
  # is needed. This is why the UART port on the devkit never needed a manual
  # reset, while the built-in USB-Serial/JTAG path did.
  echo ">> ROM entry: hardware DTR/RTS (USB-to-UART bridge)"
  flash default_reset
elif python3 - "$PORT" <<'PYEOF'
import serial, sys, time
try:
    s = serial.Serial(sys.argv[1], 115200, timeout=1)
    s.reset_input_buffer()
    time.sleep(0.3)
    s.write(b"X")
    time.sleep(0.7)
    s.close()
except Exception:
    sys.exit(1)
sys.exit(0)
PYEOF
then
  echo ">> ROM entry: X sent; trying no_reset connect"
  sleep 2
  if flash no_reset; then :; else
    echo ">> X did not land in download mode (circuit not fitted?); trying usb_reset"
    flash usb_reset
  fi
else
  echo ">> X could not be sent (meter not running?); trying usb_reset"
  flash usb_reset
fi

echo ">> verifying stream"
python3 - "$PORT" <<'PYEOF'
import serial, sys, time
port = sys.argv[1]
time.sleep(3)
s = serial.Serial(port, 115200, timeout=1)
s.reset_input_buffer()
time.sleep(2)
buf = s.read(8192)
s.close()
lines = [l for l in buf.decode("ascii", "replace").splitlines() if l.startswith("R:")]
if lines:
    print(f"   OK: streaming ({len(lines)} samples in 2 s); latest: {lines[-1]}")
else:
    print("   WARN: no R: lines. Manual sequence: hold BOOT, tap RESET,")
    print("   hold BOOT ~2 s, let go; rerun with --manual.")
PYEOF
echo ">> done."

#!/usr/bin/env python3
"""Flash C5VRX firmware to the ESP32-C5 board.

Usage: python tools/flash.py COM10
       python tools/flash.py COM10 --build-dir build_rssi_meter
"""

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def build_flash_size(build_dir: Path) -> str:
    """Flash size stamped by the build, never a hardcoded one.

    The bootloader/app image header must match the board's real flash size: an
    8MB-stamped bootloader on a 4MB devkit asserts in esp_flash_spi_init at
    boot and the board crash-loops until it is power-cycled. Hardcoding 8MB
    here re-stamped a 4MB build back to 8MB and caused exactly that.
    """
    try:
        text = (build_dir / "config" / "sdkconfig.h").read_text()
    except OSError:
        return "keep"
    for line in text.splitlines():
        if line.startswith("#define CONFIG_ESPTOOLPY_FLASHSIZE "):
            value = line.split(None, 2)[2].strip().strip('"')
            if value:
                return value
    return "keep"


def main(port: str, build_dir: Path) -> None:
    files = [build_dir / "bootloader" / "bootloader.bin",
             build_dir / "partition_table" / "partition-table.bin",
             build_dir / "c5vrx3.bin"]
    missing = [str(path) for path in files if not path.exists()]
    if missing:
        raise SystemExit(f"Missing build artifacts: {missing}. Run: idf.py build")

    command = [
        "python", "-m", "esptool",
        "--chip", "esp32c5",
        "--port", port,
        "--baud", "460800",
        "--before", "usb_reset",
        "--after", "watchdog_reset",
        "write_flash",
        "--flash-mode", "dio",
        "--flash-size", build_flash_size(build_dir),
        "--flash-freq", "80m",
        "0x2000", str(files[0]),
        "0x8000", str(files[1]),
        "0x10000", str(files[2]),
    ]
    subprocess.check_call(command)


if __name__ == "__main__":
    args = sys.argv[1:]
    build_dir = ROOT / "build"
    port = None
    i = 0
    while i < len(args):
        if args[i] == "--build-dir" and i + 1 < len(args):
            path = Path(args[i + 1])
            build_dir = path if path.is_absolute() else ROOT / path
            i += 2
        else:
            port = args[i]
            i += 1
    if port is None:
        raise SystemExit(
            "Usage: python tools/flash.py COM10 [--build-dir build_rssi_meter]")
    main(port, build_dir)

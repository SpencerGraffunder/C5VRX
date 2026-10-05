# C5VRX — RSSI meter fork

This is a fork of [Twotoz/C5VRX](https://github.com/Twotoz/C5VRX) carrying a
single branch: **`feat/rssi-meter`** — a standalone ~1 kHz RSSI meter firmware
for the ESP32-C5 5.8 GHz receiver, built for use as a **drone race timer**
input. Everything else in this repo is upstream code, unchanged.

## Purpose

A classic analog VRX exposes an RSSI pin that reports the strength of the
tuned channel; race timers use it to sense a drone's proximity. This fork
turns the C5VRX receiver into the same thing without any video:

- The receiver boots the exact same proven RF state as the upstream video
  firmware, but the entire video pipeline (PARLIO RX, BitScrambler demod,
  menu, video DAC stream) is compiled out.
- It tunes to **5800 MHz** by default (retunable at runtime) and reports
  channel power at **~1 kHz** on both:
  - **USB CDC serial**, one line per sample:
    `R:-72 NF:-95 G:52 M:0` — `R` = signal strength in dBm,
    `NF` = noise floor (the no-transmitter baseline), `G` = internal
    amplifier index, `M` = amplifier mode (0 = fixed, 1 = hardware
    auto-gain).
  - The **6-bit analog DAC** on the same D4..D9 resistor network the video
    output uses, so a timer MCU can read a plain analog voltage exactly like
    a VRX RSSI pin (−100…−40 dBm mapped to 0..63, clamped).
- 1 kHz is far more than a race car needs; decimate on the timer side.

## Changes vs upstream (`main`)

| file | change |
|---|---|
| `main/rssi_meter_main.c` | new — the meter application: 1 kHz RSSI/noise-floor streamer, analog outputs, dual-port console, serial commands |
| `main/rssi_pipeline.c/.h` | new — signal shaping: peak-hold window, median3, EMA, soft-knee compression, calibrated dBm→0..255 mapping, learned quiet baseline + signal-present detection |
| `main/rx5808_bus.c/.h` | new — RX5808 3-wire bus (SEL/CLK/DATA) so an existing lap timer can tune and read this receiver like a real VRX |
| `main/rssi_sdm.c/.h` | new — sigma-delta (1-bit, 4 MHz) analog RSSI output into an RC filter for ~8-bit resolution |
| `main/rh_node.c/.h` | new — RotorHazard USB node protocol (api level 35) so the meter can act as a timing node |
| `main/rf.c`, `main/rf.h` | new `rf_set_frequency_mhz()` — arbitrary 5 GHz retune (nearest public Wi-Fi center bootstrap + `phy_set_freq()` delta + gain/BW re-assert). This is also the planned SPI frequency-control entry point |
| `main/Kconfig.projbuild` | new `CONFIG_C5VRX_RSSI_METER` build option plus meter output/bus/console options |
| `main/CMakeLists.txt` | meter image builds only the meter sources and no `.bsasm` programs; the normal image builds exactly as before |
| `sdkconfig.rssi-meter.defaults`, `sdkconfig.rssi-meter-4mb.defaults` | new — meter image configs (the 4MB one carries the flash-size stamp the image header must match) |
| `docs/rssi-meter.md` | new — protocol, commands, build/flash instructions, bench plan, bench results |
| `tools/flash.py`, `tools/flash_rssi_meter.sh` | `--build-dir` for alternate images; a self-service build+flash loop for the meter |
| `tools/test_rssi_meter.c`, `tools/test_rh_node.c` | new — host unit tests for the pipeline, bus and node protocol (no board needed) |
| `.gitignore` | meter build dirs / generated config / release package |

The normal video firmware is not modified and builds unchanged; the meter is
a separate image. Flash the normal firmware again to restore video.

## Meter serial commands

Commands work on **either** console port — the board's USB-UART bridge or its
built-in USB-Serial/JTAG — and the answer comes back on the port you typed on.

| key | action |
|---|---|
| `H` | help |
| `T` | status line (frequency, mode, state, calibration, signal-present, DAC polarity, console ports) |
| `N` | toggle hardware auto-gain (persisted, reboots) |
| `S` | fixed-amplifier sweep G15..G81 (characterizes the RSSI measurement, restores the previous setting) |
| `F<mhz>` / `F R4` | retune, e.g. `F5800`, or a named FPV channel |
| `SCAN [ms]` | measure all 48 FPV channels, report the strongest |
| `CH R4` | tune to a named channel |
| `C [lo hi]` | show / set the calibration dB window |
| `K db r` / `E alpha` / `W ms` | soft knee / EMA / peak-hold window |
| `A [db]` | signal-present margin over the learned quiet baseline |
| `R` | forget the learned baseline and re-learn it |
| `OUT n` / `OUT off` | fix the analog output level (wiring test) / restore live value |
| `U` | RX5808 bus statistics |
| `P` / `D` | save settings to NVS / restore defaults |
| `V` | invert DAC polarity |
| `L` | pause / resume the 1 kHz stream |
| `M` | RotorHazard USB node mode (persisted, reboots) |
| `B` | reboot the meter |
| `X` | arm the external auto-download circuit, then restart (for flashing) |

## Hosting and flashing

The upstream production web flasher is hosted entirely by **GitHub Pages** at
[https://twotoz.github.io/C5VRX/](https://twotoz.github.io/C5VRX/). There is no VPS
and no proxy application server: `deploy-web.yml` is the only production web
deployment, it always checks out trusted `main`, and firmware assets are
mirrored into the Pages artifact so the browser downloads same-origin from
[https://twotoz.github.io/C5VRX/](https://twotoz.github.io/C5VRX/).

That Pages site mirrors the **upstream** repository's releases and PR builds.
This fork does not deploy its own web flasher: the meter image is built and
flashed locally (`tools/flash_rssi_meter.sh`, or the packaged
`release-rssi-meter/` archive), and no pull request is ever opened from this
fork against `Twotoz/C5VRX`.

## Build (ESP-IDF v6.0.2, same as upstream)

```sh
idf.py -B build_rssi_meter \
  -D SDKCONFIG=sdkconfig.rssi-meter \
  -D 'SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.rssi-meter.defaults' \
  build
```

## Flash

```sh
python tools/flash.py COM10 --build-dir build_rssi_meter
```

## Status

Prototype / experimental. The bench test plan (is the detector live,
pre- vs post-amplifier measurement, distance response) is in
`docs/rssi-meter.md`; results will land here as they are taken.

For the full project — the video receiver, Direct Gain, range work, and all
upstream documentation — see the original:
[Twotoz/C5VRX](https://github.com/Twotoz/C5VRX).

# RSSI Meter (standalone, ~1 kHz, no video)

Race-timer proximity receiver. Replaces the analog RSSI pin of a classic VRX:
the ESP32-C5 tunes to a fixed VTX frequency and continuously reports how much
RF power is in the channel. The video signal is deliberately sacrificed —
this is a separate firmware image, flash the normal firmware again for picture.

## What the image is

Follows the standalone capture-only meter pattern (see
`native-agc-v2.md`, "Capture memory failure and standalone meter"):

- Boots the **same proven receive-only RF state** via `rf_start()`:
  receive-only lock, PM off, BW40, promiscuous, continuous-modem ungate,
  ARC vendor-state capture, PLL tracking disabled.
- Excludes the whole video pipeline: PARLIO RX DMA, BitScrambler (no `.bsasm`
  programs are assembled into this image), demodulators, Direct Gain, menu,
  video output.
- Reserves the 64 KiB MAC dump SRAM banks before heap init, because the
  background continuous-modem dump writer keeps streaming into them
  (same reservation + ownership sanitize as the capture-only meter).
- `main/rssi_meter_main.c` is the only application entry; `rf.c` + `arc_phy.c`
  are shared unchanged with the normal image (except the new
  `rf_set_frequency_mhz()` used by both).

## Build

```sh
. ~/esp/esp-idf/export.sh
idf.py -B build_rssi_meter \
  -D SDKCONFIG=sdkconfig.rssi-meter \
  -D 'SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.rssi-meter.defaults' \
  build
```

## Flash (Windows example)

```sh
python tools/flash.py COM10 --build-dir build_rssi_meter
```

Restore video afterwards: build the normal image and flash the same way
without `--build-dir`. NVS is shared: `N` (AGC mode) persists across both
images.

## USB protocol

Console is USB CDC (`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`). One line per
sample at ~1 kHz (FreeRTOS 1 ms tick):

```
R:-72 NF:-95 G:52 M:0
```

| field | meaning |
|---|---|
| `R` | wideband RSSI in dBm (total in-channel power, signal + noise + adjacent) |
| `NF` | PHY noise-floor estimate in dBm, `-127` if invalid |
| `G` | forced gain index (valid only when `M:0`), `-1` in native AGC mode |
| `M` | `0` = firmware forced gain, `1` = native hardware AGC |

`-127` in `R` never reaches the wire (the line is suppressed); `-127` in
`NF` means the noise-floor detector had no valid value.

### 6-bit analog DAC output

The same D4..D9 resistor network as the video output
(8.2k/3.9k/2k/1k/470R/240R + 200R), same pin/bit order, static GPIO writes
(no GDMA at 1 kHz). RSSI dBm is mapped onto 0..63 and clamped:

- `RSSI_DAC_DBM_LO = -100`, `RSSI_DAC_DBM_HI = -40` (tune after the bench
  session; raw dBm is always on USB so the mapping can be adjusted from
  recorded data).
- `V` inverts polarity at runtime.

## Commands

| key | action |
|---|---|
| `H` | help |
| `T` | status line: `ST f:<mhz> mode:<forced\|native_agc> ...` |
| `N` | toggle native hardware AGC, persist to NVS, reboot |
| `S` | fixed-gain sweep G15/31/47/63/79/81 (forced mode only), prints `SWEEP G:<g> R:<rssi> NF:<nf>` per gain, restores the previous gain |
| `F<mhz>` | retune to an arbitrary MHz in the C5 5 GHz window (5180–5885), e.g. `F5800` |
| `V` | invert DAC polarity |
| `L` | pause / resume the 1 kHz stream |

## Bench test plan

### 1. Register liveness (VTX on/off, close range)

Monitor USB at 1 kHz (any serial monitor works; CDC is not limited by the
virtual baud rate). VTX on the same channel, 1 m away:

- `R` tracks the VTX: clearly above `NF` (expect >10 dB above the noise
  floor at close range).
- With the VTX off, `R` settles near `NF` and stops moving.
- If `R` is frozen at one value regardless of the VTX, the detector is not
  updating in continuous receive — report that, the meter is then useless
  for this purpose without a different strength source.

### 2. Pre-gain vs post-gain (`S`)

With the VTX on, send `S` in forced mode. The six `SWEEP` lines answer:

- **RSSI ≈ constant across G15..G81 while the video front-end would be
  clipping at the high gains** → RSSI is taken **pre-gain** (antenna side).
  Ideal for a timer: it works in any AGC mode and near saturation it simply
  reads "fully close".
- **RSSI climbs with the gain index** → RSSI is **post-gain**. Then the
  forced-gain mode is the meter mode of choice (stable firmware-held gain ⇒
  monotonic RSSI vs distance; the gain index `G` is on the line for
  linearization), and/or apply smoothing on the timer side.

Record the whole sweep for both a near and a far VTX position.

### 3. Distance response

Walk the VTX away (or move the meter). RSSI should fall monotonically on a
tens-to-hundreds-of-ms timescale. 1 kHz is far more than a race car needs;
if the timer wants less, decimate on its side. A `R - NF` difference above a
fixed threshold is the "drone in range" signal.

## Caveats

- Wideband RSSI is total in-channel power, not carrier-only — adjacent
  VTXs on the same channel also register. That is what a race timer wants.
- In native AGC mode the C5 re-acquires RX gain every ~25–50 µs
  (`native-agc-v2.md`); if step 2 shows post-gain behavior, native-AGC reads
  will bounce — use forced mode or smooth on the timer side.
- Near-range front-end saturation is the intended "close enough" behavior.
- This image produces no video and no IQ. Do not treat its measurements as
  evidence about the video pipeline.

## Frequency control (planned)

`rf_set_frequency_mhz()` (serial `F<mhz>` today) is the entry point for the
planned SPI-controlled frequency: the timer MCU will call the same function
over SPI instead of USB characters. It bootstraps on the nearest public
Wi-Fi center and applies the `phy_set_freq()` delta, then re-asserts gain/BW
ownership exactly like the proven FPV retune path.

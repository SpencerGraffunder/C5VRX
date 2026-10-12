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
- `main/rssi_meter_main.c` is the application entry; `rf.c` + `arc_phy.c`
  are shared with the normal image (the `rf_set_frequency_mhz()`,
  `rf_get_channel_at()` and `rf_find_channel_by_freq()` helpers are used by
  both).
- `main/rssi_signal.c` is a small, self-contained signal stage written for
  this project: EMA smoothing, dBm to 0..255 calibration, a learned quiet
  baseline with a signal-present decision, and a freshness rule (see
  "Signal-present detection"). It has no hardware includes and is unit
  tested on the host.

## Build

```bash
# Mac dev loop (build + flash + stream verification, one command):
tools/flash_rssi_meter.sh /dev/cu.usbmodemXXXX [--no-build] [--manual]
```

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

### Flash size and merged images

The C5 DevKit has **4 MB** flash; the XIAO board has 8 MB. The image header
declares a flash size, and boot fails when the chip's flash is **smaller**
than the header (`Detected size smaller than the size in the binary image
header`). So build the meter for 4 MB (`CONFIG_ESPTOOLPY_FLASHSIZE_4MB` in
`sdkconfig.rssi-meter-4mb.defaults`) — a 4 MB-header image boots on both
4 MB and 8 MB boards. An 8 MB-header image only boots on the 8 MB board.

When creating a merged single-file image for the C5, the 2nd-stage
bootloader lives at **0x2000** (not 0x0): the ROM reads its header from
0x2000, so a merged image with the bootloader at 0x0 boots into `invalid
header` at 0x2000. Layout: `0x2000 bootloader, 0x8000 partition table,
0x10000 app` (same as `tools/flash.py`).

Host unit tests (no board needed) cover the signal stage: calibration
mapping, baseline recovery from a hot seed, the frozen-register freshness
rule, the physical floor guard, retune invalidation and hysteresis:

```sh
cc -I main tools/test_rssi_meter.c main/rssi_signal.c -lm \
   -o /tmp/test_rssi_meter && /tmp/test_rssi_meter
```

## Signal chain

The raw RSSI is shaped before it reaches any output (`rssi_signal.c`):

```text
phy_get_rssi() @1 kHz
  -> EMA low-pass            (E command; off by default)
  -> calibration: dbLo -> 0, dbHi -> 255   (C command; defaults -100/-40)
```

The calibrated `0..255` value is what the analog output and the RotorHazard
node report (`P` on USB). The raw dBm stays on the wire as `R` for
diagnostics. With the default calibration the DAC output reproduces the
original dBm mapping exactly.

## Console protocol

The meter listens on **both** physical ports and answers on the one a command
came in on:

- **UART0** - the board's USB-UART bridge (GPIO 12 RX / GPIO 11 TX on the C5),
  or a plain serial lead from a lap timer. This is the primary console
  (`CONFIG_ESP_CONSOLE_UART_DEFAULT=y`).
- **USB-Serial/JTAG** - the board's USB socket CDC port
  (`CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG=y`), always installed as a
  command input by `meter_usbjtag_rx_init()`.

`T` prints `CONSOLE uart:ready usbjtag:ready reply_to:<port>` so the active
inputs and the reply target are visible. Replies are routed, not broadcast:
a RotorHazard server reads exactly payload+checksum from the port it wrote to,
so protocol bytes must leave by that same port. Before this routing existed the
whole console silently lived on whichever port `CONFIG_ESP_CONSOLE` picked, and
typing on the other port looked dead.

One line per sample at ~1 kHz (FreeRTOS 1 ms tick):

```
R:-72 NF:-95 G:47 M:0 P:167 S:1
```

| field | meaning |
|---|---|
| `R` | wideband RSSI in dBm, raw (total in-channel power, signal + noise + adjacent) |
| `NF` | PHY noise-floor estimate in dBm, `-127` if invalid |
| `G` | forced gain index (valid only when `M:0`), `-1` in native AGC mode |
| `M` | `0` = firmware forced gain, `1` = native hardware AGC |
| `P` | calibrated strength value 0..255 (what the outputs and node mode use) |
| `S` | signal present: `1` when the smoothed reading is `margin` dB above the learned quiet baseline (see "Signal-present detection") |

`-127` in `R` never reaches the wire (the line is suppressed); `-127` in
`NF` means the noise-floor detector had no valid value.

### 6-bit analog DAC output

The same D4..D9 resistor network as the video output
(8.2k/3.9k/2k/1k/470R/240R + 200R), same pin/bit order, static GPIO writes
(no GDMA at 1 kHz). The calibrated 0..255 value is mapped onto 0..63:

- Defaults: `dbLo = -100 dBm -> 0`, `dbHi = -40 dBm -> 255` (i.e. code
  ≈ (dBm + 100) * 63 / 60, as before). Calibrate per site with `C lo hi`.
- `V` inverts polarity at runtime.
- `OUT n` fixes the level for wiring tests; `OUT off` restores the live
  value.
- Pin tension: the tested ladder is wired to GPIO 23/24/11/12/8/9 and the C5
  console UART defaults to GPIO 12 (RX) / GPIO 11 (TX), which are ladder bits
  3 and 2. A UART-console build therefore leaves those two bits to the console
  and the ladder outputs 4 of its 6 bits (`RSSI_METER dac partial: ...` at
  boot). Set `CONFIG_C5VRX_RSSI_DAC_OWN_UART_PINS=y` to give the ladder all six
  pins; the UART port then dies and the meter is reached through the USB-JTAG
  port only.

### Signal-present detection

The vendor noise-floor read is **invalid in forced-gain mode on this board**
(`NF:-127` on every sample), so the meter cannot use it to decide whether a
transmitter is there. It learns its own quiet baseline instead:

- `baseline` tracks the quiet level with asymmetric per-sample coefficients
  (fast fall 0.05 ≈ 200 ms time constant at 1 kHz, slow rise
  `baseline_alpha`, default 0.002 ≈ 500 ms): a switched-off transmitter must
  release "signal present" within a couple of hundred ms, while a slow rise
  keeps a distant drone from lifting the reference level into itself. The
  baseline is re-seeded from the first sample after boot, a scan or an `R`.
- Readings below `floor_db` (default -105 dBm) are physically impossible for a
  40 MHz channel (kTB floor ≈ -98 dBm). They mean "the detector is not
  measuring", not "very weak signal", and never read as valid.
- A reading that has not changed for `fresh_ms` (default 1000 ms) is not
  trusted: with the VTX off the register held one value for 60 s, with it on it
  changed constantly. Without this rule a stale high value left over from a
  finished pass would hold a lap timer's threshold open forever.
- `signal present` = smoothed ≥ baseline + margin (default 12 dB), clearing at
  margin − hysteresis (default 6 dB). Transitions print one `EVENT present` /
  `EVENT clear` line.

Commands: `A` shows the signal state, `A <db>` sets the margin, `R` forgets
the learned baseline and re-learns it (do this whenever the site or the VTX
changes).

## Commands

| key | action |
|---|---|
| `H` | help |
| `T` | status line: `ST f:<mhz> mode:... state:... cal:... ema:... sig:... base:... fresh:...` plus console/rx/node health |
| `N` | toggle native hardware AGC, persist to NVS, reboot |
| `S` | fixed-gain sweep G15/31/47/63/79/81 (forced mode only), prints `SWEEP G:<g> R:<rssi> NF:<nf>` per gain, restores the previous gain |
| `F<mhz>` / `F R4` | retune (5180–5885 MHz or a named FPV channel); also sets the boot frequency that `P` persists |
| `CH R4` | tune to a named channel (line command) |
| `SCAN [ms]` | measure all 48 FPV channels (deduped, in-window only), print per-channel dB, report the strongest, restore the previous frequency |
| `C` / `C lo hi` / `C lo` / `C hi` | show / set calibration; `lo`/`hi` take the current smoothed reading |
| `E alpha` | EMA smoothing, 0.01–1 (1 = off, default) |
| `A` / `A <db>` | show / set the signal-present margin over the learned baseline (default 12 dB) |
| `R` | forget the learned baseline and re-learn it from the next valid reading |
| `OUT n` / `OUT off` | fix the analog output at 0..255 (wiring test) / restore |
| `Q [sec]` | raw PHY accessor values (no range check), or a register scan that lists only the receiver registers whose value moved — toggle the transmitter while it runs to find a live strength source |
| `Z [sec]` | diagnostics: Wi-Fi frames seen, distinct RSSI values, noise-floor validity |
| `P` | save settings (calibration, EMA, signal-present stage, boot frequency, polarity, node mode) to NVS |
| `D` | restore default settings (in memory; `P` to keep) |
| `V` | invert DAC polarity |
| `L` | pause / resume the 1 kHz stream |
| `M` | RotorHazard USB node mode (persisted) |
| `B` | reboot the meter (recovers a wedged stream) |
| `X` | arm the external auto-download circuit (if fitted), then restart; the chip comes up in download mode waiting for esptool. Without the circuit this is a plain reboot |

Settings persist in NVS (`c5vrx/rssi_cfg`, config version 4): calibration,
EMA, signal-present stage, boot frequency, DAC polarity, node mode. `P`
writes them, `D` resets the in-memory copy. Older config blobs (other
versions) are rejected and defaults are used.

## Auto-download circuit (flash without buttons, no host-reset dependency)

Background: on the C5 DevKit the USB-Serial/JTAG connector carries two
control lines the chip interprets at reset — one asserts reset, the other
holds GPIO0 (BOOT, the "wait for firmware" strapping). esptool's automatic
ROM entry (`--before usb_reset`) depends on the host driving those lines
into the right state, and on some Macs the driver's line state is not
deterministic: the chip sometimes comes up in the app, sometimes in
download mode, and sometimes neither. A pure-software self-download does
not work on the C5 (the GPIO output driver does not hold the strapping pad
low across a reset — verified on hardware 2026-10-02).

The circuit lets the running meter hold BOOT low across its own restart,
then releases it before the flasher's finish reset:

```text
meter GPIO10 ──►|──┬──────── gate ──────── 2N7000 (N-MOSFET)
                1N4148  │                                      │
                (anode  ├── 10 µF cap (+)                      drain ──► GPIO0 (BOOT pin)
                  to    │  (-) to GND                           source ──► GND
                 GPIO10)└── 50 kΩ ──► GND        (optional: 1 MΩ gate→GND)
```

Parts: 1× 2N7000, 1× 10 µF electrolytic (mind polarity), 1× 1N4148, 1× 50 kΩ.

Sequence (firmware `X` does all of it):

1. GPIO10 high ~150 ms: charges the gate cap through the diode (MOSFET on,
   pulling BOOT to GND — overriding the board's pull-up).
2. GPIO10 low, `esp_restart()`: the chip resets while the MOSFET still holds
   BOOT low → the ROM samples download mode and waits for the flasher.
3. The cap discharges through 50 kΩ (≈1.5 s to MOSFET-off). esptool
   connects (`--before no_reset`), flashes, and finishes with
   `--after watchdog_reset` — several seconds later, MOSFET long off, BOOT
   pull-up wins → the new app boots and streams.

Resulting flash loop (tools/flash_rssi_meter.sh):
send `X` → `esptool --before no_reset --after watchdog_reset write_flash`
→ streaming. No buttons, no DTR/RTS, no host-mood dependency. The circuit
is inert when not armed (MOSFET off, diode blocks backfeed into GPIO10).

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

## Bench results (2026-10-02, 4 MB C5 DevKit, VTX 5800 MHz)

Step 1 (liveness) — PASS:

- 1 kHz sustained: 30,002 samples in 30 s, zero inter-sample gaps over 20 ms.
- Frequency dependence: tuned to the VTX (5800) vs an empty channel (5750)
  the median RSSI differs by ~34 dB; retuning back restores the signal. The
  meter measures the tuned channel only.

Step 2 (`S` sweep, forced mode) — RSSI is **post-gain**:

- The reading follows the firmware gain setting (it does not stay flat
  across G15..G81). Consequences:
  - The fixed operating gain is part of the calibration. At gain 52 (the
    C5VRX default) this front end's noise floor equals the clamped
    on-signal level (zero detection margin). At **gain 47** the signal
    reads ~34 dB above the empty-channel floor (signal ≈ -61 dBm, floor
    ≈ -95 dBm at the bench position) — that is the operating point
    (`METER_DEFAULT_GAIN`), with near-range clipping as the intended
    "close enough" behavior. Re-run `S` per site/VTX power.

Step 3 (distance): not yet walked (bench position was fixed). The 34 dB
on/off margin at gain 47 is the detection budget; a `R - floor` threshold
of ~10 dB over the empty-channel reading is comfortable.

Other findings:

- The `NF` field (PHY noise floor) is frequently invalid (-127) in this
  image; use the empty-channel RSSI as the floor reference instead.
- The 6-bit DAC mapping (-100..-40 dBm → 0..63) is unchanged; analog pin
  check with a multimeter is still open (the bench board's ladder).
- The 4 MB-built image also runs on the 8 MB XIAO board (the size check
  only fails when the chip is smaller than the image header), so one image
  serves both boards.

## Bench results (2026-10-03/04, 4 MB C5 DevKit, VTX 5800 MHz, gain 47)

Read-only session (no flashing, since any flash needs a chip reset the user
had to perform manually):

1. **1 kHz cadence is stable** — 656 lines in 4 s, no gaps.
2. **The reading is post-gain** (it follows the gain writes), so a held gain is
   required for a usable monotonic response.
3. **With a carrier the register updates continuously and discriminates
   frequency**: 5800 MHz read ≈ -53, 5810 ≈ -53 then dropped, 5820+ fell to
   garbage, 5865/5885 read ≈ -91 (the noise floor).
4. **With no carrier the register is stale, not quiet**: it held one value for
   60 s (59,996 samples, zero changes) and that value was -124 dBm, which is
   below the physical floor of a 40 MHz channel. This is why the signal stage
   has the floor guard and the freshness rule.
5. **Gain response with a carrier is not monotonic**: G15/31/47 all read -53,
   G63 read -21, G79/81 read -37/-39. So the gain index cannot be used as a
   linear dB staircase; hold one gain and calibrate the window per site.
6. **Close-range saturation is real**: at gain 47 with the VTX near, the
   reading pinned around -44..-53 regardless of frequency, i.e. no distance
   granularity at close range — acceptable for "close enough" race timing.
7. **`NF` is always -127 in forced-gain mode** on this board, which is what
   made the learned-baseline approach necessary.

## Bench results (2026-10-05/06, 4 MB C5 DevKit, VTX 5800 MHz) — amplifier A/B

The decisive session for the amplifier question. The transmitter was
power-cycled by hand while the meter recorded:

- **Forced gain (firmware-held amplifier): the RSSI register does not
  update.** 200,520 samples over 200 s at 1 kHz with the transmitter
  cycled several times: exactly one value (−54 dBm), zero changes. The
  number is a stale snapshot from the initial AGC acquisition, not a
  measurement. (The earlier "valid readings in forced mode" sessions were
  the brief window while the detector was still settling after boot or a
  retune.)
- **Native hardware AGC: the register updates continuously.** Values move
  sample to sample, respond to retuning (tuned vs 50 MHz away ≈ 7 dB
  separation) with ~0.6 dB jitter.
- Enabling the PHY's periodic PLL tracking timer changed nothing (the
  detector behavior is identical with it on), so that was not the cause.
- The raw noise-floor accessor returns impossible values (raw −367/−512,
  far outside −140..−20) in both modes: the learned-baseline approach is
  the only usable quiet reference on this board.
- The `! RX STALE` watchdog caught the frozen state in the live stream
  instead of letting it masquerade as data.

Consequences (supersede the 2026-10-02 "forced gain is the meter mode"
conclusion, which was drawn from a reading that was actually frozen):

- On this board the meter must run with the **native hardware AGC** for the
  RSSI register to be a real measurement, or a different strength source
  must be found (the `Q <sec>` register scan exists for exactly this: find a
  receiver register that tracks the signal while the amplifier is held
  fixed, which would let forced gain be used again).
- Until that is resolved, treat forced-mode RSSI as a startup snapshot, not
  a stream.

## Caveats

- Wideband RSSI is total in-channel power, not carrier-only — adjacent
  VTXs on the same channel also register. That is what a race timer wants.
- In native AGC mode the C5 re-acquires RX gain every ~25–50 µs
  (`native-agc-v2.md`); reads in that mode bounce at the sample level, which
  is what the EMA (`E`) is for.
- Near-range front-end saturation is the intended "close enough" behavior.
- This image produces no video and no IQ. Do not treat its measurements as
  evidence about the video pipeline.

## Frequency control

`rf_set_frequency_mhz()` is the single retune entry point, reachable over USB
(`F`/`CH`/`SCAN`) and internally at boot (`P`-persisted boot frequency). It
bootstraps on the nearest public Wi-Fi center and applies the
`phy_set_freq()` delta, then re-asserts gain/BW ownership exactly like the
proven FPV retune path. A future SPI-controlled variant would call the same
function instead of USB characters.

## RotorHazard USB node mode

The meter can act as a RotorHazard race-timer node on the same USB port, so the
server sees it as a normal receiver module instead of a console.

Implementation: `main/rh_node.c` / `main/rh_node.h`, driven from the meter's
1 kHz sample cadence. Command bytes and response layouts follow RotorHazard's
own `src/interface/RHInterface.py` and `src/interface/serial_node.py`; the
crossing / peak / nadir / lap semantics mirror the proven NuclearCounter
`TimingCore`.

Protocol invariants (these are why the console stream must stop in node mode):

- The port is strictly request/response: the server writes one command byte
  (plus payload for writes) and reads exactly `payload + checksum` bytes with a
  250 ms timeout. Checksum is the sum of the payload bytes only, not the
  command byte. Any extra text printed on this path is read as part of the
  reply and fails the checksum.
- API level reported is 35, which puts the server on the
  `READ_LAP_PASS_STATS` (8 bytes) + `READ_LAP_EXTREMUMS` (8 bytes) path.
- RSSI on the wire is the RotorHazard 0..255 strength scale, not dBm. The
  signal stage already produces that scale, so the node layer consumes its
  counts directly. The server rejects 0 and 255
  (`Node.is_valid_rssi`), so reported values are clamped to 1..254.
- An extremum enters history only when its run ends (direction flips), matching
  the reference node: the server sees one peak per pass, not one per sample.
- The lap timestamp is the peak moment, not the moment the pass ends, because
  the server derives lap time from `ms since lap`.

Serial keys: `M` toggles node mode (persisted in NVS). `M` is not a protocol
byte, so it is the escape hatch from node mode without reflashing. `T` prints
one status line (safe on a polled port: the server retries and flushes).

Note: node mode is the **boot default** (`cfg.node_mode = 1`), because the
timer polls the port on its own schedule; press `M` once to get the human
console. Any protocol command byte arriving on a console-mode port switches to
node mode automatically, so an idle meter will quietly flip into node mode if
the server starts polling — that is intended, but it silences the 1 kHz
stream, which is the first thing to check when the stream "disappears".

Host unit test (48 checks, byte-for-byte against the server's expectations):

```sh
cc -I main tools/test_rh_node.c main/rh_node.c -lm -o /tmp/test_rh_node && /tmp/test_rh_node
```

Hardware-verified on the C5 DevKit over the board's native USB cable: the
discovery replies, the 16-byte firmware text blocks, `READ_FREQUENCY` and the
crossing/lap frames all answered, and the node entered RotorHazard mode when a
transmitter was present.

## Receive-liveness diagnostics (`Z`, `Q`, `RX`, `! RX STALE`)

A perfect 1 kHz stream is **not** proof that the receiver is receiving. The
meter proves it separately.

`Z [sec]` answers the two questions a raw number cannot:

- **frames/s** - how many 802.11 frames the promiscuous callback saw. An analog
  FPV transmitter has no 802.11 preamble, so zero frames is normal for the VTX;
  it is **not** normal when tuned onto a real access point.
- **distinct PHY RSSI values** in the window. One distinct value means the
  register is latched, not measuring.

`Z` temporarily enables promiscuous mode and always restores the previous state
(an earlier version left it on permanently and measurably degraded the receive
path - that restore is now explicit).

`Q` prints the **unvalidated** raw accessor values (the range check that turns
them into `-127` hides exactly the evidence a bring-up session needs), and
`Q <sec>` walks the whole PHY register block repeatedly, reporting only the
registers whose value moved with first/low/high bounds. Toggle the
transmitter while it runs: a register that tracks the signal is a usable
strength source whether or not the vendor accessor exposes it.

The output task runs a watchdog: if the raw RSSI holds one value for more than
5 s it prints

```text
! RX STALE raw RSSI held at -82 dBm for 45011 ms (noise floor invalid 25647/25647)
  - the PHY detector is not updating, so these readings are not measurements
```

and `T` carries `RX raw:<v> nf_invalid:<bad>/<total> phy_raw rssi:<v> nf:<v>`
so a host or lap timer can see the health of the measurement, not just its
value. A lap timer fed by a frozen RSSI fires on every lap or none, so this
state must never look like data.

### Flashing bug found during the liveness work: `tools/flash.py` re-stamped the bootloader to 8MB

`tools/flash.py` passed a hardcoded `--flash-size 8MB` to esptool. esptool uses
that value to stamp the image header it writes, so a correctly built 4MB
bootloader was rewritten as an 8MB-stamped one. On the 4MB DevKit the app then
asserts at boot:

```text
E spi_flash: Detected size(4096k) smaller than the size in the binary image header(8192k). Probe failed.
assert failed: __esp_system_init_fn_init_flash esp_flash_spi_init.c:654 (flash_ret == ESP_OK)
```

and the board crash-loops until the USB cable is unplugged. `tools/flash.py`
now reads `CONFIG_ESPTOOLPY_FLASHSIZE` out of the build's own
`config/sdkconfig.h` and falls back to `--flash-size keep`, so the build
decides the stamp.

Verified on hardware: writing the 4MB-stamped bootloader directly left
`0x2000` holding header flag byte `0x2f` (4MB) instead of `0x3f` (8MB), and
the firmware booted and locked a carrier.

/*
 * rssi_meter_main.c - Standalone ~1 kHz RSSI meter. No video path.
 *
 * Follows the standalone capture-only meter pattern (docs/native-agc-v2.md,
 * "Capture memory failure and standalone meter"): a separate app_main that
 * boots the proven receive-only RF state via rf_start(), excludes the whole
 * video pipeline (PARLIO RX DMA, BitScrambler, demodulators, menu, video
 * output), and reserves the MAC dump SRAM banks before heap initialization
 * because rf_start() arms the continuous-modem dump writer, which keeps
 * streaming into those banks in the background.
 *
 * Purpose: drone race timer proximity sensing. The receiver reports how
 * much RF power is in the tuned channel, the same job as the analog RSSI
 * pin on a classic VRX (or the RX5808 module in an FPV lap timer). Video
 * is deliberately sacrificed; flash the normal firmware again for video.
 *
 * Signal chain (see rssi_pipeline.h for rationale and attribution):
 *   phy_get_rssi() (raw dBm, 1 kHz)
 *     -> 30 ms peak-hold window (hides the ~25 ms AGC refresh dips)
 *     -> median-of-3 (drops single-sample glitches)
 *     -> optional EMA low-pass
 *     -> optional soft knee (saturating-receiver ceiling)
 *     -> calibration: dbLo -> 0, dbHi -> 255
 *   The calibrated 0..255 value drives the analog outputs and the P:
 *   stream field; the raw dBm stays on the line as R: for diagnostics.
 *
 * Outputs:
 *   USB CDC, ~1 kHz line per sample:
 *       R:<rssi_dBm> NF:<noise_floor_dBm> G:<gain_idx> M:<0|1> P:<0..255>
 *     M:0 = firmware forced gain (G valid), M:1 = native hardware AGC
 *     (G = -1, gain is hardware-owned). -127 means the PHY value was
 *     invalid/stale for that field. P is the calibrated strength value.
 *   6-bit analog DAC: the same D4..D9 resistor network as the video output,
 *     driven by static GPIO writes (no GDMA). The calibrated 0..255 value
 *     is mapped onto 0..63; V inverts the polarity.
 *   Optional sigma-delta output (rssi_sdm): 4 MHz 1-bit switching on one
 *     GPIO into an RC filter, ~8-bit resolution. Off unless configured.
 *   Optional RX5808 3-wire bus (rx5808_bus): SEL/CLK/DATA, makes the meter
 *     a drop-in replacement for the RX5808 module in an FPV lap timer
 *     (FPVGate-compatible). The timer tunes the receiver by writing the
 *     synthesizer register and reads RSSI on the analog pin or digitally
 *     from registers 0x6/0x7.
 *
 * Commands:
 *   H      help
 *   T      one status line (frequency, mode, calibration, bus state)
 *   N      toggle native hardware AGC, persist to NVS, reboot
 *   S      fixed-gain sweep G15..G81 (forced mode only): the pre-gain /
 *          post-gain RSSI oracle. Restores the previous gain.
 *   F<mhz> retune to a frequency inside the C5 5 GHz window, e.g. F5800.
 *          F R4 / F F4 also accepts a named FPV channel (snaps within 2 MHz).
 *   SCAN [dwell_ms]  measure all 48 FPV channels, report the strongest,
 *          restore the previous frequency.
 *   CH <band><n>     tune to a named channel (CH R4). Both F and CH update
 *          the boot frequency that P persists.
 *   C               show the calibration (dbLo, dbHi)
 *   C <lo> <hi>     set the dB levels that read as 0 and 255
 *   C lo | C hi     use the current smoothed reading as the low / high end
 *   K <db> <r> | K off   soft knee: squeeze signals above <db>
 *   E <alpha>       EMA smoothing, 0.01 to 1 (1 = off)
 *   W <ms>          peak-hold window in ms (0 = off)
 *   OUT <0..255> | OUT off   fix the analog output level (wiring test)
 *   U               RX5808 bus statistics (frames, last write/read)
 *   P               save the settings to NVS (boot frequency included)
 *   D               restore default settings (in memory, not saved)
 *   V      invert DAC polarity
 *   L      pause / resume the 1 kHz stream
 *   B      reboot the meter (clean esp_restart; recovers a wedged stream)
 *   X      arm the external auto-download circuit and restart. The circuit
 *          (see "Auto-download circuit" in docs/rssi-meter.md) is a
 *          diode + 10 uF cap + 2N7000 MOSFET + 50k resistor that GPIO10
 *          arms: the MOSFET then holds GPIO0 (BOOT) low ACROSS the restart,
 *          so the ROM samples download mode and waits for esptool. The RC
 *          discharges in ~1.5 s, so the NEXT reset (esptool's watchdog
 *          finish, seconds later) sees the BOOT pull-up high and boots the
 *          new app: no buttons, no host DTR/RTS dependency.
 *          Without the hardware fitted, X degrades to a plain reboot.
 *
 * Settings persist in NVS (namespace c5vrx, key rssi_cfg): calibration,
 * EMA, knee, window, boot frequency, DAC polarity. P saves, D resets the
 * in-memory copy to defaults.
 *
 * Scheduling: the 1 kHz output task (core 1) reads RSSI, shapes it through
 * the pipeline, drives the analog outputs and refreshes the bus readback
 * registers. A 1 ms bus task (core 0) processes queued RX5808 frames, so
 * host tuning works even while the console blocks in getchar(). The RF
 * mutex serializes tune/power transitions between the bus task and the
 * console. The pipeline, RX5808 bus and sigma-delta output are ports of
 * the FPVGateC5RX project (CC BY-NC-SA 4.0).
 *
 * History (2026-10-02): a pure-software self-download (drive GPIO0 low from
 * the GPIO output latch + restart) was tried and proven NOT to work on the
 * C5: the pad is sampled high at reset, so the chip always came back in the
 * app. The host-side esptool usb_reset (DTR/RTS) path was proven flaky on
 * the dev Mac: the kernel's RTS state decides whether the BOOT pin is held
 * low at reset, and it is not deterministic. The external circuit above
 * removes both dependencies.
 *
 * Invariants honoured: in native AGC mode every firmware gain write stays
 * refused at rf_set_rx_gain(); the DAC pin order and network are unchanged;
 * USB is the only console consumer and the bus ISR runs on core 0 while the
 * 1 kHz output task is pinned to core 1.
 */
#include <ctype.h>
#include <math.h>
#include <strings.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "heap_memory_layout.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/gpio_ll.h"
#include "nvs_flash.h"
#include "rf.h"
#include "rssi_pipeline.h"
#include "rssi_sdm.h"
#include "rx5808_bus.h"

/* The allocation selector disconnects BOTH 64 KiB dump banks from the CPU.
 * Same proven reservation as the capture-only meter; the background
 * continuous-modem dump writer keeps streaming into them in this image. */
SOC_RESERVE_MEMORY_REGION(0x4082ffc0, 0x40850040, c5vrx_rssi_meter_ram);
extern char _bss_end;

/* Race timer default center; F<mhz> retunes at runtime. */
#define METER_BOOT_FREQ_MHZ  5800u

/* C5 5 GHz receive window (channel 36 = 5180 MHz .. 177 = 5885 MHz). */
#define METER_FREQ_MIN_MHZ   5180u
#define METER_FREQ_MAX_MHZ   5885u

/* Auto-download arm line (external circuit, docs/rssi-meter.md). GPIO10 is
 * free: not a DAC pin (23/24/11/12/8/9), not USB (18/19), not flash (20/21),
 * not a strapping pin (0/23/29/30/31). */
#define METER_DOWNLOAD_ARM_GPIO  GPIO_NUM_10

/* Meter operating gain. rf_start() leaves the C5VRX default (52) in place;
 * the 2026-10-02 bench (4 MB DevKit, VTX 5800 MHz, fixed position) showed
 * gain 52 sits in a high-gain stage whose noise floor (-60 dBm) equals the
 * clamped on-signal level, i.e. zero detection margin. At 47 and below the
 * signal reads 34 dB above the floor (signal ~-60, floor ~-94 dBm), which is
 * the race-timer operating point. Re-calibrate with the S sweep per site. */
#define METER_DEFAULT_GAIN   47u

/* Six-bit video DAC. Same pin order as the video path (byte bit i -> pin i,
 * PARLIO LSB-first): never change the order, the physical
 * 8.2k/3.9k/2k/1k/470R/240R + 200R network is wired to exactly these pins. */
static const gpio_num_t s_dac_gpio[6] = {23, 24, 11, 12, 8, 9};

/* RX5808 bus pins (Kconfig; defaults match FPVGate's XIAO-S3 wiring so the
 * C5 side is like-for-like). */
#define METER_BUS_SEL_PIN    ((gpio_num_t)CONFIG_C5VRX_RSSI_BUS_SEL_PIN)
#define METER_BUS_CLK_PIN    ((gpio_num_t)CONFIG_C5VRX_RSSI_BUS_CLK_PIN)
#define METER_BUS_DATA_PIN   ((gpio_num_t)CONFIG_C5VRX_RSSI_BUS_DATA_PIN)

/* Meter state, reported in the bus status register D11-14 and on T. */
typedef enum {
    METER_STATE_BOOT = 0,
    METER_STATE_IDLE = 1,
    METER_STATE_TUNING = 2,
    METER_STATE_TRACKING = 3,
    METER_STATE_POWERDOWN = 4,
    METER_STATE_FAULT = 5,
} meter_state_t;

/* Persisted settings (NVS c5vrx/rssi_cfg). */
#define METER_CFG_MAGIC    0x4335524Du /* "C5RM" */
#define METER_CFG_VERSION  1u
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t boot_mhz;      /* 0 = METER_BOOT_FREQ_MHZ */
    float db_lo;            /* calibrated 0 dBm end */
    float db_hi;            /* calibrated 255 dBm end */
    float ema_alpha;
    uint8_t use_median3;
    uint8_t dac_invert;
    uint16_t window_max_ms;
    float knee_db;
    float knee_ratio;
} meter_cfg_t;

static void meter_cfg_defaults(meter_cfg_t *cfg)
{
    cfg->magic = METER_CFG_MAGIC;
    cfg->version = METER_CFG_VERSION;
    cfg->boot_mhz = 0;
    cfg->db_lo = -100.0f;   /* matches the original DAC window */
    cfg->db_hi = -40.0f;
    cfg->ema_alpha = 1.0f;  /* off, like the reference default */
    cfg->use_median3 = 1;
    cfg->dac_invert = 0;
    cfg->window_max_ms = 30;
    cfg->knee_db = -55.0f;
    cfg->knee_ratio = 1.0f; /* off */
}

static bool meter_cfg_valid(const meter_cfg_t *cfg)
{
    if (cfg->magic != METER_CFG_MAGIC || cfg->version != METER_CFG_VERSION) {
        return false;
    }
    if (cfg->db_lo < -140.0f || cfg->db_lo > -10.0f ||
        cfg->db_hi < -100.0f || cfg->db_hi > 10.0f ||
        cfg->db_hi <= cfg->db_lo + 5.0f) {
        return false;
    }
    if (cfg->ema_alpha <= 0.0f || cfg->ema_alpha > 1.0f) {
        return false;
    }
    if (cfg->window_max_ms > RSSI_PIPELINE_WIN_CAP) {
        return false;
    }
    if (cfg->knee_ratio < 1.0f) {
        return false;
    }
    if (cfg->boot_mhz != 0u &&
        (cfg->boot_mhz < METER_FREQ_MIN_MHZ || cfg->boot_mhz > METER_FREQ_MAX_MHZ)) {
        return false;
    }
    return true;
}

static void meter_cfg_load(meter_cfg_t *cfg)
{
    nvs_handle_t h;
    if (nvs_open("c5vrx", NVS_READONLY, &h) != ESP_OK) {
        return; /* keep defaults */
    }
    size_t len = sizeof(*cfg);
    esp_err_t err = nvs_get_blob(h, "rssi_cfg", cfg, &len);
    nvs_close(h);
    if (err != ESP_OK || len != sizeof(*cfg) || !meter_cfg_valid(cfg)) {
        meter_cfg_defaults(cfg);
    }
}

static bool meter_cfg_save(const meter_cfg_t *cfg)
{
    nvs_handle_t h;
    if (nvs_open("c5vrx", NVS_READWRITE, &h) != ESP_OK) {
        return false;
    }
    esp_err_t err = nvs_set_blob(h, "rssi_cfg", cfg, sizeof(*cfg));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err == ESP_OK;
}

static meter_cfg_t s_cfg;
static rssi_pipeline_t s_pipe;
static SemaphoreHandle_t s_rf_mutex; /* serializes tune / power transitions */
static volatile bool s_stream_paused;
static volatile bool s_dac_invert;
static volatile int s_meter_gain = 52; /* rf_start() forced-gain default */
static volatile int s_force_counts = -1; /* OUT command wiring test */
static volatile uint8_t s_state = METER_STATE_BOOT;
static volatile uint16_t s_tuned_mhz = 0;
static volatile bool s_freq_supported = true;
static volatile bool s_powered = true;

/* ---- RX5808 bus (ISR on core 0, consumers on core 0 main loop) ---- */
static rx5808_bus_t s_bus;
static volatile uint32_t s_bus_read_regs[16];
static volatile uint32_t s_bus_pending_word;
static volatile bool s_bus_pending_valid;
static volatile uint32_t s_bus_frames;
static volatile uint32_t s_bus_writes;
static volatile uint32_t s_bus_reads;
static volatile uint8_t s_bus_last_read_addr;
static volatile uint32_t s_bus_last_word;
static volatile bool s_bus_last_sel;
static volatile bool s_bus_last_clk;

static void dac_write(uint8_t code)
{
    for (unsigned i = 0u; i < 6u; ++i) {
        gpio_set_level(s_dac_gpio[i], (code >> i) & 1u);
    }
}

/* Calibrated 0..255 strength -> 6-bit DAC code (0..63). With the default
 * calibration (-100..-40 dBm) this reproduces the original dBm mapping. */
static uint8_t counts_to_code(uint8_t counts)
{
    uint8_t code = (uint8_t)((uint16_t)counts * 63u / 255u);
    if (s_dac_invert) {
        code = 63u - code;
    }
    return code;
}

static void meter_output_task(void *arg)
{
    for (;;) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        int rssi = -127;
        int nf = -127;
        bool rssi_ok = false;
        bool nf_ok = false;

        if (!s_stream_paused && s_powered) {
            rssi_ok = rf_try_get_wideband_rssi_dbm(&rssi);
            nf_ok = rf_try_get_noise_floor_dbm(&nf);
            if (rssi_ok) {
                rssi_pipeline_on_sample(&s_pipe, (float)rssi, now_ms);
            }
        }
        rssi_pipeline_tick(&s_pipe, now_ms);

        /* TUNING -> TRACKING once the pipeline has a post-settle reading. */
        if (s_state == METER_STATE_TUNING && rssi_pipeline_valid(&s_pipe)) {
            s_state = METER_STATE_TRACKING;
        }

        uint8_t p = (s_force_counts >= 0) ? (uint8_t)s_force_counts
                                          : rssi_pipeline_counts(&s_pipe);
        dac_write(counts_to_code(p));
        rssi_sdm_set_counts(p);

        /* Bus readback registers, refreshed every tick so the interrupt
         * handler never has to touch the pipeline. */
        s_bus_read_regs[RX5808_REG_SYNTH_RF] =
            rx5808_mhz_to_synth_reg(s_tuned_mhz != 0 ? s_tuned_mhz : METER_BOOT_FREQ_MHZ);
        int smoothed = (int)lroundf(rssi_pipeline_smoothed_db(&s_pipe));
        if (smoothed < -128) smoothed = -128;
        if (smoothed > 127) smoothed = 127;
        s_bus_read_regs[RX5808_REG_EXT_INFO] = rx5808_build_info_word(smoothed);
        s_bus_read_regs[RX5808_REG_EXT_STATUS] = rx5808_build_status_word(
            p, rssi_pipeline_valid(&s_pipe), s_freq_supported, s_state);

        if (!s_stream_paused && rssi_ok) {
            bool native = rf_native_agc_active();
            printf("R:%d NF:%d G:%d M:%d P:%u\n",
                   rssi,
                   nf_ok ? nf : -127,
                   native ? -1 : s_meter_gain,
                   native ? 1 : 0,
                   (unsigned)p);
        }
        vTaskDelay(pdMS_TO_TICKS(1)); /* ~1 kHz; FreeRTOS tick = 1 ms */
    }
}

/* ---- RX5808 bus pin access (ISR context: direct register ops only) ---- */
static void bus_data_release(void)
{
    gpio_ll_output_disable(&GPIO, METER_BUS_DATA_PIN);
}

static void bus_data_drive(bool level)
{
    /* Set the level before enabling the driver, so the host never samples
     * a transition mid-bit. */
    gpio_ll_set_level(&GPIO, METER_BUS_DATA_PIN, level ? 1u : 0u);
    gpio_ll_output_enable(&GPIO, METER_BUS_DATA_PIN);
}

static bool bus_data_sample(void)
{
    return gpio_ll_get_level(&GPIO, METER_BUS_DATA_PIN) != 0;
}

static uint32_t bus_read_provider(uint8_t addr)
{
    return s_bus_read_regs[addr & 0x0Fu];
}

static void bus_frame_cb(uint32_t bits25, bool is_write, uint8_t addr)
{
    /* ISR context: queue the word, the bus task processes it. */
    s_bus_frames++;
    if (is_write) {
        s_bus_pending_word = bits25;
        s_bus_pending_valid = true;
        s_bus_writes++;
        s_bus_last_word = bits25;
    } else {
        s_bus_reads++;
        s_bus_last_read_addr = addr;
    }
}

static void IRAM_ATTR bus_isr(void *arg)
{
    (void)arg;
    /* One handler for both pins; check which one actually changed. */
    bool sel = gpio_ll_get_level(&GPIO, METER_BUS_SEL_PIN) != 0;
    bool clk = gpio_ll_get_level(&GPIO, METER_BUS_CLK_PIN) != 0;
    if (sel != s_bus_last_sel) {
        s_bus_last_sel = sel;
        rx5808_bus_sel_change(&s_bus, sel);
    }
    if (clk != s_bus_last_clk) {
        s_bus_last_clk = clk;
        rx5808_bus_clk_change(&s_bus, clk);
    }
}

static bool meter_bus_begin(void)
{
    uint64_t mask = (1ULL << (uint32_t)METER_BUS_SEL_PIN) |
                    (1ULL << (uint32_t)METER_BUS_CLK_PIN) |
                    (1ULL << (uint32_t)METER_BUS_DATA_PIN);
    gpio_config_t cfg = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&cfg) != ESP_OK) {
        return false;
    }
    /* SEL idles high (host pull-up mirrors the timer side); give it a
     * pull-up in case the timer's is weak. */
    gpio_set_pull_mode(METER_BUS_SEL_PIN, GPIO_PULLUP_ENABLE);
    /* DATA goes both ways. Configure it as an output so it is routed as a
     * plain GPIO, then switch the driver off. The ISR only turns the driver
     * on and off after that. */
    gpio_config_t data_cfg = {
        .pin_bit_mask = 1ULL << (uint32_t)METER_BUS_DATA_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_set_level(METER_BUS_DATA_PIN, 0);
    if (gpio_config(&data_cfg) != ESP_OK) {
        return false;
    }
    gpio_ll_input_enable(&GPIO, METER_BUS_DATA_PIN);
    gpio_ll_output_disable(&GPIO, METER_BUS_DATA_PIN);

    if (gpio_install_isr_service(0) != ESP_OK &&
        gpio_install_isr_service(0) != ESP_ERR_INVALID_STATE) {
        return false;
    }
    gpio_isr_handler_add(METER_BUS_SEL_PIN, bus_isr, NULL);
    gpio_isr_handler_add(METER_BUS_CLK_PIN, bus_isr, NULL);
    if (gpio_set_intr_type(METER_BUS_SEL_PIN, GPIO_INTR_NEGEDGE | GPIO_INTR_POSEDGE) != ESP_OK ||
        gpio_set_intr_type(METER_BUS_CLK_PIN, GPIO_INTR_NEGEDGE | GPIO_INTR_POSEDGE) != ESP_OK) {
        return false;
    }

    s_bus.read_provider = bus_read_provider;
    s_bus.frame_cb = bus_frame_cb;
    s_bus.data_release = bus_data_release;
    s_bus.data_drive = bus_data_drive;
    s_bus.data_sample = bus_data_sample;
    s_bus.sel_level = NULL;
    s_bus.clk_level = NULL;
    s_bus_last_sel = gpio_ll_get_level(&GPIO, METER_BUS_SEL_PIN) != 0;
    s_bus_last_clk = gpio_ll_get_level(&GPIO, METER_BUS_CLK_PIN) != 0;
    return true;
}

/* ---- Tuning / power (bus task and console; serialized by s_rf_mutex) ---- */
static esp_err_t meter_tune(uint16_t mhz, uint32_t now_ms)
{
    s_tuned_mhz = mhz;
    s_freq_supported = (mhz >= METER_FREQ_MIN_MHZ && mhz <= METER_FREQ_MAX_MHZ);
    xSemaphoreTake(s_rf_mutex, portMAX_DELAY);
    if (!s_powered) {
        esp_wifi_set_promiscuous(true);
        s_powered = true;
    }
    esp_err_t err;
    if (!s_freq_supported) {
        /* A channel the C5 can't receive: accept the command, so the
         * read-back still matches, but report no signal. */
        err = ESP_ERR_NOT_SUPPORTED;
    } else {
        err = rf_set_frequency_mhz(mhz);
    }
    xSemaphoreGive(s_rf_mutex);
    rssi_pipeline_on_tune(&s_pipe, now_ms);
    s_state = (err == ESP_OK) ? METER_STATE_TUNING
            : (err == ESP_ERR_NOT_SUPPORTED) ? METER_STATE_TRACKING
                                              : METER_STATE_FAULT;
    return err;
}

static void bus_handle_word(uint32_t bits25)
{
    rx5808_word_t w = rx5808_parse_word(bits25);
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

    switch (w.address) {
    case RX5808_REG_SYNTH_RF: {
        if (!w.write) {
            return;
        }
        uint16_t mhz = rx5808_synth_reg_to_mhz((uint16_t)(w.data & 0xFFFFu));
        const fpv_channel_t *ch = rf_find_channel_by_freq(mhz, 2);
        if (ch != NULL) {
            mhz = ch->freq_mhz; /* snap, like the reference normaliseFrequency */
        }
        meter_tune(mhz, now_ms);
        break;
    }
    case RX5808_REG_POWER:
        if (!w.write) {
            return;
        }
        if ((w.data & 0xFFFFFu) == 0xFFFFFu) {
            /* FPV-style timers power the RX5808 down by writing all ones. */
            xSemaphoreTake(s_rf_mutex, portMAX_DELAY);
            esp_wifi_set_promiscuous(false);
            s_powered = false;
            xSemaphoreGive(s_rf_mutex);
            s_state = METER_STATE_POWERDOWN;
        } else if (!s_powered) {
            meter_tune(s_tuned_mhz ? s_tuned_mhz : METER_BOOT_FREQ_MHZ, now_ms);
        }
        break;
    case RX5808_REG_STATE:
        if (!w.write) {
            return;
        }
        /* Reset: re-assert the meter gain and retune to the last frequency.
         * meter_tune takes the RF mutex itself. */
        rf_set_rx_gain(true, (uint8_t)s_meter_gain);
        meter_tune(s_tuned_mhz ? s_tuned_mhz : METER_BOOT_FREQ_MHZ, now_ms);
        break;
    default:
        /* Other registers need no action. */
        break;
    }
}

/* Runs on a dedicated 1 ms task so host tuning works even while the
 * console is blocked in getchar(). */
static void meter_bus_task(void *arg)
{
    (void)arg;
    portMUX_TYPE bus_mux;
    for (;;) {
        if (s_bus_pending_valid) {
            portENTER_CRITICAL(&bus_mux);
            uint32_t w = s_bus_pending_word;
            s_bus_pending_valid = false;
            portEXIT_CRITICAL(&bus_mux);
            bus_handle_word(w);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/* ---- Console ---- */
static void meter_help(void)
{
    printf("RSSI_METER commands:\n"
           " H   this help\n"
           " T   status line (includes bus stats)\n"
           " N   toggle native HW AGC (persist, reboot)\n"
           " S   fixed-gain sweep G15..G81 (forced mode only, pre/post-gain oracle)\n"
           " F<mhz>|F R4  retune, 5180-5885, or a named FPV channel\n"
           " SCAN [ms]   measure all 48 FPV channels, report strongest\n"
           " CH R4       tune to a named channel\n"
           " C [lo hi]   show / set calibration dB (or 'C lo' / 'C hi')\n"
           " K db r|off  soft knee | E alpha EMA | W ms peak-hold window\n"
           " OUT n|off   fix analog output 0..255 (wiring test)\n"
           " U   RX5808 bus statistics\n"
           " P   save settings to NVS | D defaults (not saved)\n"
           " V   invert DAC polarity\n"
           " L   pause/resume 1 kHz stream\n"
           " B   reboot the meter\n"
           " X   arm external auto-download circuit, then restart (for flashing)\n");
}

static const char *state_name(uint8_t s)
{
    switch (s) {
    case METER_STATE_BOOT: return "BOOT";
    case METER_STATE_IDLE: return "IDLE";
    case METER_STATE_TUNING: return "TUNING";
    case METER_STATE_TRACKING: return "TRACKING";
    case METER_STATE_POWERDOWN: return "POWERDOWN";
    case METER_STATE_FAULT: return "FAULT";
    }
    return "?";
}

static void meter_status(void)
{
    const fpv_channel_t *ch = rf_find_channel_by_freq(s_tuned_mhz, 0);
    printf("ST f:%u%s mode:%s state:%s cal:%.0f..%.0f dB ema:%.2f win:%u ms "
           "knee:%.0f/%.1f dac_pol:%s paused:%d gain:%d\n",
           s_tuned_mhz, ch ? ch->name : "",
           rf_native_agc_active() ? "native_agc" : "forced",
           state_name(s_state),
           s_pipe.cfg.db_lo, s_pipe.cfg.db_hi,
           s_pipe.cfg.ema_alpha,
           (unsigned)s_pipe.cfg.window_max_ms,
           s_pipe.cfg.knee_db, s_pipe.cfg.knee_ratio,
           s_dac_invert ? "inverted" : "normal",
           s_stream_paused ? 1 : 0,
           s_meter_gain);
#ifdef CONFIG_C5VRX_RSSI_BUS
    printf("BUS sel:%d clk:%d data:%d frames:%lu writes:%lu reads:%u last:0x%05lX read@0x%X\n",
           (int)METER_BUS_SEL_PIN, (int)METER_BUS_CLK_PIN, (int)METER_BUS_DATA_PIN,
           (unsigned long)s_bus_frames, (unsigned long)s_bus_writes,
           (unsigned)s_bus_reads, (unsigned long)s_bus_last_word,
           (unsigned)s_bus_last_read_addr);
#endif
}

static void meter_gain_sweep(void)
{
    if (rf_native_agc_active()) {
        printf("SWEEP REFUSED reason=native_agc_active gain_writes_must_stay_refused\n");
        return;
    }
    /* Same spread as the video-firmware RSSI oracle (serial R). */
    static const uint8_t gains[] = {15u, 31u, 47u, 63u, 79u, 81u};
    const int restore = s_meter_gain;
    printf("SWEEP start f=%u restore_g=%d\n", s_tuned_mhz, restore);
    for (unsigned i = 0u; i < sizeof(gains) / sizeof(gains[0]); ++i) {
        rf_set_rx_gain(true, gains[i]);
        s_meter_gain = gains[i];
        vTaskDelay(pdMS_TO_TICKS(150));
        int rssi = -127;
        int nf = -127;
        bool rssi_ok = rf_try_get_wideband_rssi_dbm(&rssi);
        bool nf_ok = rf_try_get_noise_floor_dbm(&nf);
        printf("SWEEP G:%u R:%d NF:%d\n",
               (unsigned)gains[i], rssi_ok ? rssi : -127, nf_ok ? nf : -127);
    }
    rf_set_rx_gain(true, (uint8_t)restore);
    s_meter_gain = restore;
    vTaskDelay(pdMS_TO_TICKS(150));
    printf("SWEEP done restored G:%d\n", restore);
}

/* Read the rest of the current line (to newline/EOS) into buf. A
 * previously-read first character may be passed in (the generic fallback
 * path); pass -1 when the command character was already consumed by the
 * dispatcher and nothing else has been read. Returns the byte count. */
static int read_line(char *buf, int len, int first_char)
{
    int n = 0;
    int c = (first_char >= 0) ? first_char : getchar();
    while (c >= 0 && c != '\n' && c != '\r') {
        if (n < len - 1) {
            buf[n++] = (char)c;
        }
        c = getchar();
    }
    buf[n] = '\0';
    return n;
}

/* Split `s` (modified in place) on spaces into up to `max` tokens. */
static int tokenize(char *s, char **tok, int max)
{
    int n = 0;
    while (*s && n < max) {
        while (*s == ' ' || *s == '\t') {
            ++s;
        }
        if (!*s) {
            break;
        }
        tok[n++] = s;
        while (*s && *s != ' ' && *s != '\t') {
            ++s;
        }
        if (*s) {
            *s++ = '\0';
        }
    }
    return n;
}

static uint16_t parse_freq_token(const char *t)
{
    /* Named FPV channel (R4, F4, A1, ...) or plain MHz. */
    if (isalpha((unsigned char)t[0]) && isdigit((unsigned char)t[1]) && !t[2]) {
        for (size_t i = 0u; i < rf_get_channel_count(); ++i) {
            const fpv_channel_t *ch = rf_get_channel_at(i);
            if (ch && toupper((unsigned char)t[0]) == toupper((unsigned char)ch->name[0]) &&
                t[1] == ch->name[1]) {
                return ch->freq_mhz;
            }
        }
        return 0;
    }
    char *end = NULL;
    unsigned long mhz = strtoul(t, &end, 10);
    if (end == t || mhz < 1000ul || mhz > 9999ul) {
        return 0;
    }
    return (uint16_t)mhz;
}

/* Console tune: also updates the boot frequency that P persists. */
static void meter_set_frequency(uint16_t mhz)
{
    if (mhz == 0) {
        printf("FUSAGE F<mhz> or F<band><n>, e.g. F5800 / F R4\n");
        return;
    }
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    esp_err_t err = meter_tune(mhz, now_ms);
    s_cfg.boot_mhz = mhz;
    const fpv_channel_t *ch = rf_find_channel_by_freq(s_tuned_mhz, 0);
    printf("F:%u err=%s%s (P saves as boot frequency)\n",
           mhz, esp_err_to_name(err), ch ? ch->name : "");
}

static void meter_scan(uint32_t dwell_ms)
{
    if (dwell_ms == 0) {
        dwell_ms = 50;
    }
    uint16_t restore = s_tuned_mhz;
    uint16_t peak_mhz = 0;
    float peak_db = -200.0f;
    printf("SCAN start dwell=%u ms\n", (unsigned)dwell_ms);
    for (size_t i = 0u; i < rf_get_channel_count(); ++i) {
        const fpv_channel_t *ch = rf_get_channel_at(i);
        if (ch == NULL) {
            continue;
        }
        /* Skip duplicate MHz (R7/F8 = 5880) and out-of-window channels. */
        bool dup = false;
        for (size_t j = 0u; j < i; ++j) {
            const fpv_channel_t *prev = rf_get_channel_at(j);
            if (prev && prev->freq_mhz == ch->freq_mhz) {
                dup = true;
                break;
            }
        }
        if (dup || ch->freq_mhz < METER_FREQ_MIN_MHZ ||
            ch->freq_mhz > METER_FREQ_MAX_MHZ) {
            continue;
        }
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        meter_tune(ch->freq_mhz, now_ms);
        vTaskDelay(pdMS_TO_TICKS(s_pipe.cfg.settle_ms + dwell_ms));
        /* Peak over the dwell (the pipeline already peak-holds). */
        float db = rssi_pipeline_smoothed_db(&s_pipe);
        printf("SCAN %s %u dB:%.1f\n", ch->name, ch->freq_mhz, db);
        if (db > peak_db) {
            peak_db = db;
            peak_mhz = ch->freq_mhz;
        }
    }
    if (restore != 0) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        meter_tune(restore, now_ms);
    }
    const fpv_channel_t *pch = peak_mhz ? rf_find_channel_by_freq(peak_mhz, 0) : NULL;
    printf("SCAN done peak=%s %u dB:%.1f\n",
           pch ? pch->name : "--", peak_mhz, peak_db);
}

static void meter_calibration(int argc, char **argv)
{
    if (argc == 0) {
        printf("CAL db_lo=%.1f db_hi=%.1f (0..255)\n",
               s_pipe.cfg.db_lo, s_pipe.cfg.db_hi);
        return;
    }
    if (argc == 1 && !strcasecmp(argv[0], "lo")) {
        float v = rssi_pipeline_smoothed_db(&s_pipe);
        s_pipe.cfg.db_lo = v;
        s_cfg.db_lo = v;
        printf("CAL db_lo=%.1f (from current reading)\n", v);
        return;
    }
    if (argc == 1 && !strcasecmp(argv[0], "hi")) {
        float v = rssi_pipeline_smoothed_db(&s_pipe);
        s_pipe.cfg.db_hi = v;
        s_cfg.db_hi = v;
        printf("CAL db_hi=%.1f (from current reading)\n", v);
        return;
    }
    if (argc == 2) {
        float lo = strtof(argv[0], NULL);
        float hi = strtof(argv[1], NULL);
        if (hi > lo + 5.0f) {
            rssi_pipeline_set_calibration(&s_pipe, lo, hi);
            s_cfg.db_lo = lo;
            s_cfg.db_hi = hi;
            printf("CAL db_lo=%.1f db_hi=%.1f\n", lo, hi);
        } else {
            printf("CALUSAGE hi must be > lo + 5 dB\n");
        }
    } else {
        printf("CALUSAGE C [lo hi] | C lo | C hi\n");
    }
}

static void meter_apply_cfg_to_pipeline(void)
{
    rssi_pipeline_set_calibration(&s_pipe, s_cfg.db_lo, s_cfg.db_hi);
    rssi_pipeline_set_ema_alpha(&s_pipe, s_cfg.ema_alpha);
    rssi_pipeline_set_knee(&s_pipe, s_cfg.knee_db, s_cfg.knee_ratio);
    s_pipe.cfg.use_median3 = s_cfg.use_median3 != 0;
    s_pipe.cfg.window_max_ms = s_cfg.window_max_ms;
    s_dac_invert = s_cfg.dac_invert != 0;
}

static void meter_defaults(void)
{
    meter_cfg_t d;
    meter_cfg_defaults(&d);
    s_cfg = d;
    meter_apply_cfg_to_pipeline();
    printf("DEFAULTS applied (P saves, B+P keeps)\n");
}

void app_main(void)
{
    if ((uintptr_t)&_bss_end > 0x4082ffc0u) {
        printf("RSSI_METER ERROR bss_overlaps_dump_banks\n");
        return;
    }

    /* CPU-only resets can leave dump ownership latched. Sanitize before PHY,
     * same sequence as the capture-only meter. */
    volatile uint32_t *ctrl = (volatile uint32_t *)0x600a9004u;
    volatile uint32_t *usage = (volatile uint32_t *)0x60095004u;
    *ctrl &= ~(0x80000000u | 0x00080000u | 0x00040000u | 0x00020000u);
    *usage &= ~0x00010f00u;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");

    esp_err_t err = rf_start();
    if (err != ESP_OK) {
        printf("RSSI_METER ERROR rf_start=%s\n", esp_err_to_name(err));
        return;
    }

    /* Load persisted settings before anything else user-visible. */
    meter_cfg_load(&s_cfg);
    s_dac_invert = s_cfg.dac_invert != 0;

    /* Static DAC pins: outputs, code 0 until the first sample. */
    gpio_config_t dac_cfg = {
        .pin_bit_mask = 0ULL,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    for (unsigned i = 0u; i < 6u; ++i) {
        dac_cfg.pin_bit_mask |= 1ULL << (uint32_t)s_dac_gpio[i];
        gpio_set_level(s_dac_gpio[i], 0);
    }
    if ((err = gpio_config(&dac_cfg)) != ESP_OK) {
        printf("RSSI_METER ERROR dac_gpio=%s\n", esp_err_to_name(err));
        return;
    }

    /* Pipeline (calibrated shaping of the raw RSSI). */
    rssi_pipeline_cfg_t pcfg = {
        .db_lo = s_cfg.db_lo,
        .db_hi = s_cfg.db_hi,
        .ema_alpha = s_cfg.ema_alpha,
        .use_median3 = s_cfg.use_median3 != 0,
        .window_max_ms = s_cfg.window_max_ms,
        .knee_db = s_cfg.knee_db,
        .knee_ratio = s_cfg.knee_ratio,
        .settle_ms = 35,
        .stall_ms = 200,
    };
    rssi_pipeline_begin(&s_pipe, &pcfg);

    /* Optional sigma-delta analog output. */
    bool sdm_on = rssi_sdm_begin(CONFIG_C5VRX_RSSI_SDM_PIN,
                                 CONFIG_C5VRX_RSSI_SDM_DIVIDER_M1000);
    if (CONFIG_C5VRX_RSSI_SDM_PIN >= 0 && !sdm_on) {
        printf("RSSI_METER WARN sdm_begin failed (pin %d)\n",
               CONFIG_C5VRX_RSSI_SDM_PIN);
    }

    /* RX5808 3-wire bus. The frame-processing task must exist before the
     * ISR can queue words. */
    s_rf_mutex = xSemaphoreCreateMutex();
#ifdef CONFIG_C5VRX_RSSI_BUS
    if (meter_bus_begin() &&
        xTaskCreatePinnedToCore(meter_bus_task, "rssi_bus", 4096, NULL, 7,
                                NULL, 0) == pdPASS) {
        printf("RSSI_METER bus ready (SEL %d CLK %d DATA %d)\n",
               (int)METER_BUS_SEL_PIN, (int)METER_BUS_CLK_PIN,
               (int)METER_BUS_DATA_PIN);
    } else {
        printf("RSSI_METER WARN bus_begin failed (SEL %d CLK %d DATA %d)\n",
               (int)METER_BUS_SEL_PIN, (int)METER_BUS_CLK_PIN,
               (int)METER_BUS_DATA_PIN);
    }
#endif

    /* Meter operating gain (forced mode only; in native AGC mode the
     * rf_set_rx_gain() write is refused and the firmware-held default stays). */
    rf_set_rx_gain(true, METER_DEFAULT_GAIN);
    s_meter_gain = METER_DEFAULT_GAIN;

    uint16_t boot_mhz = s_cfg.boot_mhz ? s_cfg.boot_mhz : METER_BOOT_FREQ_MHZ;
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    meter_tune(boot_mhz, now_ms);
    if (s_state == METER_STATE_FAULT) {
        printf("RSSI_METER WARN tune %u MHz failed (staying %u MHz)\n",
               boot_mhz, rf_get_frequency_mhz());
        s_state = METER_STATE_IDLE;
    }

    if (xTaskCreatePinnedToCore(meter_output_task, "rssi_out", 4096, NULL, 6,
                                NULL, 0) != pdPASS) {
        printf("RSSI_METER ERROR output_task\n");
        return;
    }

    printf("RSSI_METER READY f=%u mode=%s stream=1kHz dac=6bit cal=%.0f..%.0f dB "
           "win=%u ms%s%s\n",
           s_tuned_mhz,
           rf_native_agc_active() ? "native_agc" : "forced",
           s_pipe.cfg.db_lo, s_pipe.cfg.db_hi,
           (unsigned)s_pipe.cfg.window_max_ms,
           rssi_sdm_active() ? " sdm:on" : "",
#ifdef CONFIG_C5VRX_RSSI_BUS
           " bus:rx5808"
#else
           ""
#endif
    );
    meter_help();
    fflush(stdout);

    char line[64];
    for (;;) {
        int c = getchar();
        if (c < 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (c == 'H' || c == 'h') {
            meter_help();
        } else if (c == 'T' || c == 't') {
            meter_status();
        } else if (c == 'N' || c == 'n') {
            bool enable = !rf_native_agc_active();
            err = rf_request_native_agc_boot(enable);
            printf("RSSI_METER_NATIVE_AGC_ARMED next_boot=%s action=reboot err=%s\n",
                   enable ? "native_hw_agc" : "forced_gain", esp_err_to_name(err));
            fflush(stdout);
            vTaskDelay(pdMS_TO_TICKS(150));
            esp_restart();
        } else if (c == 'S' || c == 's') {
            meter_gain_sweep();
        } else if (c == 'V' || c == 'v') {
            s_dac_invert = !s_dac_invert;
            s_cfg.dac_invert = s_dac_invert;
            printf("DAC_POL %s (P saves)\n", s_dac_invert ? "inverted" : "normal");
        } else if (c == 'L' || c == 'l') {
            s_stream_paused = !s_stream_paused;
            printf("STREAM %s\n", s_stream_paused ? "paused" : "running");
        } else if (c == 'B' || c == 'b') {
            printf("RSSI_METER rebooting\n");
            fflush(stdout);
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_restart();
        } else if (c == 'U' || c == 'u') {
            meter_status();
        } else if (c == 'X' || c == 'x') {
            /* Arm the external auto-download circuit, then restart. GPIO10
             * charges the gate capacitor through a diode; the MOSFET holds
             * GPIO0 (BOOT) low across the restart, so the ROM samples
             * download mode and waits for esptool. The RC discharges in
             * ~1.5 s, so esptool's watchdog finish (seconds later) boots
             * the new app. Without the hardware this is a plain reboot. */
            gpio_config_t arm_io = {
                .pin_bit_mask = 1ULL << METER_DOWNLOAD_ARM_GPIO,
                .mode = GPIO_MODE_OUTPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE,
            };
            if (gpio_config(&arm_io) == ESP_OK) {
                gpio_set_level(METER_DOWNLOAD_ARM_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(150));
                gpio_set_level(METER_DOWNLOAD_ARM_GPIO, 0);
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            printf("RSSI_METER download-armed: flash now with "
                   "--before no_reset --after watchdog_reset\n");
            fflush(stdout);
            esp_restart();
        } else if (c == 'F' || c == 'f') {
            read_line(line, sizeof(line), -1);
            char *tok[2] = {NULL, NULL};
            int t = tokenize(line, tok, 2);
            if (t >= 1) {
                meter_set_frequency(parse_freq_token(tok[0]));
            } else {
                printf("FUSAGE F<mhz> or F<band><n>, e.g. F5800 / F R4\n");
            }
        } else if (c == 'C' || c == 'c') {
            read_line(line, sizeof(line), -1);
            char *tok[3] = {NULL, NULL, NULL};
            int t = tokenize(line, tok, 3);
            meter_calibration(t, tok);
        } else if (c == 'K' || c == 'k') {
            read_line(line, sizeof(line), -1);
            char *tok[3] = {NULL, NULL, NULL};
            int t = tokenize(line, tok, 3);
            if (t == 1 && !strcasecmp(tok[0], "off")) {
                rssi_pipeline_set_knee(&s_pipe, s_pipe.cfg.knee_db, 1.0f);
                s_cfg.knee_ratio = 1.0f;
                printf("KNEE off\n");
            } else if (t == 2) {
                float db = strtof(tok[0], NULL);
                float r = strtof(tok[1], NULL);
                rssi_pipeline_set_knee(&s_pipe, db, r);
                s_cfg.knee_db = db;
                s_cfg.knee_ratio = s_pipe.cfg.knee_ratio;
                printf("KNEE db=%.1f ratio=%.1f (P saves)\n", db, s_pipe.cfg.knee_ratio);
            } else {
                printf("KUSAGE K <db> <ratio> | K off\n");
            }
        } else if (c == 'E' || c == 'e') {
            read_line(line, sizeof(line), -1);
            char *tok[2] = {NULL, NULL};
            int t = tokenize(line, tok, 2);
            if (t == 1) {
                float a = strtof(tok[0], NULL);
                if (a > 0.0f && a <= 1.0f) {
                    rssi_pipeline_set_ema_alpha(&s_pipe, a);
                    s_cfg.ema_alpha = a;
                    printf("EMA alpha=%.2f (P saves)\n", a);
                }
            }
            if (t != 1) {
                printf("EUSAGE E <alpha 0.01..1>\n");
            }
        } else if (c == 'W' || c == 'w') {
            read_line(line, sizeof(line), -1);
            char *tok[2] = {NULL, NULL};
            int t = tokenize(line, tok, 2);
            if (t == 1) {
                long ms = strtol(tok[0], NULL, 10);
                if (ms >= 0 && ms <= (long)RSSI_PIPELINE_WIN_CAP) {
                    s_pipe.cfg.window_max_ms = (uint32_t)ms;
                    s_pipe.win_head = 0;
                    s_pipe.win_count = 0;
                    s_cfg.window_max_ms = (uint16_t)ms;
                    printf("WINDOW %ld ms (P saves)\n", ms);
                }
            }
            if (t != 1) {
                printf("WUSAGE W <ms 0..200>\n");
            }
        } else if (c == 'P' || c == 'p') {
            bool ok = meter_cfg_save(&s_cfg);
            printf("SAVE %s (boot_mhz=%u)\n", ok ? "ok" : "FAILED", s_cfg.boot_mhz);
        } else if (c == 'D' || c == 'd') {
            meter_defaults();
        } else if (c == 'O' || c == 'o') {
            /* OUT <0..255> | OUT off */
            read_line(line, sizeof(line), -1);
            char *tok[2] = {NULL, NULL};
            int t = tokenize(line, tok, 2);
            if (t == 1 && !strcasecmp(tok[0], "off")) {
                s_force_counts = -1;
                printf("OUT restored\n");
            } else if (t == 1) {
                long v = strtol(tok[0], NULL, 10);
                if (v >= 0 && v <= 255) {
                    s_force_counts = (int)v;
                    printf("OUT fixed at %ld\n", v);
                }
            } else {
                printf("OUTUSAGE OUT <0..255> | OUT off\n");
            }
        } else if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
            /* skip */
        } else {
            /* Line-style commands: SCAN, CH */
            read_line(line, sizeof(line), c);
            char *tok[3] = {NULL, NULL, NULL};
            int t = tokenize(line, tok, 3);
            if (t >= 1 && !strcasecmp(tok[0], "SCAN")) {
                uint32_t dwell = (t >= 2) ? (uint32_t)strtoul(tok[1], NULL, 10) : 0u;
                meter_scan(dwell);
            } else if (t >= 2 && !strcasecmp(tok[0], "CH")) {
                meter_set_frequency(parse_freq_token(tok[1]));
            } else if (t >= 1) {
                printf("RSSI_METER unknown cmd=%s (H for help)\n", tok[0]);
            }
        }
        fflush(stdout);
    }
}

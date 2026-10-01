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
 * Purpose: drone race timer proximity sensing. The receiver is tuned to a
 * fixed VTX frequency and reports how much RF power is in the channel, the
 * same job as the analog RSSI pin on a classic VRX. Video is deliberately
 * sacrificed; flash the normal firmware again for video.
 *
 * Outputs:
 *   USB CDC, ~1 kHz line per sample:
 *       R:<rssi_dBm> NF:<noise_floor_dBm> G:<gain_idx> M:<0|1>
 *     M:0 = firmware forced gain (G valid), M:1 = native hardware AGC
 *     (G = -1, gain is hardware-owned). -127 means the PHY value was
 *     invalid/stale for that field.
 *   6-bit analog DAC: the same D4..D9 resistor network as the video output,
 *     driven by static GPIO writes (no GDMA). RSSI dBm is mapped onto
 *     0..63 and clamped; V inverts the polarity.
 *
 * Commands:
 *   H      help
 *   T      one status line (frequency, mode, DAC range, polarity, paused)
 *   N      toggle native hardware AGC, persist to NVS, reboot
 *   S      fixed-gain sweep G15..G81 (forced mode only): the pre-gain /
 *          post-gain RSSI oracle. Restores the previous gain.
 *   F<mhz> retune to an arbitrary MHz inside the C5 5 GHz window
 *          (e.g. F5800). Same two-step path as FPV channel retunes: nearest
 *          public Wi-Fi center first, then the phy_set_freq() delta.
 *   V      invert DAC polarity (runtime)
 *   L      pause / resume the 1 kHz stream
 *
 * Invariants honoured: in native AGC mode every firmware gain write stays
 * refused at rf_set_rx_gain(); the DAC pin order and network are unchanged;
 * USB is the only consumer and nothing paces anything because no IQ
 * production exists in this image.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "heap_memory_layout.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rf.h"

/* The allocation selector disconnects BOTH 64 KiB dump banks from the CPU.
 * Same proven reservation as the capture-only meter; the background
 * continuous-modem dump writer keeps streaming into them in this image. */
SOC_RESERVE_MEMORY_REGION(0x4082ffc0, 0x40850040, c5vrx_rssi_meter_ram);
extern char _bss_end;

/* Race timer default center; F<mhz> retunes at runtime. */
#define METER_BOOT_FREQ_MHZ  5800u

/* Six-bit video DAC. Same pin order as the video path (byte bit i -> pin i,
 * PARLIO LSB-first): never change the order, the physical
 * 8.2k/3.9k/2k/1k/470R/240R + 200R network is wired to exactly these pins. */
static const gpio_num_t s_dac_gpio[6] = {23, 24, 11, 12, 8, 9};

/* RSSI -> DAC code window. Tune after the first bench session; the raw dBm
 * is always on USB so the mapping can be adjusted from data. */
#define RSSI_DAC_DBM_LO   (-100)
#define RSSI_DAC_DBM_HI   (-40)

static volatile bool s_stream_paused;
static volatile bool s_dac_invert;
static volatile int s_meter_gain = 52; /* rf_start() forced-gain default */

static void dac_write(uint8_t code)
{
    for (unsigned i = 0u; i < 6u; ++i) {
        gpio_set_level(s_dac_gpio[i], (code >> i) & 1u);
    }
}

static uint8_t rssi_to_code(int dbm)
{
    int code = (dbm - RSSI_DAC_DBM_LO) * 63 / (RSSI_DAC_DBM_HI - RSSI_DAC_DBM_LO);
    if (code < 0) code = 0;
    if (code > 63) code = 63;
    if (s_dac_invert) code = 63 - code;
    return (uint8_t)code;
}

static void meter_output_task(void *arg)
{
    for (;;) {
        if (!s_stream_paused) {
            int rssi = -127;
            int nf = -127;
            bool rssi_ok = rf_try_get_wideband_rssi_dbm(&rssi);
            bool nf_ok = rf_try_get_noise_floor_dbm(&nf);
            if (rssi_ok) {
                bool native = rf_native_agc_active();
                printf("R:%d NF:%d G:%d M:%d\n",
                       rssi,
                       nf_ok ? nf : -127,
                       native ? -1 : s_meter_gain,
                       native ? 1 : 0);
                dac_write(rssi_to_code(rssi));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1)); /* ~1 kHz; FreeRTOS tick = 1 ms */
    }
}

static void meter_help(void)
{
    printf("RSSI_METER commands:\n"
           " H   this help\n"
           " T   status line\n"
           " N   toggle native HW AGC (persist, reboot)\n"
           " S   fixed-gain sweep G15..G81 (forced mode only, pre/post-gain oracle)\n"
           " F<mhz>  retune, 5180-5885, e.g. F5800\n"
           " V   invert DAC polarity\n"
           " L   pause/resume 1 kHz stream\n");
}

static void meter_status(void)
{
    printf("ST f:%u mode:%s dac:%d..%d dBm pol:%s paused:%d gain:%d\n",
           rf_get_frequency_mhz(),
           rf_native_agc_active() ? "native_agc" : "forced",
           RSSI_DAC_DBM_LO, RSSI_DAC_DBM_HI,
           s_dac_invert ? "inverted" : "normal",
           s_stream_paused ? 1 : 0,
           s_meter_gain);
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
    printf("SWEEP start f=%u restore_g=%d\n", rf_get_frequency_mhz(), restore);
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

static void meter_set_frequency(const char *tok)
{
    char *end = NULL;
    unsigned long mhz = strtoul(tok, &end, 10);
    if (end == tok || mhz < 1000ul || mhz > 9999ul) {
        printf("FUSAGE F<mhz> e.g. F5800\n");
        return;
    }
    esp_err_t err = rf_set_frequency_mhz((uint16_t)mhz);
    printf("F:%lu err=%s f=%u\n",
           (unsigned long)mhz, esp_err_to_name(err), rf_get_frequency_mhz());
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

    if ((err = rf_set_frequency_mhz(METER_BOOT_FREQ_MHZ)) != ESP_OK) {
        printf("RSSI_METER WARN tune %u MHz err=%s (staying %u MHz)\n",
               METER_BOOT_FREQ_MHZ, esp_err_to_name(err), rf_get_frequency_mhz());
    }

    if (xTaskCreatePinnedToCore(meter_output_task, "rssi_out", 4096, NULL, 6,
                                NULL, 1) != pdPASS) {
        printf("RSSI_METER ERROR output_task\n");
        return;
    }

    printf("RSSI_METER READY f=%u mode=%s stream=1kHz dac=6bit %d..%d dBm\n",
           rf_get_frequency_mhz(),
           rf_native_agc_active() ? "native_agc" : "forced",
           RSSI_DAC_DBM_LO, RSSI_DAC_DBM_HI);
    meter_help();
    fflush(stdout);

    for (;;) {
        int c = getchar();
        if (c < 0) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
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
            printf("DAC_POL %s\n", s_dac_invert ? "inverted" : "normal");
        } else if (c == 'L' || c == 'l') {
            s_stream_paused = !s_stream_paused;
            printf("STREAM %s\n", s_stream_paused ? "paused" : "running");
        } else if (c == 'F' || c == 'f') {
            char tok[8] = {0};
            unsigned n = 0u;
            for (;;) {
                int d = getchar();
                if (d < 0 || d < '0' || d > '9' || n >= 7u) break;
                tok[n++] = (char)d;
            }
            meter_set_frequency(tok);
        } else {
            printf("RSSI_METER unknown cmd=%c (H for help)\n", c);
        }
        fflush(stdout);
    }
}

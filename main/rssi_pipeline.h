/*
 * rssi_pipeline.h - RSSI shaping for the standalone meter.
 *
 * Ported from the FPVGateC5RX project (github.com/LouisHitchcock/FPVGateC5RX,
 * core/rssi_pipeline.h, CC BY-NC-SA 4.0) to C. It turns raw phy_get_rssi()
 * readings (dBm) into the calibrated 0..255 strength value that drives the
 * analog output and the P: field on the USB stream.
 *
 * Stages, in order:
 *   1. peak-hold window max: the C5's RSSI reading dips ~10 dB for 2-3 ms
 *      every ~25 ms when the AGC refreshes; a window longer than that cycle
 *      hides the dips. Rises come through immediately, falls are delayed by
 *      at most the window length.
 *   2. optional median-of-3: drops single-sample glitches.
 *   3. optional EMA low-pass: out += alpha * (in - out); alpha 1.0 = off.
 *   4. optional soft knee: above kneeDb every kneeRatio dB counts as 1 dB,
 *      squeezing strong signals together like a saturating receiver.
 *   5. calibration: dbLo reads as 0, dbHi reads as 255 (both ends pass
 *      through the knee so dbHi is exactly 255).
 *
 * The pipeline also blanks its output for settleMs after a retune and flags
 * itself stalled when no sample arrives for stallMs.
 */
#ifndef RSSI_PIPELINE_H
#define RSSI_PIPELINE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* Calibration: dbLo reads as 0, dbHi reads as 255. */
    float db_lo;
    float db_hi;
    /* EMA low-pass coefficient, (0, 1]; 1.0 = no smoothing. */
    float ema_alpha;
    bool  use_median3;
    /* Peak-hold window in ms of samples; 0 = off. Capped at the ring size. */
    uint32_t window_max_ms;
    /* Soft knee: above knee_db, extra signal is divided by knee_ratio.
     * knee_ratio <= 1.0 disables the knee. */
    float knee_db;
    float knee_ratio;
    /* Hold the output this long after a retune (ms). */
    uint32_t settle_ms;
    /* No sample for this long marks the pipeline stalled (ms). */
    uint32_t stall_ms;
} rssi_pipeline_cfg_t;

/* Ring capacity = longest peak-hold window in ms of samples. At the meter's
 * 1 kHz cadence one sample per ms, so 200 samples = 200 ms max window. */
#define RSSI_PIPELINE_WIN_CAP 200u

typedef struct {
    rssi_pipeline_cfg_t cfg;
    float win_db[RSSI_PIPELINE_WIN_CAP];
    uint32_t win_t[RSSI_PIPELINE_WIN_CAP];
    uint16_t win_head;
    uint16_t win_count;
    float med[3];
    uint8_t med_count;
    uint8_t med_idx;
    float ema_db;
    bool have_ema;
    uint8_t counts;
    uint8_t held_counts;
    bool valid;
    bool stalled;
    uint32_t last_sample_ms;
    uint32_t blank_until_ms;
    bool have_sample;
} rssi_pipeline_t;

void rssi_pipeline_begin(rssi_pipeline_t *p, const rssi_pipeline_cfg_t *cfg);
void rssi_pipeline_reset(rssi_pipeline_t *p);

/* Add one reading (dBm), timestamped in ms. */
void rssi_pipeline_on_sample(rssi_pipeline_t *p, float db, uint32_t now_ms);
/* Call after a retune: holds the output while the radio settles. */
void rssi_pipeline_on_tune(rssi_pipeline_t *p, uint32_t now_ms);
/* Call regularly, even with no readings, so stalls get noticed. */
void rssi_pipeline_tick(rssi_pipeline_t *p, uint32_t now_ms);

uint8_t rssi_pipeline_counts(const rssi_pipeline_t *p);
float rssi_pipeline_smoothed_db(const rssi_pipeline_t *p);
bool rssi_pipeline_valid(const rssi_pipeline_t *p);
bool rssi_pipeline_stalled(const rssi_pipeline_t *p);

void rssi_pipeline_set_calibration(rssi_pipeline_t *p, float db_lo, float db_hi);
void rssi_pipeline_set_ema_alpha(rssi_pipeline_t *p, float alpha);
void rssi_pipeline_set_knee(rssi_pipeline_t *p, float db, float ratio);

#ifdef __cplusplus
}
#endif

#endif /* RSSI_PIPELINE_H */

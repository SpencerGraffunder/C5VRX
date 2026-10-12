/*
 * rssi_signal.h - strength shaping for the RSSI meter.
 *
 * Written for this project. Turns the raw 1 kHz RSSI reading (dBm) into:
 *   - a smoothed level (EMA low-pass),
 *   - a calibrated 0..255 strength value (linear dBm window),
 *   - a learned quiet baseline (asymmetric tracking: falls fast toward
 *     lower readings, rises slowly, so the quiet floor is tracked even
 *     when the meter boots with a strong transmitter in range),
 *   - a signal-present decision (margin above baseline + hysteresis),
 *   - a freshness flag. A reading that stops changing is a latched
 *     register, not a measurement: the 2026-10-05 bench showed the C5
 *     holding one RSSI value for minutes with the transmitter toggled,
 *     so nothing downstream may call a stale reading "signal present".
 *
 * The module is pure C with no hardware includes so it can be unit tested
 * on a host (tools/test_rssi_meter.c).
 */
#ifndef C5VRX_RSSI_SIGNAL_H
#define C5VRX_RSSI_SIGNAL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    /* dBm level that calibrates to 0 and to 255. */
    float db_lo;
    float db_hi;
    /* EMA low-pass coefficient, 0 < alpha <= 1 (1 = no smoothing). */
    float ema_alpha;
    /* Present when the smoothed level is at least margin_db above the
     * learned baseline; released margin_db - hysteresis_db above it. */
    float margin_db;
    float hysteresis_db;
    /* Below this level a 40 MHz channel cannot physically read (kTB floor
     * ~ -98 dBm), so the detector is not measuring; readings are invalid. */
    float floor_db;
    /* Baseline slow-rise rate (the fast-fall rate is a module constant). */
    float baseline_alpha;
    /* A raw reading unchanged for this many ms is stale. */
    uint32_t fresh_ms;
} rssi_signal_cfg_t;

typedef struct {
    rssi_signal_cfg_t cfg;

    bool primed;          /* at least one sample since begin/on_tune */
    float smoothed;       /* EMA of the raw reading, dBm */
    float baseline;       /* learned quiet level, dBm */
    bool baseline_valid;  /* baseline seeded by a sample */
    float last_raw;       /* last accepted raw reading */
    uint32_t last_change_ms;
    bool present;         /* signal-present state, with hysteresis */
    bool fresh;           /* refreshed by tick() from last_change_ms */
} rssi_signal_t;

/* Copy the config in and clear all signal state. */
void rssi_signal_begin(rssi_signal_t *s, const rssi_signal_cfg_t *cfg);

/* One raw dBm sample from the receiver (call only with a valid reading). */
void rssi_signal_on_sample(rssi_signal_t *s, float dbm, uint32_t now_ms);

/* The receiver changed channel: the previous reading no longer applies. */
void rssi_signal_on_tune(rssi_signal_t *s, uint32_t now_ms);

/* Refresh the freshness flag; call once per output period. */
void rssi_signal_tick(rssi_signal_t *s, uint32_t now_ms);

/* True once a sample has arrived and the level is physically possible. */
bool rssi_signal_valid(const rssi_signal_t *s);

/* Smoothed level in dBm (0 before the first sample). */
float rssi_signal_smoothed_db(const rssi_signal_t *s);

/* Calibrated strength 0..255 (linear over db_lo..db_hi, clamped). */
uint8_t rssi_signal_counts(const rssi_signal_t *s);

/* True while the reading has changed recently (not a latched register). */
bool rssi_signal_fresh(const rssi_signal_t *s);

/* True when the smoothed level clears the learned baseline by the margin
 * and the reading is valid and fresh. */
bool rssi_signal_signal_present(const rssi_signal_t *s);

/* Learned quiet level in dBm (0 before it has been seeded). */
float rssi_signal_baseline_db(const rssi_signal_t *s);

/* Forget the baseline; the next sample re-seeds it. Use after a scan or
 * when the site / transmitter changes. */
void rssi_signal_reset_baseline(rssi_signal_t *s);

void rssi_signal_set_calibration(rssi_signal_t *s, float db_lo, float db_hi);
void rssi_signal_set_ema_alpha(rssi_signal_t *s, float alpha);
void rssi_signal_set_margin(rssi_signal_t *s, float margin_db,
                            float hysteresis_db);

#endif /* C5VRX_RSSI_SIGNAL_H */

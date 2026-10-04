/*
 * rssi_pipeline.c - RSSI shaping for the standalone meter.
 *
 * Ported from FPVGateC5RX (core/rssi_pipeline.cpp, CC BY-NC-SA 4.0). See
 * rssi_pipeline.h for the stage order and rationale.
 */
#include "rssi_pipeline.h"

#include <math.h>

static float med3(float a, float b, float c)
{
    float mx = a > b ? a : b;
    float mn = a < b ? a : b;
    float m = c > mx ? mx : (c < mn ? mn : c);
    return m;
}

void rssi_pipeline_reset(rssi_pipeline_t *p)
{
    p->med_count = 0; p->med_idx = 0;
    p->have_ema = false; p->ema_db = p->cfg.db_lo;
    p->counts = 0; p->held_counts = 0;
    p->valid = false; p->stalled = false;
    p->have_sample = false;
    p->last_sample_ms = 0; p->blank_until_ms = 0;
    p->win_head = 0; p->win_count = 0;
    p->baseline_db = p->cfg.db_lo;
    p->have_baseline = false;
    p->no_carrier = false;
    p->fresh = false;
    p->have_last_raw = false; p->last_raw_db = 0.0f;
    p->signal_present = false;
    p->last_change_ms = 0;
}

void rssi_pipeline_begin(rssi_pipeline_t *p, const rssi_pipeline_cfg_t *cfg)
{
    p->cfg = *cfg;
    if (p->cfg.ema_alpha <= 0.0f || p->cfg.ema_alpha > 1.0f) {
        p->cfg.ema_alpha = 1.0f;
    }
    if (p->cfg.knee_ratio < 1.0f) {
        p->cfg.knee_ratio = 1.0f;
    }
    if (p->cfg.window_max_ms > RSSI_PIPELINE_WIN_CAP) {
        p->cfg.window_max_ms = RSSI_PIPELINE_WIN_CAP;
    }
    /* Defaults for the signal-present stage when the caller left them unset
     * (0 is not a meaningful value for any of these). */
    if (p->cfg.margin_db <= 0.0f) p->cfg.margin_db = 12.0f;
    if (p->cfg.hysteresis_db < 0.0f) p->cfg.hysteresis_db = 6.0f;
    if (p->cfg.baseline_alpha <= 0.0f || p->cfg.baseline_alpha > 1.0f) {
        p->cfg.baseline_alpha = 0.002f;   /* ~500 ms time constant at 1 kHz */
    }
    if (p->cfg.floor_db >= 0.0f) p->cfg.floor_db = -105.0f;
    if (p->cfg.fresh_ms == 0u) p->cfg.fresh_ms = 1000u;
    rssi_pipeline_reset(p);
}

static float window_max(rssi_pipeline_t *p, float db, uint32_t now_ms)
{
    if (p->cfg.window_max_ms == 0u) {
        return db;
    }
    /* Append the new sample (overwrite the oldest if the buffer is full). */
    uint16_t idx = (uint16_t)((p->win_head + p->win_count) % RSSI_PIPELINE_WIN_CAP);
    if (p->win_count == RSSI_PIPELINE_WIN_CAP) {
        p->win_head = (uint16_t)((p->win_head + 1u) % RSSI_PIPELINE_WIN_CAP);
        p->win_count--;
    }
    p->win_db[idx] = db;
    p->win_t[idx] = now_ms;
    p->win_count++;
    /* Drop samples that have aged out of the window. */
    while (p->win_count > 1u &&
           (now_ms - p->win_t[p->win_head]) >= p->cfg.window_max_ms) {
        p->win_head = (uint16_t)((p->win_head + 1u) % RSSI_PIPELINE_WIN_CAP);
        p->win_count--;
    }
    float mx = p->win_db[p->win_head];
    for (uint16_t i = 1u; i < p->win_count; ++i) {
        float v = p->win_db[(p->win_head + i) % RSSI_PIPELINE_WIN_CAP];
        if (v > mx) {
            mx = v;
        }
    }
    return mx;
}

static float compress(const rssi_pipeline_t *p, float db)
{
    if (p->cfg.knee_ratio <= 1.0f || db <= p->cfg.knee_db) {
        return db;
    }
    return p->cfg.knee_db + (db - p->cfg.knee_db) / p->cfg.knee_ratio;
}

static uint8_t map_counts(const rssi_pipeline_t *p, float db)
{
    /* Both the reading and the top of the scale go through the soft ceiling,
     * so dbHi still reads exactly 255 whatever the knee setting. */
    float lo = compress(p, p->cfg.db_lo);
    float span = compress(p, p->cfg.db_hi) - lo;
    if (span <= 0.0f) {
        return 0u;
    }
    float x = (compress(p, db) - lo) / span * 255.0f;
    if (x < 0.0f) x = 0.0f;
    if (x > 255.0f) x = 255.0f;
    return (uint8_t)(x + 0.5f);
}

void rssi_pipeline_on_tune(rssi_pipeline_t *p, uint32_t now_ms)
{
    /* Hold the current output while the radio settles, and mark it invalid. */
    p->held_counts = p->counts;
    p->blank_until_ms = now_ms + p->cfg.settle_ms;
    p->valid = false;
    /* Keep the smoothed value, so the output doesn't restart from zero, but
     * forget the old channel's readings. */
    p->med_count = 0; p->med_idx = 0;
    p->win_head = 0; p->win_count = 0;
}

void rssi_pipeline_on_sample(rssi_pipeline_t *p, float db, uint32_t now_ms)
{
    p->last_sample_ms = now_ms;
    p->have_sample = true;
    p->stalled = false;

    /* Freshness. The RSSI register is only refreshed when the detector is
     * actually measuring energy: with the VTX off it held one value for 60 s
     * (59,996 samples, zero changes), with the VTX on it changed constantly.
     * So a value that repeats for fresh_ms is evidence there is no carrier,
     * and must not be reported as "drone in range" - a stale high value from a
     * finished pass would otherwise hold a lap timer's threshold open. */
    if (!p->have_last_raw || db != p->last_raw_db) {
        p->last_change_ms = now_ms;
        p->have_last_raw = true;
        p->last_raw_db = db;
    }
    p->fresh = (now_ms - p->last_change_ms) <= p->cfg.fresh_ms;
    /* A reading below the physical channel floor means "not measuring", not
     * "very weak signal" (the register reported -124, which is impossible for
     * a 40 MHz channel whose kTB floor is about -98 dBm). */
    p->no_carrier = (db < p->cfg.floor_db);

    /* Learned baseline. The vendor noise-floor read is invalid in forced-gain
     * mode on this board, so the meter learns its own quiet level. Only
     * readings that are not themselves a pass train it, so a drone in range
     * cannot lift the reference level. */
    if (!p->no_carrier) {
        if (!p->have_baseline) {
            p->baseline_db = db;
            p->have_baseline = true;
        } else if (db < p->baseline_db + p->cfg.margin_db) {
            p->baseline_db += p->cfg.baseline_alpha * (db - p->baseline_db);
        }
    }

    float held = window_max(p, db, now_ms);   /* hides the AGC-refresh dips */
    float filtered = held;
    if (p->cfg.use_median3) {
        p->med[p->med_idx] = held;
        p->med_idx = (uint8_t)((p->med_idx + 1u) % 3u);
        if (p->med_count < 3u) {
            p->med_count++;
        }
        if (p->med_count == 3u) {
            filtered = med3(p->med[0], p->med[1], p->med[2]);
        }
    }

    if (!p->have_ema) {
        p->ema_db = filtered;
        p->have_ema = true;
    } else {
        p->ema_db += p->cfg.ema_alpha * (filtered - p->ema_db);
    }

    uint8_t mapped = map_counts(p, p->ema_db);

    if (now_ms < p->blank_until_ms) {
        /* Still settling: keep showing the last good value, and do not let a
         * half-measured reading change the signal-present state. */
        p->counts = p->held_counts;
        p->valid = false;
        return;
    }

    p->counts = mapped;
    p->valid = true;

    if (!p->have_baseline || p->no_carrier || !p->fresh) {
        p->signal_present = false;
    } else if (!p->signal_present) {
        if (p->ema_db >= p->baseline_db + p->cfg.margin_db) {
            p->signal_present = true;
        }
    } else if (p->ema_db <= p->baseline_db + p->cfg.margin_db - p->cfg.hysteresis_db) {
        p->signal_present = false;
    }
}

void rssi_pipeline_tick(rssi_pipeline_t *p, uint32_t now_ms)
{
    if (p->have_sample && (now_ms - p->last_sample_ms) > p->cfg.stall_ms) {
        p->stalled = true;
        p->valid = false;
    }
}

uint8_t rssi_pipeline_counts(const rssi_pipeline_t *p)
{
    return p->counts;
}

float rssi_pipeline_smoothed_db(const rssi_pipeline_t *p)
{
    return p->ema_db;
}

bool rssi_pipeline_valid(const rssi_pipeline_t *p)
{
    return p->valid;
}

bool rssi_pipeline_stalled(const rssi_pipeline_t *p)
{
    return p->stalled;
}

void rssi_pipeline_set_calibration(rssi_pipeline_t *p, float db_lo, float db_hi)
{
    if (db_hi > db_lo) {
        p->cfg.db_lo = db_lo;
        p->cfg.db_hi = db_hi;
    }
}

void rssi_pipeline_set_ema_alpha(rssi_pipeline_t *p, float alpha)
{
    if (alpha > 0.0f && alpha <= 1.0f) {
        p->cfg.ema_alpha = alpha;
    }
}

void rssi_pipeline_set_knee(rssi_pipeline_t *p, float db, float ratio)
{
    p->cfg.knee_db = db;
    p->cfg.knee_ratio = ratio < 1.0f ? 1.0f : ratio;
}

void rssi_pipeline_set_margin(rssi_pipeline_t *p, float margin_db, float hysteresis_db)
{
    if (margin_db > 0.0f) p->cfg.margin_db = margin_db;
    if (hysteresis_db >= 0.0f) p->cfg.hysteresis_db = hysteresis_db;
}

void rssi_pipeline_reset_baseline(rssi_pipeline_t *p)
{
    p->have_baseline = false;
    p->have_last_raw = false;
    p->signal_present = false;
}

bool rssi_pipeline_signal_present(const rssi_pipeline_t *p)
{
    return p->signal_present;
}

float rssi_pipeline_baseline_db(const rssi_pipeline_t *p)
{
    return p->baseline_db;
}

float rssi_pipeline_signal_db(const rssi_pipeline_t *p)
{
    if (!p->have_baseline) return 0.0f;
    return p->ema_db - p->baseline_db;
}

bool rssi_pipeline_fresh(const rssi_pipeline_t *p)
{
    return p->fresh;
}

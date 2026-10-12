/*
 * rssi_signal.c - strength shaping for the RSSI meter. See rssi_signal.h.
 */
#include "rssi_signal.h"

#include <math.h>

/* The baseline must fall to a new quiet floor within a few hundred ms
 * (a VTX switching off must not hold "signal present" open), while a slow
 * rise keeps a distant drone from pulling the baseline up to itself.
 * 0.05 at 1 kHz is a ~200 ms fall time constant. */
#define RSSI_SIGNAL_BASELINE_FAST_FALL 0.05f

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void rssi_signal_begin(rssi_signal_t *s, const rssi_signal_cfg_t *cfg)
{
    s->cfg = *cfg;
    s->primed = false;
    s->smoothed = 0.0f;
    s->baseline = 0.0f;
    s->baseline_valid = false;
    s->last_raw = 0.0f;
    s->last_change_ms = 0;
    s->present = false;
    s->fresh = false;
}

void rssi_signal_on_sample(rssi_signal_t *s, float dbm, uint32_t now_ms)
{
    if (!isfinite(dbm)) {
        return;
    }
    if (!s->primed) {
        s->smoothed = dbm;
        s->primed = true;
    } else {
        s->smoothed += s->cfg.ema_alpha * (dbm - s->smoothed);
    }

    if (!s->baseline_valid) {
        s->baseline = s->smoothed;
        s->baseline_valid = true;
    } else if (s->smoothed < s->baseline) {
        /* Fast fall toward a lower reading. */
        s->baseline += RSSI_SIGNAL_BASELINE_FAST_FALL *
                       (s->smoothed - s->baseline);
    } else {
        /* Slow rise so a weak signal cannot lift the floor into itself. */
        s->baseline += s->cfg.baseline_alpha * (s->smoothed - s->baseline);
    }

    if (dbm != s->last_raw) {
        s->last_raw = dbm;
        s->last_change_ms = now_ms;
    }
}

void rssi_signal_on_tune(rssi_signal_t *s, uint32_t now_ms)
{
    /* The reading is meaningless until the new channel settles. */
    s->primed = false;
    s->present = false;
    s->last_change_ms = now_ms;
}

void rssi_signal_tick(rssi_signal_t *s, uint32_t now_ms)
{
    if (!s->primed) {
        s->fresh = false;
        return;
    }
    bool moving = (now_ms - s->last_change_ms) <= s->cfg.fresh_ms;
    if (!moving) {
        s->fresh = false;
        s->present = false;   /* a latched reading is not a signal */
        return;
    }
    s->fresh = true;

    float over = s->smoothed - s->baseline;
    if (!s->present) {
        s->present = (over >= s->cfg.margin_db);
    } else {
        s->present = (over >= s->cfg.margin_db - s->cfg.hysteresis_db);
    }
}

bool rssi_signal_valid(const rssi_signal_t *s)
{
    return s->primed && s->smoothed >= s->cfg.floor_db;
}

float rssi_signal_smoothed_db(const rssi_signal_t *s)
{
    return s->primed ? s->smoothed : 0.0f;
}

uint8_t rssi_signal_counts(const rssi_signal_t *s)
{
    if (!s->primed) {
        return 0u;
    }
    float span = s->cfg.db_hi - s->cfg.db_lo;
    if (span <= 0.0f) {
        return 0u;
    }
    float v = (s->smoothed - s->cfg.db_lo) / span * 255.0f;
    return (uint8_t)lroundf(clampf(v, 0.0f, 255.0f));
}

bool rssi_signal_fresh(const rssi_signal_t *s)
{
    return s->fresh;
}

bool rssi_signal_signal_present(const rssi_signal_t *s)
{
    return s->present && s->fresh && rssi_signal_valid(s);
}

float rssi_signal_baseline_db(const rssi_signal_t *s)
{
    return s->baseline_valid ? s->baseline : 0.0f;
}

void rssi_signal_reset_baseline(rssi_signal_t *s)
{
    s->baseline_valid = false;
    s->present = false;
}

void rssi_signal_set_calibration(rssi_signal_t *s, float db_lo, float db_hi)
{
    s->cfg.db_lo = db_lo;
    s->cfg.db_hi = db_hi;
}

void rssi_signal_set_ema_alpha(rssi_signal_t *s, float alpha)
{
    if (alpha > 0.0f && alpha <= 1.0f) {
        s->cfg.ema_alpha = alpha;
    }
}

void rssi_signal_set_margin(rssi_signal_t *s, float margin_db,
                            float hysteresis_db)
{
    if (margin_db > 0.0f && margin_db <= 60.0f &&
        hysteresis_db >= 0.0f && hysteresis_db <= margin_db) {
        s->cfg.margin_db = margin_db;
        s->cfg.hysteresis_db = hysteresis_db;
    }
}

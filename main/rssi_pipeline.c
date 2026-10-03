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
        /* Still settling: keep showing the last good value. */
        p->counts = p->held_counts;
        p->valid = false;
    } else {
        p->counts = mapped;
        p->valid = true;
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

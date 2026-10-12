/*
 * Host unit tests for the meter's hardware-independent signal stage.
 *
 * Compiles main/rssi_signal.c natively (no ESP-IDF, no board) so the
 * shaping rules can be checked on a laptop:
 *
 *   cc -I main tools/test_rssi_meter.c main/rssi_signal.c -lm \
 *      -o /tmp/test_rssi_meter && /tmp/test_rssi_meter
 *
 * The cases encode the bench facts recorded in docs/rssi-meter.md (quiet
 * floor ~-90 dBm, close-range signal ~-45 dBm at gain 47, and the frozen
 * register the meter must refuse to call a signal), so an edit that breaks
 * one of them fails here instead of on the race track.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "rssi_signal.h"

static int s_fail;

#define CHECK(cond, msg) do { \
        if (!(cond)) { \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, msg); \
            s_fail++; \
        } else { \
            printf("ok   %s\n", msg); \
        } \
    } while (0)

static rssi_signal_t mk_default(void)
{
    rssi_signal_t s;
    rssi_signal_cfg_t c = {
        .db_lo = -100.0f,
        .db_hi = -40.0f,
        .ema_alpha = 1.0f,      /* off: the cases want exact values */
        .margin_db = 12.0f,
        .hysteresis_db = 6.0f,
        .floor_db = -105.0f,
        .baseline_alpha = 0.002f,
        .fresh_ms = 1000u,
    };
    rssi_signal_begin(&s, &c);
    return s;
}

/* Feed N identical samples 1 ms apart, advancing now_ms. */
static void feed(rssi_signal_t *s, float dbm, uint32_t *now_ms, int n)
{
    for (int i = 0; i < n; ++i) {
        *now_ms += 1;
        rssi_signal_on_sample(s, dbm, *now_ms);
        rssi_signal_tick(s, *now_ms);
    }
}

static void test_calibration_mapping(void)
{
    rssi_signal_t s = mk_default();
    uint32_t t = 0;
    feed(&s, -100.0f, &t, 3);
    CHECK(rssi_signal_counts(&s) == 0, "cal: db_lo maps to 0");
    feed(&s, -40.0f, &t, 3);
    CHECK(rssi_signal_counts(&s) == 255, "cal: db_hi maps to 255");
    feed(&s, -70.0f, &t, 3);
    CHECK(rssi_signal_counts(&s) == 128, "cal: mid-window maps to ~128");
    feed(&s, -30.0f, &t, 3);
    CHECK(rssi_signal_counts(&s) == 255, "cal: above window clamps to 255");
    feed(&s, -120.0f, &t, 3);
    CHECK(rssi_signal_counts(&s) == 0, "cal: below window clamps to 0");
}

static void test_baseline_recovers_from_hot_seed(void)
{
    /* The meter boots with a strong VTX in range: the baseline must fall
     * to the quiet floor after the transmitter switches off, and a later
     * pass must trip the signal decision. */
    rssi_signal_t s = mk_default();
    uint32_t t = 0;

    feed(&s, -45.0f, &t, 100);            /* VTX on, close range */
    feed(&s, -90.0f, &t, 2000);           /* VTX off: 2 s of quiet floor */
    float base = rssi_signal_baseline_db(&s);
    CHECK(base > -92.0f && base < -88.0f,
          "baseline: falls to the quiet floor from a hot seed");
    CHECK(!rssi_signal_signal_present(&s),
          "baseline: no signal while at the floor");

    feed(&s, -48.0f, &t, 5);              /* a pass goes back into range */
    CHECK(rssi_signal_signal_present(&s),
          "baseline: close-range pass trips signal-present");

    feed(&s, -90.0f, &t, 2000);           /* gone again */
    CHECK(!rssi_signal_signal_present(&s),
          "baseline: releases after the pass");
}

static void test_frozen_register_is_not_a_signal(void)
{
    /* Bench fact (2026-10-05): with the amplifier held fixed the C5 held
     * one RSSI value for 200 s while the transmitter was power-cycled.
     * A reading that stops changing must not read as "signal present". */
    rssi_signal_t s = mk_default();
    uint32_t t = 0;
    feed(&s, -90.0f, &t, 100);            /* learn the floor */
    feed(&s, -45.0f, &t, 20);             /* signal arrives */
    CHECK(rssi_signal_signal_present(&s),
          "fresh: a moving signal reads as present");
    feed(&s, -45.0f, &t, 6000);           /* value stops changing */
    CHECK(!rssi_signal_fresh(&s), "fresh: stale after fresh_ms of no change");
    CHECK(!rssi_signal_signal_present(&s),
          "fresh: a latched value is not a signal");
    feed(&s, -46.0f, &t, 10);             /* value moves again */
    CHECK(rssi_signal_fresh(&s), "fresh: recovers when the value moves");
}

static void test_floor_rejects_dead_readings(void)
{
    rssi_signal_t s = mk_default();
    uint32_t t = 0;
    feed(&s, -110.0f, &t, 10);            /* below the physical floor */
    CHECK(!rssi_signal_valid(&s), "floor: impossible level is invalid");
    CHECK(!rssi_signal_signal_present(&s),
          "floor: an invalid reading cannot be a signal");
}

static void test_tune_invalidates(void)
{
    rssi_signal_t s = mk_default();
    uint32_t t = 0;
    feed(&s, -45.0f, &t, 50);
    CHECK(rssi_signal_valid(&s), "tune: valid before the retune");
    rssi_signal_on_tune(&s, t + 1);
    CHECK(!rssi_signal_valid(&s), "tune: reading invalid until the next sample");
    feed(&s, -90.0f, &t, 5);
    CHECK(rssi_signal_valid(&s), "tune: valid again after the next sample");
}

static void test_hysteresis(void)
{
    rssi_signal_t s = mk_default();
    uint32_t t = 0;
    feed(&s, -90.0f, &t, 100);
    /* Just above margin - hysteresis: must not trip while clear. */
    feed(&s, -90.0f + 8.0f, &t, 5);
    CHECK(!rssi_signal_signal_present(&s),
          "hysteresis: within the release band stays clear");
    /* Above the full margin: trips. */
    feed(&s, -90.0f + 13.0f, &t, 5);
    CHECK(rssi_signal_signal_present(&s), "hysteresis: above margin trips");
    /* Back down into the band: must hold, not release. */
    feed(&s, -90.0f + 8.0f, &t, 5);
    CHECK(rssi_signal_signal_present(&s),
          "hysteresis: inside the band while present holds");
}

int main(void)
{
    test_calibration_mapping();
    test_baseline_recovers_from_hot_seed();
    test_frozen_register_is_not_a_signal();
    test_floor_rejects_dead_readings();
    test_tune_invalidates();
    test_hysteresis();
    if (s_fail) {
        printf("%d test(s) FAILED\n", s_fail);
        return 1;
    }
    printf("all tests passed\n");
    return 0;
}

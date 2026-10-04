/*
 * Host unit tests for the meter's hardware-independent modules.
 *
 * These compile rssi_pipeline.c and rx5808_bus.c natively (no ESP-IDF, no
 * board), so the shaping and bus rules can be checked on a laptop. Build and
 * run:
 *
 *   cc -I main tools/test_rssi_meter.c main/rssi_pipeline.c main/rx5808_bus.c \
 *      -lm -o /tmp/test_rssi_meter && /tmp/test_rssi_meter
 *
 * The cases encode the bench facts recorded in docs/rssi-meter.md, so a future
 * edit that breaks one of them fails here instead of on the race track.
 */
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "rssi_pipeline.h"
#include "rx5808_bus.h"

static int failures = 0;
static int checks = 0;

static void check(const char *what, bool ok)
{
    ++checks;
    if (!ok) {
        ++failures;
        printf("FAIL %s\n", what);
    } else {
        printf("ok   %s\n", what);
    }
}

static void check_near(const char *what, float got, float want, float tol)
{
    ++checks;
    if (fabsf(got - want) > tol) {
        ++failures;
        printf("FAIL %s got=%.2f want=%.2f\n", what, got, want);
    } else {
        printf("ok   %s (%.2f)\n", what, got);
    }
}

/* ---- pipeline ---- */

static rssi_pipeline_cfg_t base_cfg(void)
{
    rssi_pipeline_cfg_t c;
    c.db_lo = -100.0f;
    c.db_hi = -40.0f;
    c.ema_alpha = 1.0f;          /* no smoothing, like the reference default */
    c.use_median3 = false;       /* off so each test isolates one stage */
    c.window_max_ms = 0;         /* peak-hold off */
    c.knee_db = -55.0f;
    c.knee_ratio = 1.0f;         /* knee off */
    c.settle_ms = 35;
    c.stall_ms = 200;
    c.floor_db = -105.0f;
    c.baseline_alpha = 0.002f;
    c.margin_db = 12.0f;
    c.hysteresis_db = 6.0f;
    c.fresh_ms = 1000u;
    return c;
}

static void test_calibration(void)
{
    rssi_pipeline_t p;
    rssi_pipeline_cfg_t c = base_cfg();
    rssi_pipeline_begin(&p, &c);

    rssi_pipeline_on_sample(&p, -100.0f, 100);
    check("calibration: db_lo reads 0", rssi_pipeline_counts(&p) == 0u);
    rssi_pipeline_on_sample(&p, -40.0f, 101);
    check("calibration: db_hi reads 255", rssi_pipeline_counts(&p) == 255u);
    rssi_pipeline_on_sample(&p, -70.0f, 102);
    check("calibration: midpoint reads 128", rssi_pipeline_counts(&p) == 128u);

    /* A below-floor reading is not a weak signal, it is the detector not
     * measuring: it must map to 0 and must not train the baseline. */
    rssi_pipeline_reset_baseline(&p);
    rssi_pipeline_on_sample(&p, -124.0f, 103);
    check("below floor maps to 0", rssi_pipeline_counts(&p) == 0u);
    check("below floor is not a signal", !rssi_pipeline_signal_present(&p));
    check_near("below floor does not train baseline",
               rssi_pipeline_baseline_db(&p), -100.0f, 0.01f);
}

static void test_peak_hold_and_median(void)
{
    rssi_pipeline_t p;
    rssi_pipeline_cfg_t c = base_cfg();
    c.window_max_ms = 30;
    rssi_pipeline_begin(&p, &c);

    /* The AGC-refresh dip: one sample 10 dB low inside a 30 ms window must
     * not pull the output down. -60 dBm maps to 170, -70 to 128. */
    rssi_pipeline_on_sample(&p, -60.0f, 100);
    check("peak-hold base level", rssi_pipeline_counts(&p) == 170u);
    rssi_pipeline_on_sample(&p, -70.0f, 101);   /* dip */
    check("peak-hold hides a single dip", rssi_pipeline_counts(&p) == 170u);
    /* Once the strong sample has aged out of the window, the dip shows. */
    rssi_pipeline_on_sample(&p, -70.0f, 131);
    check("peak-hold releases after the window", rssi_pipeline_counts(&p) == 128u);

    /* Median-of-3 kills a single-sample glitch. */
    rssi_pipeline_cfg_t m = base_cfg();
    m.use_median3 = true;
    rssi_pipeline_t q;
    rssi_pipeline_begin(&q, &m);
    rssi_pipeline_on_sample(&q, -60.0f, 200);
    rssi_pipeline_on_sample(&q, -60.0f, 201);
    rssi_pipeline_on_sample(&q, -90.0f, 202);   /* glitch */
    check("median3 rejects a single glitch", rssi_pipeline_counts(&q) == 170u);
}

static void test_knee(void)
{
    rssi_pipeline_t p;
    rssi_pipeline_cfg_t c = base_cfg();
    c.knee_db = -55.0f;
    c.knee_ratio = 4.0f;
    rssi_pipeline_begin(&p, &c);

    /* The knee compresses the region above it: below the knee the scale keeps
     * full resolution, above it a 10 dB swing costs only 10/ratio dB, so a
     * saturating signal cannot slam the output to the top with small changes.
     * dbHi still maps to exactly 255 whatever the ratio. */
    rssi_pipeline_on_sample(&p, -55.0f, 100);
    check("knee: at the knee the output is 235", rssi_pipeline_counts(&p) == 235u);
    rssi_pipeline_on_sample(&p, -45.0f, 101);   /* 10 dB above the knee -> 13 counts */
    check("knee: above the knee the slope is 1.3 counts/dB", rssi_pipeline_counts(&p) == 248u);
    rssi_pipeline_on_sample(&p, -65.0f, 102);   /* 10 dB below the knee -> 52 counts */
    check("knee: below the knee the slope is 5.2 counts/dB", rssi_pipeline_counts(&p) == 183u);
    rssi_pipeline_on_sample(&p, -40.0f, 103);   /* db_hi still reads 255 */
    check("knee: db_hi still reads 255", rssi_pipeline_counts(&p) == 255u);
}

static void test_settle_and_stall(void)
{
    rssi_pipeline_t p;
    rssi_pipeline_cfg_t c = base_cfg();
    rssi_pipeline_begin(&p, &c);

    rssi_pipeline_on_sample(&p, -60.0f, 100);
    uint8_t held = rssi_pipeline_counts(&p);
    rssi_pipeline_on_tune(&p, 200);
    check("retune blanks the output", !rssi_pipeline_valid(&p));
    rssi_pipeline_on_sample(&p, -40.0f, 210);   /* inside settle: hold */
    check("retune holds the last good value", rssi_pipeline_counts(&p) == held);
    rssi_pipeline_on_sample(&p, -40.0f, 240);   /* settle_ms = 35 -> open at 235 */
    check("settle window ends", rssi_pipeline_valid(&p) && rssi_pipeline_counts(&p) == 255u);

    rssi_pipeline_tick(&p, 441);                /* stall_ms = 200 */
    check("stall detected with no samples", !rssi_pipeline_valid(&p) && rssi_pipeline_stalled(&p));
}

static void test_signal_present(void)
{
    rssi_pipeline_t p;
    rssi_pipeline_cfg_t c = base_cfg();
    rssi_pipeline_begin(&p, &c);

    /* Learn the quiet level. Values must differ so the freshness rule stays
     * satisfied (a frozen value means the detector is not measuring). */
    for (uint32_t t = 100; t < 600; ++t) {
        rssi_pipeline_on_sample(&p, (t & 1u) ? -90.0f : -91.0f, t);
    }
    check_near("baseline learns the quiet level", rssi_pipeline_baseline_db(&p), -90.5f, 1.0f);
    check("quiet level is not a pass", !rssi_pipeline_signal_present(&p));

    /* A pass: strong readings must not lift the baseline. */
    for (uint32_t t = 600; t < 700; ++t) {
        rssi_pipeline_on_sample(&p, (t & 1u) ? -45.0f : -46.0f, t);
    }
    check("pass trips the margin", rssi_pipeline_signal_present(&p));
    check_near("a pass does not lift the baseline", rssi_pipeline_baseline_db(&p), -90.5f, 1.0f);

    /* Hysteresis: it must clear only below margin - hysteresis. */
    for (uint32_t t = 700; t < 800; ++t) {
        rssi_pipeline_on_sample(&p, (t & 1u) ? -80.0f : -81.0f, t);   /* between margin and margin-hyst */
    }
    check("stays present inside the hysteresis band", rssi_pipeline_signal_present(&p));
    for (uint32_t t = 800; t < 900; ++t) {
        rssi_pipeline_on_sample(&p, (t & 1u) ? -88.0f : -89.0f, t);   /* below margin - hyst */
    }
    check("clears below the hysteresis point", !rssi_pipeline_signal_present(&p));
}

static void test_stale_value(void)
{
    rssi_pipeline_t p;
    rssi_pipeline_cfg_t c = base_cfg();
    rssi_pipeline_begin(&p, &c);

    for (uint32_t t = 100; t < 600; ++t) {
        rssi_pipeline_on_sample(&p, (t & 1u) ? -90.0f : -91.0f, t);
    }
    /* A stale high value from a finished pass: the register stops moving, so
     * after fresh_ms it must not keep a lap timer reporting a drone in range. */
    for (uint32_t t = 600; t < 700; ++t) {
        rssi_pipeline_on_sample(&p, -45.0f, t);
    }
    check("stale high value still counts while fresh", rssi_pipeline_signal_present(&p));
    for (uint32_t t = 700; t < 1700; ++t) {
        rssi_pipeline_on_sample(&p, -45.0f, t);
    }
    check("frozen value is not trusted as a pass", !rssi_pipeline_fresh(&p));
    check("stale high value clears the pass", !rssi_pipeline_signal_present(&p));
}

/* ---- RX5808 bus ---- */

static bool g_test_data;
static uint32_t g_test_reg[16];
static uint32_t test_provider(uint8_t addr) { return g_test_reg[addr & 0x0Fu]; }
static uint32_t g_last_word;
static bool g_last_is_write;
static void test_frame_cb(uint32_t bits25, bool is_write, uint8_t addr)
{
    g_last_word = bits25;
    g_last_is_write = is_write;
    (void)addr;
}
static void test_release(void) {}
static uint32_t g_driven_bits;
static int g_driven_count;
static void test_drive(bool level)
{
    if (level) g_driven_bits |= 1u << g_driven_count;
    ++g_driven_count;
}
static bool test_sample(void) { return g_test_data; }

static void test_bus(void)
{
    rx5808_bus_t bus;
    memset(&bus, 0, sizeof(bus));
    bus.read_provider = test_provider;
    bus.frame_cb = test_frame_cb;
    bus.data_release = test_release;
    bus.data_drive = test_drive;
    bus.data_sample = test_sample;

    /* Write frame to register 1 (synth), data = synth reg for 5800 MHz. */
    uint32_t word = rx5808_build_word(RX5808_REG_SYNTH_RF, true,
                                      rx5808_mhz_to_synth_reg(5800u));
    check("build/parse round trip", rx5808_parse_word(word).address == RX5808_REG_SYNTH_RF &&
          rx5808_parse_word(word).write &&
          rx5808_parse_word(word).data == (word >> 5));

    rx5808_bus_sel_change(&bus, false);           /* frame start */
    g_test_data = false;
    for (int i = 0; i < 25; ++i) {
        g_test_data = (word >> i) & 1u;
        rx5808_bus_clk_change(&bus, true);        /* rising edge: host sends */
        rx5808_bus_clk_change(&bus, false);
    }
    check("write frame completes", rx5808_bus_complete(&bus));
    g_last_word = 0;
    rx5808_bus_sel_change(&bus, true);            /* frame end -> consumer */
    check("write frame delivered", g_last_word == word && g_last_is_write);

    /* A short frame must not reach the consumer. */
    g_last_word = 0;
    rx5808_bus_sel_change(&bus, false);
    for (int i = 0; i < 10; ++i) {
        rx5808_bus_clk_change(&bus, true);
        rx5808_bus_clk_change(&bus, false);
    }
    rx5808_bus_sel_change(&bus, true);
    check("short frame is not delivered", g_last_word == 0);

    /* Read frame: host sends 5 header bits, then we send 20. */
    memset(&bus, 0, sizeof(bus));
    bus.read_provider = test_provider;
    bus.frame_cb = test_frame_cb;
    bus.data_release = test_release;
    bus.data_drive = test_drive;
    bus.data_sample = test_sample;
    g_test_reg[RX5808_REG_EXT_STATUS] = rx5808_build_status_word(200, true, true, 3, true);
    g_driven_bits = 0; g_driven_count = 0;
    rx5808_bus_sel_change(&bus, false);
    uint32_t hdr = rx5808_build_word(RX5808_REG_EXT_STATUS, false, 0);
    for (int i = 0; i < 5; ++i) {
        g_test_data = (hdr >> i) & 1u;
        rx5808_bus_clk_change(&bus, true);
        rx5808_bus_clk_change(&bus, false);
    }
    check("host drives the header only on a read", !rx5808_bus_host_drives(&bus));
    /* The first data bit goes out on the falling edge of the 5th header clock,
     * the rest on the following falling edges. */
    check("first data bit driven during the header edge", g_driven_count == 1 &&
          (g_driven_bits & 1u) == (g_test_reg[RX5808_REG_EXT_STATUS] & 1u));
    for (int i = 0; i < 19; ++i) {
        rx5808_bus_clk_change(&bus, false);
    }
    check("read returns the register value", g_driven_bits == g_test_reg[RX5808_REG_EXT_STATUS]);
    check("signal-present bit is D10", (g_driven_bits >> 10) & 1u);
    check("read frame completes after 20 data bits", rx5808_bus_complete(&bus));

    /* pop_bit is the same 20 bits for a host that samples DATA itself. */
    memset(&bus, 0, sizeof(bus));
    bus.read_provider = test_provider;
    g_test_reg[RX5808_REG_EXT_STATUS] = rx5808_build_status_word(200, true, true, 3, true);
    uint32_t hdr2 = rx5808_build_word(RX5808_REG_EXT_STATUS, false, 0);
    for (int i = 0; i < 5; ++i) {
        rx5808_bus_push_bit(&bus, (hdr2 >> i) & 1u);
    }
    uint32_t got = 0;
    for (int i = 0; i < 20; ++i) {
        if (rx5808_bus_pop_bit(&bus)) got |= 1u << i;
    }
    check("pop_bit path returns the same register value", got == g_test_reg[RX5808_REG_EXT_STATUS]);

    /* Frequency register math. */
    check("synth reg round trip for 5645 (R4)",
          rx5808_synth_reg_to_mhz(rx5808_mhz_to_synth_reg(5645u)) == 5645u);
    check("synth reg round trip for 5800",
          rx5808_synth_reg_to_mhz(rx5808_mhz_to_synth_reg(5800u)) == 5799u);
    check("power-down word is all ones",
          rx5808_parse_word(rx5808_build_word(RX5808_REG_POWER, true, 0xFFFFFu)).data == 0xFFFFFu);
}

int main(void)
{
    test_calibration();
    test_peak_hold_and_median();
    test_knee();
    test_settle_and_stall();
    test_signal_present();
    test_stale_value();
    test_bus();
    printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}

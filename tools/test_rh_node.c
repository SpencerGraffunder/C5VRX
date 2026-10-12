/*
 * Host unit test for the RotorHazard node protocol layer (main/rh_node.c).
 * Run:
 *   cc -I main tools/test_rh_node.c main/rh_node.c -o /tmp/test_rh_node && /tmp/test_rh_node
 *
 * It replays the exact byte sequences RotorHazard's serial_node.py sends and
 * checks the responses byte-for-byte against the server's own expectations
 * (payload length, checksum = sum of payload bytes, field layout).
 */
#include <stdio.h>
#include <string.h>
#include "rh_node.h"

static int fails = 0;
static int checks = 0;

static void check(const char *what, int ok)
{
    checks++;
    if (!ok) {
        fails++;
        printf("FAIL %s\n", what);
    } else {
        printf("ok   %s\n", what);
    }
}

static uint8_t csum(const uint8_t *d, int len)
{
    uint32_t s = 0;
    for (int i = 0; i < len; i++) s += d[i];
    return (uint8_t)(s & 0xFF);
}

/* Send a read command, get the response into resp (payload + checksum). */
static int read_cmd(rh_node_t *n, uint8_t cmd, uint8_t *resp)
{
    int len = rh_node_handle_byte(n, cmd, resp);
    if (len > 0 && csum(resp, len - 1) != resp[len - 1]) {
        printf("FAIL checksum for cmd 0x%02X\n", cmd);
        fails++;
    }
    return len;
}

/* Send a write command with payload; the checksum is over the payload only. */
static void write_cmd(rh_node_t *n, uint8_t cmd, const uint8_t *payload, int plen)
{
    uint8_t out[32];
    rh_node_handle_byte(n, cmd, out);
    for (int i = 0; i < plen; i++) rh_node_handle_byte(n, payload[i], out);
    uint8_t c = csum(payload, plen);
    int len = rh_node_handle_byte(n, c, out);
    if (len != 0) {
        printf("FAIL write cmd 0x%02X produced a response\n", cmd);
        fails++;
    }
}

static uint16_t be16(const uint8_t *d) { return (uint16_t)((d[0] << 8) | d[1]); }

int main(void)
{
    rh_node_t n;
    rh_node_begin(&n, 5800, 120, 100, 1000);
    n.fw_version = "C5VRX_RSSI_1.0";
    n.fw_build_date = "Oct  4 2026";
    n.fw_build_time = "15:30:00";
    n.fw_proctype = "ESP32C5";
    uint8_t resp[32];

    /* --- Discovery, exactly as serial_node.py discover() does --- */
    int len = read_cmd(&n, RH_READ_REVISION_CODE, resp);
    check("revision code length is 2+1", len == 3);
    check("revision high byte is 0x25", resp[0] == 0x25);
    check("api level >= 32", (resp[1] & 0xFF) >= 32);

    len = read_cmd(&n, RH_READ_MULTINODE_COUNT, resp);
    check("multinode length", len == 2);
    check("multinode count is 1", resp[0] == 1);

    len = read_cmd(&n, RH_READ_FW_VERSION, resp);
    check("fw version block is 16+1", len == 17);
    check("fw version text round-trips", strcmp((char *)resp, "C5VRX_RSSI_1.0") == 0);

    len = read_cmd(&n, RH_READ_FW_PROCTYPE, resp);
    check("fw proctype block", len == 17 && strcmp((char *)resp, "ESP32C5") == 0);

    len = read_cmd(&n, RH_READ_FW_BUILDDATE, resp);
    check("fw build date block", len == 17 && strcmp((char *)resp, "Oct  4 2026") == 0);

    /* --- Frequency write (2-byte payload) --- */
    uint8_t f[2] = { (uint8_t)(5660 >> 8), (uint8_t)(5660 & 0xFF) };
    write_cmd(&n, RH_WRITE_FREQUENCY, f, 2);
    check("frequency write applied", n.freq_mhz == 5660 && n.pending_freq_mhz == 5660);
    check("frequency pending for the tuner", n.freq_pending);

    uint8_t bad[2] = { 0x0F, 0xA0 };   /* 4000 MHz, outside the C5 window */
    write_cmd(&n, RH_WRITE_FREQUENCY, bad, 2);
    check("out-of-window frequency ignored", n.freq_mhz == 5660);

    len = read_cmd(&n, RH_READ_FREQUENCY, resp);
    check("frequency read length", len == 3);
    check("frequency read back", be16(resp) == 5660);

    /* Bad checksum must be rejected, not applied. */
    uint8_t p[2] = { 0x16, 0x1C };     /* 5660 again */
    rh_node_handle_byte(&n, RH_WRITE_FREQUENCY, resp);
    for (int i = 0; i < 2; i++) rh_node_handle_byte(&n, p[i], resp);
    rh_node_handle_byte(&n, 0x00, resp);   /* deliberately wrong checksum */
    check("bad checksum ignored", n.freq_mhz == 5660);

    /* --- Thresholds --- */
    uint8_t e1[1] = { 150 };
    write_cmd(&n, RH_WRITE_ENTER_AT_LEVEL, e1, 1);
    uint8_t e2[1] = { 120 };
    write_cmd(&n, RH_WRITE_EXIT_AT_LEVEL, e2, 1);
    len = read_cmd(&n, RH_READ_ENTER_AT_LEVEL, resp);
    check("enter level read back", len == 2 && resp[0] == 150);
    len = read_cmd(&n, RH_READ_EXIT_AT_LEVEL, resp);
    check("exit level read back", len == 2 && resp[0] == 120);

    /* --- Quiet baseline: no crossing, RSSI in the range the server accepts --- */
    for (int i = 0; i < 20; i++) rh_node_on_sample(&n, 5, 1000 + i);
    len = read_cmd(&n, RH_READ_LAP_PASS_STATS, resp);
    check("pass stats length is 8+1", len == 9);
    check("no lap yet", resp[0] == 0);
    check("quiet rssi reported", resp[3] == 5);
    len = read_cmd(&n, RH_READ_LAP_EXTREMUMS, resp);
    check("quiet: nothing queued yet (extremums queue on direction change)",
          resp[3] == 0 && be16(resp + 4) == 0);
    check("quiet: not crossing", !(resp[0] & RH_LAPSTATS_FLAG_CROSSING));

    /* --- A pass: rise above enter, fall below exit --- */
    uint32_t t = 1100;
    for (int v = 5; v <= 200; v += 15) rh_node_on_sample(&n, (uint8_t)v, t += 10);
    len = read_cmd(&n, RH_READ_LAP_PASS_STATS, resp);
    check("node peak tracked", resp[4] == 200);
    len = read_cmd(&n, RH_READ_LAP_EXTREMUMS, resp);
    check("crossing flag set while high", (resp[0] & RH_LAPSTATS_FLAG_CROSSING) != 0);
    check("peak still in progress while rising", resp[3] == 0);

    for (int v = 200; v >= 60; v -= 20) rh_node_on_sample(&n, (uint8_t)v, t += 10);
    len = read_cmd(&n, RH_READ_LAP_PASS_STATS, resp);
    check("lap recorded after exit", resp[0] == 1);
    check("pass peak kept for the lap", resp[5] == 200);
    check("ms since lap is the peak age", be16(resp + 1) > 0);

    /* Drain the extremum queue. An extremum only enters history when its run
     * ends, so the peak is queued by the first downward step and the nadir by
     * the next upward one - a still-falling nadir is pending, not history. */
    int saw_peak = 0, saw_nadir = 0, peak_first = 0, drained = 0;
    for (int i = 0; i < 8; i++) {
        len = read_cmd(&n, RH_READ_LAP_EXTREMUMS, resp);
        if (resp[3] == 0) break;
        drained++;
        if (resp[3] == 200) { saw_peak = 1; if (!saw_nadir) peak_first = 1; }
        if (resp[3] == 60) saw_nadir = 1;
    }
    check("pass peak reached the history", saw_peak);
    check("pending nadir is not history yet", drained == 1);
    for (int v = 60; v <= 80; v += 20) rh_node_on_sample(&n, (uint8_t)v, t += 10);
    len = read_cmd(&n, RH_READ_LAP_EXTREMUMS, resp);
    check("nadir enters history when the run ends", resp[3] == 60);
    check("peak came out before nadir", peak_first);
    len = read_cmd(&n, RH_READ_LAP_EXTREMUMS, resp);
    check("queue drains to empty", resp[3] == 0);
    len = read_cmd(&n, RH_READ_LAP_EXTREMUMS, resp);
    check("crossing cleared after exit", !(resp[0] & RH_LAPSTATS_FLAG_CROSSING));

    /* --- Min-lap guard: a second fast pass must not double-count --- */
    for (int v = 60; v <= 210; v += 20) rh_node_on_sample(&n, (uint8_t)v, t += 5);
    for (int v = 210; v >= 60; v -= 20) rh_node_on_sample(&n, (uint8_t)v, t += 5);
    len = read_cmd(&n, RH_READ_LAP_PASS_STATS, resp);
    check("second pass inside min-lap window rejected", resp[0] == 1);

    /* Now let enough time pass and run a proper pass. */
    for (int i = 0; i < 1200; i++) rh_node_on_sample(&n, 60, t += 1);
    for (int v = 60; v <= 220; v += 20) rh_node_on_sample(&n, (uint8_t)v, t += 5);
    for (int v = 220; v >= 60; v -= 20) rh_node_on_sample(&n, (uint8_t)v, t += 5);
    len = read_cmd(&n, RH_READ_LAP_PASS_STATS, resp);
    check("lap counted after min-lap guard clears", resp[0] == 2);

    /* --- Force end crossing --- */
    uint8_t fe[1] = { 0 };
    write_cmd(&n, RH_FORCE_END_CROSSING, fe, 1);
    len = read_cmd(&n, RH_READ_LAP_EXTREMUMS, resp);
    check("force end crossing clears the flag", !(resp[0] & RH_LAPSTATS_FLAG_CROSSING));

    /* --- Jump to bootloader is an action, not a response --- */
    uint8_t jb[1] = { 0 };
    write_cmd(&n, RH_JUMP_TO_BOOTLOADER, jb, 1);
    check("jump-to-bootloader flagged", n.bootloader_pending);

    /* --- Garbage must not produce a response (baud mismatch safety) --- */
    int g = rh_node_handle_byte(&n, 'H', resp);
    check("non-command byte ignored", g == 0);

    /* --- Clamping to the range the server accepts --- */
    check("0 clamps to 1", rh_node_report_rssi(0) == 1);
    check("255 clamps to 254", rh_node_report_rssi(255) == 254);
    check("mid value passes through", rh_node_report_rssi(128) == 128);

    /* --- 16-bit reads the server frames with get_value_16 --- */
    len = read_cmd(&n, RH_READ_TIME_MILLIS, resp);
    check("time millis is 2 payload bytes", len == 3);
    len = read_cmd(&n, RH_READ_LAP_STATS, resp);
    check("legacy lap stats is 8 payload bytes", len == 9);
    len = read_cmd(&n, RH_READ_FREQUENCY, resp);
    check("frequency is 2 payload bytes", len == 3);
    len = read_cmd(&n, RH_READ_RHFEAT_FLAGS, resp);
    check("feature flags are 2 payload bytes", len == 3);
    len = read_cmd(&n, RH_READ_REVISION_CODE, resp);
    check("revision code is 2 payload bytes", len == 3);
    len = read_cmd(&n, RH_READ_FW_VERSION, resp);
    check("fw version block is 16 payload bytes", len == 17);

    printf("\n%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}

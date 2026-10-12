/*
 * rh_node.c - RotorHazard USB serial node protocol.
 *
 * See rh_node.h for the contract. Kept free of ESP-IDF calls so it can be
 * unit-tested on the host (tools/test_rh_node.c) and driven by any timestamp
 * source.
 */
#include "rh_node.h"
#include <string.h>

static uint8_t sum_checksum(const uint8_t *data, uint8_t len)
{
    uint32_t s = 0;
    for (uint8_t i = 0u; i < len; ++i) {
        s += data[i];
    }
    return (uint8_t)(s & 0xFFu);
}

/* The server rejects RSSI 0 and 255 (Node.is_valid_rssi), so report 1..254. */
uint8_t rh_node_report_rssi(uint8_t v)
{
    if (v == 0u) {
        return 1u;
    }
    if (v >= 255u) {
        return 254u;
    }
    return v;
}

uint32_t rh_node_ms_since_lap(const rh_node_t *n)
{
    if (n->last_lap_ms == 0u || n->now_ms < n->last_lap_ms) {
        return 0u;
    }
    uint32_t d = n->now_ms - n->last_lap_ms;
    return d > 0xFFFFu ? 0xFFFFu : d;
}

uint32_t rh_node_ms_since_extremum(const rh_node_t *n, const rh_extremum_t *e)
{
    if (n->now_ms < e->first_time) {
        return 0u;
    }
    uint32_t d = n->now_ms - e->first_time;
    return d > 0xFFFFu ? 0xFFFFu : d;
}

void rh_node_reset(rh_node_t *n)
{
    n->lap_id = 0;
    n->last_lap_ms = 0;
    n->current = 0;
    n->node_peak = 0;
    n->pass_peak = 0;
    n->pass_nadir = 255;
    n->node_nadir = 255;
    n->crossing = false;
    n->crossing_start = 0;
    n->last_rssi = 0;
    n->rssi_change = 0;
    n->capture_peak = 0;
    n->capture_peak_time = 0;
    n->laps_recorded = 0;
    n->peak_write = 0;
    n->peak_read = 0;
    n->nadir_write = 0;
    n->nadir_read = 0;
    memset(&n->cur_peak, 0, sizeof(n->cur_peak));
    memset(&n->cur_nadir, 0, sizeof(n->cur_nadir));
    n->cur_nadir.rssi = 255;
}

void rh_node_begin(rh_node_t *n, uint16_t freq_mhz, uint8_t enter_at,
                   uint8_t exit_at, uint32_t min_lap_ms)
{
    memset(n, 0, sizeof(*n));
    n->freq_mhz = freq_mhz;
    n->enter_at = enter_at;
    n->exit_at = exit_at;
    n->min_lap_ms = min_lap_ms;
    n->node_index = 0;
    n->slot_index = 0;
    rh_node_reset(n);
}

static void buffer_extremum(rh_extremum_t *buf, uint16_t *write, uint16_t *read,
                            const rh_extremum_t *e)
{
    if (!e->valid) {
        return;
    }
    /* Overwriting the oldest entry when the queue is full matches the
     * reference node: history is best-effort, a lap is never lost. */
    buf[*write & (RH_EXTREMUM_BUFFER_SIZE - 1u)] = *e;
    *write = (uint16_t)(*write + 1u);
    if (*write - *read > RH_EXTREMUM_BUFFER_SIZE) {
        *read = (uint16_t)(*write - RH_EXTREMUM_BUFFER_SIZE);
    }
}

static void process_extremums(rh_node_t *n, uint8_t rssi, uint32_t now_ms)
{
    int change = (int)rssi - (int)n->last_rssi;

    /* An extremum is a run in one direction: it is extended while the value
     * keeps moving that way and only queued when the direction flips. The
     * reference node does the same, which is why the server sees one peak per
     * pass rather than one entry per sample. */
    if (change > 0) {
        if (rssi > n->cur_peak.rssi) {
            n->cur_peak.rssi = rssi;
            n->cur_peak.first_time = now_ms;
            n->cur_peak.duration = 0;
            n->cur_peak.valid = true;
        }
        /* A plateau at the extremum counts as the end of the run: the run is
         * complete as soon as the value starts moving the other way, so the
         * comparison is inclusive (a strict-sign test loses a peak that sits
         * flat for one sample). */
        if (n->rssi_change <= 0 && n->cur_nadir.valid) {
            buffer_extremum(n->nadir_buf, &n->nadir_write, &n->nadir_read, &n->cur_nadir);
        }
    } else if (change < 0) {
        if (rssi < n->cur_nadir.rssi) {
            n->cur_nadir.rssi = rssi;
            n->cur_nadir.first_time = now_ms;
            n->cur_nadir.duration = 0;
            n->cur_nadir.valid = true;
        }
        if (n->rssi_change >= 0 && n->cur_peak.valid) {
            buffer_extremum(n->peak_buf, &n->peak_write, &n->peak_read, &n->cur_peak);
        }
    }

    if (n->cur_peak.valid) {
        uint32_t d = now_ms - n->cur_peak.first_time;
        n->cur_peak.duration = (uint16_t)(d > 0xFFFFu ? 0xFFFFu : d);
        if (n->cur_peak.duration == 0xFFFFu) {
            buffer_extremum(n->peak_buf, &n->peak_write, &n->peak_read, &n->cur_peak);
            n->cur_peak.rssi = rssi;
            n->cur_peak.first_time = now_ms;
            n->cur_peak.duration = 0;
            n->cur_peak.valid = true;
        }
    }
    if (n->cur_nadir.valid) {
        uint32_t d = now_ms - n->cur_nadir.first_time;
        n->cur_nadir.duration = (uint16_t)(d > 0xFFFFu ? 0xFFFFu : d);
        if (n->cur_nadir.duration == 0xFFFFu) {
            buffer_extremum(n->nadir_buf, &n->nadir_write, &n->nadir_read, &n->cur_nadir);
            n->cur_nadir.rssi = rssi;
            n->cur_nadir.first_time = now_ms;
            n->cur_nadir.duration = 0;
            n->cur_nadir.valid = true;
        }
    }

    n->last_rssi = rssi;
    if (change > 127) {
        n->rssi_change = 127;
    } else if (change < -127) {
        n->rssi_change = -127;
    } else {
        n->rssi_change = (int8_t)change;
    }
}

void rh_node_on_sample(rh_node_t *n, uint8_t counts, uint32_t now_ms)
{
    n->now_ms = now_ms;
    n->current = counts;

    if (counts < n->node_nadir) {
        n->node_nadir = counts;
    }
    if (counts < n->pass_nadir) {
        n->pass_nadir = counts;
    }

    process_extremums(n, counts, now_ms);

    /* A peak only counts if enough time has passed since the last lap, so the
     * same pass cannot be counted twice. */
    bool can_capture = n->min_lap_ms == 0u || n->last_lap_ms == 0u ||
                       (now_ms - n->last_lap_ms) >= n->min_lap_ms;

    if (can_capture && counts >= n->enter_at && counts > n->capture_peak) {
        n->capture_peak = counts;
        n->capture_peak_time = now_ms;
        if (counts > n->node_peak) {
            n->node_peak = counts;
        }
    }

    bool peak_captured = counts < n->capture_peak && counts < n->exit_at;

    bool was_crossing = n->crossing;
    n->crossing = can_capture && counts >= n->enter_at;
    if (was_crossing != n->crossing) {
        if (n->crossing) {
            n->crossing_start = now_ms;
        }
    }

    if (peak_captured && n->capture_peak > 0u) {
        uint32_t since_last = (n->last_lap_ms != 0u) ? (now_ms - n->last_lap_ms)
                                                     : 0xFFFFFFFFu;
        if (since_last >= n->min_lap_ms) {
            /* The lap timestamp is the peak moment, not the moment the pass
             * ends - the server derives lap time from ms-since-lap, so this
             * is what makes lap times land on the gate rather than on exit. */
            n->last_lap_ms = n->capture_peak_time;
            n->pass_peak = n->capture_peak;
            n->lap_id = (uint8_t)(n->lap_id + 1u);
            n->laps_recorded++;
            n->capture_peak = 0;
            n->capture_peak_time = 0;
            n->node_peak = 0;
            n->pass_nadir = 255;
        }
    }
}

bool rh_node_is_command(uint8_t b)
{
    switch (b) {
    case RH_READ_ADDRESS:
    case RH_READ_FREQUENCY:
    case RH_READ_LAP_STATS:
    case RH_READ_LAP_PASS_STATS:
    case RH_READ_LAP_EXTREMUMS:
    case RH_READ_RHFEAT_FLAGS:
    case RH_READ_REVISION_CODE:
    case RH_READ_NODE_RSSI_PEAK:
    case RH_READ_NODE_RSSI_NADIR:
    case RH_READ_ENTER_AT_LEVEL:
    case RH_READ_EXIT_AT_LEVEL:
    case RH_READ_TIME_MILLIS:
    case RH_READ_MULTINODE_COUNT:
    case RH_READ_CURNODE_INDEX:
    case RH_READ_NODE_SLOTIDX:
    case RH_READ_FW_VERSION:
    case RH_READ_FW_BUILDDATE:
    case RH_READ_FW_BUILDTIME:
    case RH_READ_FW_PROCTYPE:
    case RH_WRITE_FREQUENCY:
    case RH_WRITE_ENTER_AT_LEVEL:
    case RH_WRITE_EXIT_AT_LEVEL:
    case RH_SEND_STATUS_MESSAGE:
    case RH_FORCE_END_CROSSING:
    case RH_WRITE_CURNODE_INDEX:
    case RH_JUMP_TO_BOOTLOADER:
        return true;
    default:
        return false;
    }
}

static uint8_t payload_size(uint8_t cmd)
{
    switch (cmd) {
    case RH_WRITE_FREQUENCY:
    case RH_SEND_STATUS_MESSAGE:
        return 2u;
    case RH_WRITE_ENTER_AT_LEVEL:
    case RH_WRITE_EXIT_AT_LEVEL:
    case RH_FORCE_END_CROSSING:
    case RH_WRITE_CURNODE_INDEX:
    case RH_JUMP_TO_BOOTLOADER:
        return 1u;
    default:
        return 0u;
    }
}

static void write_text_block(uint8_t *out, uint8_t *len, const char *s)
{
    size_t l = s ? strlen(s) : 0u;
    for (uint8_t i = 0u; i < RH_FW_TEXT_BLOCK; ++i) {
        out[(*len)++] = (uint8_t)(i < l ? (unsigned char)s[i] : 0);
    }
}

static int build_response(rh_node_t *n, uint8_t *out)
{
    uint8_t len = 0;
    switch (n->cmd) {
    case RH_READ_ADDRESS:
        out[len++] = 0x08u;   /* nominal node address */
        break;
    case RH_READ_FREQUENCY:
        out[len++] = (uint8_t)(n->freq_mhz >> 8);
        out[len++] = (uint8_t)(n->freq_mhz & 0xFFu);
        break;
    case RH_READ_LAP_PASS_STATS:
        /* 8 bytes: lap, ms since lap (16), current, node peak, pass peak,
         * loop time (16). */
        out[len++] = n->lap_id;
        uint32_t age = rh_node_ms_since_lap(n);
        out[len++] = (uint8_t)((age >> 8) & 0xFFu);
        out[len++] = (uint8_t)(age & 0xFFu);
        out[len++] = rh_node_report_rssi(n->current);
        out[len++] = rh_node_report_rssi(n->node_peak);
        out[len++] = rh_node_report_rssi(n->pass_peak);
        out[len++] = 1u;      /* loop time ms: one FreeRTOS tick */
        out[len++] = 0u;
        break;
    case RH_READ_LAP_EXTREMUMS: {
        /* 8 bytes: flags, pass nadir, node nadir, extremum rssi, first time
         * (16), duration (16). Oldest pending extremum goes first, matching
         * the server's history handling. */
        bool has_peak = n->peak_write != n->peak_read;
        bool has_nadir = n->nadir_write != n->nadir_read;
        bool send_peak = false;
        uint8_t flags = n->crossing ? RH_LAPSTATS_FLAG_CROSSING : 0u;

        if (has_peak && !has_nadir) {
            send_peak = true;
        } else if (has_peak && has_nadir) {
            const rh_extremum_t *p = &n->peak_buf[n->peak_read & (RH_EXTREMUM_BUFFER_SIZE - 1u)];
            const rh_extremum_t *d = &n->nadir_buf[n->nadir_read & (RH_EXTREMUM_BUFFER_SIZE - 1u)];
            if (p->first_time < d->first_time) {
                send_peak = true;
            }
        }
        if (send_peak) {
            flags |= RH_LAPSTATS_FLAG_PEAK;
        }

        out[len++] = flags;
        out[len++] = rh_node_report_rssi(n->pass_nadir);
        out[len++] = rh_node_report_rssi(n->node_nadir);

        const rh_extremum_t *e = NULL;
        if (send_peak && has_peak) {
            e = &n->peak_buf[n->peak_read & (RH_EXTREMUM_BUFFER_SIZE - 1u)];
            n->peak_read = (uint16_t)(n->peak_read + 1u);
        } else if (!send_peak && has_nadir) {
            e = &n->nadir_buf[n->nadir_read & (RH_EXTREMUM_BUFFER_SIZE - 1u)];
            n->nadir_read = (uint16_t)(n->nadir_read + 1u);
        }

        if (e) {
            uint32_t age = rh_node_ms_since_extremum(n, e);
            out[len++] = rh_node_report_rssi(e->rssi);
            out[len++] = (uint8_t)((age >> 8) & 0xFFu);
            out[len++] = (uint8_t)(age & 0xFFu);
            out[len++] = (uint8_t)((e->duration >> 8) & 0xFFu);
            out[len++] = (uint8_t)(e->duration & 0xFFu);
        } else {
            out[len++] = 0u;
            out[len++] = 0u;
            out[len++] = 0u;
            out[len++] = 0u;
            out[len++] = 0u;
        }
        break;
    }
    case RH_READ_TIME_MILLIS:
        /* The server reads this as a 16-bit value (get_value_16), so the low
         * half of millis() is what it can see. */
        out[len++] = (uint8_t)((n->now_ms >> 8) & 0xFFu);
        out[len++] = (uint8_t)(n->now_ms & 0xFFu);
        break;
    case RH_READ_LAP_STATS:
        /* Only reachable for a server talking to api_level < 32. Same 8-byte
         * block as READ_LAP_PASS_STATS so a mixed-version server still frames. */
        out[len++] = n->lap_id;
        uint32_t lap_age = rh_node_ms_since_lap(n);
        out[len++] = (uint8_t)((lap_age >> 8) & 0xFFu);
        out[len++] = (uint8_t)(lap_age & 0xFFu);
        out[len++] = rh_node_report_rssi(n->current);
        out[len++] = rh_node_report_rssi(n->node_peak);
        out[len++] = rh_node_report_rssi(n->pass_peak);
        out[len++] = 1u;
        out[len++] = 0u;
        break;
    case RH_READ_RHFEAT_FLAGS:
        out[len++] = (uint8_t)((RH_FEAT_FLAGS >> 8) & 0xFFu);
        out[len++] = (uint8_t)(RH_FEAT_FLAGS & 0xFFu);
        break;
    case RH_READ_REVISION_CODE:
        /* The server identifies a node from this: high byte 0x25, low byte
         * API level (must be >= 32 for the serial path). */
        out[len++] = 0x25u;
        out[len++] = (uint8_t)RH_NODE_API_LEVEL;
        break;
    case RH_READ_NODE_RSSI_PEAK:
        out[len++] = rh_node_report_rssi(n->node_peak);
        break;
    case RH_READ_NODE_RSSI_NADIR:
        out[len++] = rh_node_report_rssi(n->node_nadir);
        break;
    case RH_READ_ENTER_AT_LEVEL:
        out[len++] = n->enter_at;
        break;
    case RH_READ_EXIT_AT_LEVEL:
        out[len++] = n->exit_at;
        break;
    case RH_READ_MULTINODE_COUNT:
        out[len++] = 1u;
        break;
    case RH_READ_CURNODE_INDEX:
        out[len++] = n->node_index;
        break;
    case RH_READ_NODE_SLOTIDX:
        out[len++] = n->slot_index;
        break;
    case RH_READ_FW_VERSION:
        write_text_block( out, &len, n->fw_version);
        break;
    case RH_READ_FW_BUILDDATE:
        write_text_block( out, &len, n->fw_build_date);
        break;
    case RH_READ_FW_BUILDTIME:
        write_text_block( out, &len, n->fw_build_time);
        break;
    case RH_READ_FW_PROCTYPE:
        write_text_block( out, &len, n->fw_proctype);
        break;
    default:
        return 0;
    }

    uint8_t c = sum_checksum(out, len);
    out[len++] = c;
    return len;
}

int rh_node_handle_byte(rh_node_t *n, uint8_t b, uint8_t *out)
{
    if (n->size == 0u) {
        n->cmd = b;
        if (!rh_node_is_command(b)) {
            n->cmd = 0;
            n->errors++;
            return 0;
        }
        if (b > 0x50u) {
            uint8_t p = payload_size(b);
            n->size = (uint8_t)(p + 1u);
            n->index = 0u;
            return 0;
        }
        n->reads++;
        return build_response(n, out);
    }

    n->payload[n->index++] = b;
    if (n->index == n->size) {
        uint8_t p = (uint8_t)(n->size - 1u);
        if (sum_checksum(n->payload, p) == n->payload[p]) {
            switch (n->cmd) {
            case RH_WRITE_FREQUENCY: {
                uint16_t f = (uint16_t)((n->payload[0] << 8) | n->payload[1]);
                /* Out-of-range frequencies are ignored, as the server does. */
                if (f >= 5180u && f <= 5885u && f != n->freq_mhz) {
                    n->freq_mhz = f;
                    n->pending_freq_mhz = f;
                    n->freq_pending = true;
                    /* A retune invalidates the peak held for the current pass. */
                    n->capture_peak = 0;
                    n->capture_peak_time = 0;
                }
                break;
            }
            case RH_WRITE_ENTER_AT_LEVEL:
                n->enter_at = n->payload[0];
                break;
            case RH_WRITE_EXIT_AT_LEVEL:
                n->exit_at = n->payload[0];
                break;
            case RH_WRITE_CURNODE_INDEX:
                n->node_index = n->payload[0];
                break;
            case RH_FORCE_END_CROSSING:
                n->crossing = false;
                break;
            case RH_JUMP_TO_BOOTLOADER:
                n->bootloader_pending = true;
                break;
            case RH_SEND_STATUS_MESSAGE:
                /* Status messages from the server (button/shutdown/idle); the
                 * meter has no display to reflect them, so they are consumed. */
                break;
            default:
                break;
            }
        } else {
            n->errors++;
        }
        n->size = 0u;
        n->cmd = 0u;
    }
    return 0;
}

/*
 * rh_node.h - RotorHazard USB serial node protocol for the RSSI meter.
 *
 * Implements the RotorHazard node protocol exactly as the server expects it
 * (command bytes and response layouts taken from RotorHazard
 * src/interface/RHInterface.py and src/interface/serial_node.py), with the
 * crossing / peak / nadir / lap semantics mirrored from the NuclearCounter
 * TimingCore (src/timing_core.cpp), which is the proven working node.
 *
 * The protocol is strictly request/response: the server sends one command
 * byte (plus payload for writes) and reads exactly payload + checksum bytes.
 * Checksum is the sum of the payload bytes only, not the command byte.
 * Because of that, nothing else may be written to the port while node mode
 * is active - the meter's 1 kHz text stream must be off.
 *
 * RSSI on the wire is the RotorHazard 0..255 strength scale, not dBm. The
 * meter's calibrated pipeline already produces that scale, so the node layer
 * consumes pipeline counts directly. The server rejects readings of 0 and 255
 * (Node.is_valid_rssi: value > 0 and value < 255), so reported values are
 * clamped to 1..254.
 */
#ifndef RH_NODE_H
#define RH_NODE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Command bytes - must match RotorHazard RHInterface.py. */
#define RH_READ_ADDRESS          0x00
#define RH_READ_FREQUENCY        0x03
#define RH_READ_LAP_STATS        0x05
#define RH_READ_LAP_PASS_STATS   0x0D
#define RH_READ_LAP_EXTREMUMS    0x0E
#define RH_READ_RHFEAT_FLAGS     0x11
#define RH_READ_REVISION_CODE    0x22
#define RH_READ_NODE_RSSI_PEAK   0x23
#define RH_READ_NODE_RSSI_NADIR  0x24
#define RH_READ_ENTER_AT_LEVEL   0x31
#define RH_READ_EXIT_AT_LEVEL    0x32
#define RH_READ_TIME_MILLIS      0x33
#define RH_READ_MULTINODE_COUNT  0x39
#define RH_READ_CURNODE_INDEX    0x3A
#define RH_READ_NODE_SLOTIDX     0x3C
#define RH_READ_FW_VERSION       0x3D
#define RH_READ_FW_BUILDDATE     0x3E
#define RH_READ_FW_BUILDTIME     0x3F
#define RH_READ_FW_PROCTYPE      0x40

#define RH_WRITE_FREQUENCY       0x51
#define RH_WRITE_ENTER_AT_LEVEL  0x71
#define RH_WRITE_EXIT_AT_LEVEL   0x72
#define RH_WRITE_CURNODE_INDEX   0x7A
#define RH_SEND_STATUS_MESSAGE   0x75
#define RH_FORCE_END_CROSSING    0x78
#define RH_JUMP_TO_BOOTLOADER    0x7E

#define RH_NODE_API_LEVEL        35u
#define RH_FEAT_FLAGS            0x0000u   /* plain single-node ESP32, no IAP */
#define RH_FW_TEXT_BLOCK         16u
#define RH_LAPSTATS_FLAG_CROSSING 0x01u
#define RH_LAPSTATS_FLAG_PEAK    0x02u

/* Circular extremum buffers (power of two, as the reference node does). */
#define RH_EXTREMUM_BUFFER_SIZE  128u

typedef struct {
    uint8_t rssi;
    uint32_t first_time;
    uint16_t duration;
    bool valid;
} rh_extremum_t;

typedef struct {
    /* Settings (RotorHazard writes these with the WRITE_... commands). */
    uint16_t freq_mhz;
    uint8_t enter_at;
    uint8_t exit_at;
    uint32_t min_lap_ms;
    uint8_t node_index;
    uint8_t slot_index;

    /* Timing state. */
    uint8_t lap_id;
    uint32_t last_lap_ms;
    uint8_t current;
    uint8_t node_peak;      /* max since last lap */
    uint8_t pass_peak;      /* peak of the last recorded pass */
    uint8_t pass_nadir;     /* min since end of last pass */
    uint8_t node_nadir;     /* min since boot */
    bool crossing;
    uint32_t crossing_start;
    uint8_t last_rssi;
    int8_t rssi_change;
    uint8_t capture_peak;   /* running peak for lap detection */
    uint32_t capture_peak_time;
    uint32_t laps_recorded;
    uint32_t now_ms;        /* last sample timestamp, for age fields */

    /* Extremum queues (marshal-mode history). */
    rh_extremum_t peak_buf[RH_EXTREMUM_BUFFER_SIZE];
    rh_extremum_t nadir_buf[RH_EXTREMUM_BUFFER_SIZE];
    uint16_t peak_write, peak_read;
    uint16_t nadir_write, nadir_read;
    rh_extremum_t cur_peak, cur_nadir;

    /* Receive state machine. */
    uint8_t cmd;
    uint8_t payload[8];
    uint8_t size;    /* expected payload + checksum */
    uint8_t index;

    /* Actions the host firmware must pick up. */
    uint16_t pending_freq_mhz;
    bool freq_pending;
    bool bootloader_pending;

    /* Identity strings reported to the server (max 16 bytes each). */
    const char *fw_version;
    const char *fw_build_date;
    const char *fw_build_time;
    const char *fw_proctype;

    uint32_t reads;
    uint32_t errors;
} rh_node_t;

void rh_node_begin(rh_node_t *n, uint16_t freq_mhz, uint8_t enter_at,
                   uint8_t exit_at, uint32_t min_lap_ms);
void rh_node_reset(rh_node_t *n);

/* One calibrated 0..255 strength sample, timestamped in ms. Called at the
 * meter's 1 kHz cadence. */
void rh_node_on_sample(rh_node_t *n, uint8_t counts, uint32_t now_ms);

/* True for bytes the RotorHazard protocol defines as commands. Used to tell
 * a server request apart from a human console keystroke. */
bool rh_node_is_command(uint8_t b);

/* Feed one received byte. Returns the number of response bytes placed in
 * `out` (0 when no response is due). */
int rh_node_handle_byte(rh_node_t *n, uint8_t b, uint8_t *out);

/* Age fields (ms since lap, ms since extremum) are relative to the last
 * sample timestamp, which is what the server expects. */
uint32_t rh_node_ms_since_lap(const rh_node_t *n);
uint32_t rh_node_ms_since_extremum(const rh_node_t *n, const rh_extremum_t *e);

/* Server-visible values, clamped to the range the server accepts. */
uint8_t rh_node_report_rssi(uint8_t v);

#ifdef __cplusplus
}
#endif

#endif /* RH_NODE_H */

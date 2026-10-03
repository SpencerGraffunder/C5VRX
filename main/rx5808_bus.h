/*
 * rx5808_bus.h - Receiving side of the RX5808 3-wire bus, one bit at a time.
 *
 * Ported from FPVGateC5RX (core/rx5808_bus.h, core/rx5808_decode.cpp,
 * core/host_regs.h, CC BY-NC-SA 4.0). FPV-style lap timers (FPVGate and
 * the RX5808 protocol in general) tune the video receiver over a
 * three-wire bus (SEL, CLK, DATA) and read the signal strength back as an
 * analog voltage. With this module fitted the meter becomes a drop-in
 * replacement for the RX5808 module in such a timer.
 *
 * The GPIO interrupt handlers call rx5808_bus_sel_change() and
 * rx5808_bus_clk_change() as SEL and CLK edge. Unit-test and console
 * drivers can feed recorded bits through push_bit()/pop_bit() the same
 * way.
 *
 * A frame is 25 bits, least significant first: bits 0..3 are the address,
 * bit 4 is R/W (1 = write), bits 5..24 are 20 bits of data.
 *   Write: the host sends all 25 bits; we sample DATA on each rising CLK.
 *   Read:  the host sends the first 5 bits; we send the other 20 from the
 *          addressed register, and the host samples them while CLK is low.
 *
 * Registers we answer:
 *   0x1  last frequency value written (FPVGate's verifyFrequency() check)
 *   0x6  C5RX extension: D0-7 signal strength in dBm (signed), D8-15 0xC5
 *   0x7  C5RX extension: D0-7 RSSI 0..255, D8 valid, D9 channel
 *        supported, D11-14 state, D15 always 1
 * Registers the host may write:
 *   0x0  any write = reset (restart the radio, retune to the last frequency)
 *   0x1  D0-15 = RX5808 synthesizer register encoding the target MHz
 *   0x2  D0-19 all ones = power down; any other value = wake
 */
#ifndef RX5808_BUS_H
#define RX5808_BUS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Host register addresses (RX5808 convention + C5RX extensions). */
#define RX5808_REG_STATE      0x0u
#define RX5808_REG_SYNTH_RF   0x1u
#define RX5808_REG_POWER      0x2u
#define RX5808_REG_EXT_INFO   0x6u
#define RX5808_REG_EXT_STATUS 0x7u

#define RX5808_SIGNATURE      0xC5u

typedef struct {
    uint8_t address;
    bool write;
    uint32_t data;      /* 20 bits, only meaningful for writes */
    bool valid;
} rx5808_word_t;

typedef struct {
    /* Called on a read to get the 20-bit value of register addr. */
    uint32_t (*read_provider)(uint8_t addr);
    /* Called once per complete frame with the raw 25 bits (writes: full
     * frame; reads: only the first 5 bits are host data). */
    void (*frame_cb)(uint32_t bits25, bool is_write, uint8_t address);
    /* DATA line control: release the driver (host owns the line) and drive
     * a level (we own the line, during reads only). */
    void (*data_release)(void);
    void (*data_drive)(bool level);
    /* Sample the DATA line (host driving). */
    bool (*data_sample)(void);
    /* Sample SEL / CLK levels. */
    bool (*sel_level)(void);
    bool (*clk_level)(void);

    uint8_t bit_idx;
    uint32_t acc;
    uint8_t address;
    bool write;
    uint32_t out_word;
    bool have_out;
    bool done;
} rx5808_bus_t;

/* SEL changed: the handler calls this with the new level. */
void rx5808_bus_sel_change(rx5808_bus_t *bus, bool sel_high);
/* CLK changed: the handler calls this with the new level. */
void rx5808_bus_clk_change(rx5808_bus_t *bus, bool clk_high);

/* Feed one host bit, sampled on a rising CLK edge (test/console path). */
void rx5808_bus_push_bit(rx5808_bus_t *bus, bool bit);
/* Next bit to send during a read (test/console path). */
bool rx5808_bus_pop_bit(rx5808_bus_t *bus);
/* Does the host send the next bit? It always sends the first 5, and on a
 * write it sends the data too. */
bool rx5808_bus_host_drives(const rx5808_bus_t *bus);
bool rx5808_bus_complete(const rx5808_bus_t *bus);
uint8_t rx5808_bus_bit_index(const rx5808_bus_t *bus);
/* The received frame as raw 25 bits (writes: ready for rx5808_parse_word). */
uint32_t rx5808_bus_raw_bits(const rx5808_bus_t *bus);

/* Frame decode / encode. */
rx5808_word_t rx5808_parse_word(uint32_t bits25);
uint32_t rx5808_build_word(uint8_t address, bool write, uint32_t data20);

/* FPVGate's frequency register math:
 *   tf = (f - 479) / 2;  N = tf / 32;  A = tf % 32;  reg = (N << 7) + A
 * so going back:
 *   N = reg >> 7;  A = reg & 0x7F;  tf = N * 32 + A;  f = tf * 2 + 479 */
uint16_t rx5808_synth_reg_to_mhz(uint16_t reg);
uint16_t rx5808_mhz_to_synth_reg(uint16_t mhz);

/* Build the 20-bit read value for register addr (see header for layout). */
static inline uint32_t rx5808_build_status_word(uint8_t rssi, bool valid,
                                                bool freq_supported,
                                                uint8_t state)
{
    uint32_t w = rssi & 0xFFu;
    if (valid) w |= 1u << 8;
    if (freq_supported) w |= 1u << 9;
    w |= ((uint32_t)(state & 0xFu)) << 11;
    w |= 1u << 15;
    return w;
}

static inline uint32_t rx5808_build_info_word(int dbm)
{
    uint32_t w = (uint8_t)dbm;
    w |= (uint32_t)RX5808_SIGNATURE << 8;
    return w;
}

#ifdef __cplusplus
}
#endif

#endif /* RX5808_BUS_H */

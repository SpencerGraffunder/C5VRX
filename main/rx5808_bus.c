/*
 * rx5808_bus.c - RX5808 3-wire bus, receiving side.
 *
 * Ported from FPVGateC5RX (core/rx5808_bus.h, core/rx5808_decode.cpp,
 * CC BY-NC-SA 4.0). No allocations, no locks: the state machine is driven
 * from the SEL/CLK GPIO interrupt handlers on one core and read back by
 * the main loop.
 */
#include "rx5808_bus.h"

#define RX5808_FRAME_BITS 25u
#define RX5808_HEADER_BITS 5u

static bool host_drives(const rx5808_bus_t *bus)
{
    if (bus->bit_idx >= RX5808_FRAME_BITS) {
        return false;
    }
    if (bus->bit_idx < RX5808_HEADER_BITS) {
        return true;
    }
    return bus->write;
}

void rx5808_bus_sel_change(rx5808_bus_t *bus, bool sel_high)
{
    if (!sel_high) {
        /* SEL low: a frame starts, and the host sends the first bits. */
        bus->bit_idx = 0;
        bus->acc = 0;
        bus->address = 0;
        bus->write = true;
        bus->out_word = 0;
        bus->have_out = false;
        bus->done = false;
        if (bus->data_release) {
            bus->data_release();
        }
    } else {
        /* SEL high: the frame has ended. Only complete frames reach the
         * consumer (a short frame carries no trustworthy data). */
        if (bus->frame_cb && bus->done) {
            bus->frame_cb(bus->acc & 0x1FFFFFFu, bus->write, bus->address);
        }
        if (bus->data_release) {
            bus->data_release();
        }
    }
}

void rx5808_bus_clk_change(rx5808_bus_t *bus, bool clk_high)
{
    if (clk_high) {
        /* Rising edge: read the next bit if the host is sending it. */
        if (host_drives(bus) && bus->data_sample) {
            bool bit = bus->data_sample();
            if (bus->bit_idx < RX5808_FRAME_BITS) {
                if (bit) {
                    bus->acc |= (1u << bus->bit_idx);
                }
                bus->bit_idx++;
                if (bus->bit_idx == RX5808_HEADER_BITS) {
                    /* Address and R/W are now known. */
                    bus->address = (uint8_t)(bus->acc & 0x0Fu);
                    bus->write = (bus->acc >> 4) & 0x1u;
                    if (!bus->write && bus->read_provider) {
                        bus->out_word = bus->read_provider(bus->address) & 0xFFFFFu;
                        bus->have_out = true;
                    }
                }
                if (bus->bit_idx == RX5808_FRAME_BITS) {
                    bus->done = true;
                }
            }
        }
    } else {
        /* Falling edge: during a read, put the next bit on DATA so it's
         * ready when the host samples it. */
        if (!host_drives(bus) && !bus->write && !bus->done &&
            bus->have_out && bus->data_drive) {
            bool bit = (bus->out_word >> (bus->bit_idx - RX5808_HEADER_BITS)) & 0x1u;
            bus->data_drive(bit);
            bus->bit_idx++;
            if (bus->bit_idx == RX5808_FRAME_BITS) {
                bus->done = true;
            }
        }
    }
}

void rx5808_bus_push_bit(rx5808_bus_t *bus, bool bit)
{
    if (bus->bit_idx >= RX5808_FRAME_BITS) {
        return;
    }
    if (bit) {
        bus->acc |= (1u << bus->bit_idx);
    }
    bus->bit_idx++;
    if (bus->bit_idx == RX5808_HEADER_BITS) {
        bus->address = (uint8_t)(bus->acc & 0x0Fu);
        bus->write = (bus->acc >> 4) & 0x1u;
        if (!bus->write && bus->read_provider) {
            bus->out_word = bus->read_provider(bus->address) & 0xFFFFFu;
            bus->have_out = true;
        }
    }
    if (bus->bit_idx == RX5808_FRAME_BITS) {
        bus->done = true;
    }
}

bool rx5808_bus_pop_bit(rx5808_bus_t *bus)
{
    if (bus->bit_idx < RX5808_HEADER_BITS || bus->bit_idx >= RX5808_FRAME_BITS ||
        bus->write || !bus->have_out) {
        return false;
    }
    bool bit = (bus->out_word >> (bus->bit_idx - RX5808_HEADER_BITS)) & 0x1u;
    bus->bit_idx++;
    if (bus->bit_idx == RX5808_FRAME_BITS) {
        bus->done = true;
    }
    return bit;
}

bool rx5808_bus_host_drives(const rx5808_bus_t *bus)
{
    return host_drives(bus);
}

bool rx5808_bus_complete(const rx5808_bus_t *bus)
{
    return bus->done;
}

uint8_t rx5808_bus_bit_index(const rx5808_bus_t *bus)
{
    return bus->bit_idx;
}

uint32_t rx5808_bus_raw_bits(const rx5808_bus_t *bus)
{
    return bus->acc & 0x1FFFFFFu;
}

rx5808_word_t rx5808_parse_word(uint32_t bits25)
{
    rx5808_word_t w;
    w.address = (uint8_t)(bits25 & 0x0Fu);
    w.write = (bits25 >> 4) & 0x1u;
    w.data = (bits25 >> 5) & 0xFFFFFu;
    w.valid = true;
    return w;
}

uint32_t rx5808_build_word(uint8_t address, bool write, uint32_t data20)
{
    uint32_t w = (uint32_t)(address & 0x0Fu);
    w |= (uint32_t)(write ? 1u : 0u) << 4;
    w |= (data20 & 0xFFFFFu) << 5;
    return w & 0x1FFFFFFu;
}

uint16_t rx5808_synth_reg_to_mhz(uint16_t reg)
{
    uint16_t n = (uint16_t)(reg >> 7);
    uint16_t a = (uint16_t)(reg & 0x7Fu);
    uint32_t tf = (uint32_t)n * 32u + a;
    return (uint16_t)(tf * 2u + 479u);
}

uint16_t rx5808_mhz_to_synth_reg(uint16_t mhz)
{
    uint16_t tf = (uint16_t)((mhz - 479u) / 2u);
    uint16_t n = (uint16_t)(tf / 32u);
    uint16_t a = (uint16_t)(tf % 32u);
    return (uint16_t)((n << 7) + a);
}

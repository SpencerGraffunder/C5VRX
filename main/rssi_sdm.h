/*
 * rssi_sdm.h - Sigma-delta analog RSSI output (optional).
 *
 * The C5 has no DAC. Its sigma-delta peripheral switches a GPIO on and off
 * several million times a second; an RC filter on the board smooths the
 * switching into a steady voltage. This is the same output the FPVGateC5RX
 * project (CC BY-NC-SA 4.0) uses for its RSSI pin, and it gives ~8-bit
 * resolution versus the 6-bit resistor ladder the meter also drives.
 *
 * Wiring (from the reference docs/HARDWARE.md):
 *
 *   C5 GPIO <pin> ---[ R1 10k ]---+----------- RSSI (timer ADC input)
 *                                 |
 *                                 +---[ R2 10k ]---- GND
 *                                 |
 *                                 +---[ C1 100nF ]-- GND
 *
 * With no divider (R1 absent, R2/C1 only) use the 1000 divider setting.
 */
#ifndef RSSI_SDM_H
#define RSSI_SDM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Begin the sigma-delta output on pin. Returns false if the pin is -1
 * (disabled) or the driver init failed. */
bool rssi_sdm_begin(int pin, int divider_m1000);
/* Map a calibrated 0..255 strength value to the output voltage.
 * density = Vdd * (d+128)/256 * divider_ratio. */
void rssi_sdm_set_counts(int counts);
bool rssi_sdm_active(void);

#ifdef __cplusplus
}
#endif

#endif /* RSSI_SDM_H */

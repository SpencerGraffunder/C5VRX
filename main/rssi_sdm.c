/*
 * rssi_sdm.c - Sigma-delta analog RSSI output (optional).
 *
 * Mapping ported from FPVGateC5RX (core/rssi_codec.cpp, CC BY-NC-SA 4.0):
 * the ESP-IDF sigma-delta "density" runs from -128 to 127, and the average
 * output is Vdd * (density + 128) / 256.
 */
#include "rssi_sdm.h"

#include <math.h>

#include "driver/gpio.h"
#include "driver/sdm.h"

#define RSSI_SDM_VDD          3.3f
#define RSSI_SDM_V_FULL_SCALE 1.50f   /* RSSI 255 stays under a ~1.55 V ADC limit */
#define RSSI_SDM_V_FLOOR      0.0f
#define RSSI_SDM_RATE_HZ      (4 * 1000 * 1000)

static sdm_channel_handle_t s_chan = NULL;
static float s_divider_ratio = 0.5f;

bool rssi_sdm_begin(int pin, int divider_m1000)
{
    if (pin < 0) {
        return false;
    }
    s_divider_ratio = (float)divider_m1000 / 1000.0f;
    if (s_divider_ratio <= 0.0f) {
        s_divider_ratio = 0.5f;
    }

    sdm_config_t cfg = {
        .gpio_num = (gpio_num_t)pin,
        .clk_src = SDM_CLK_SRC_DEFAULT,
        .sample_rate_hz = RSSI_SDM_RATE_HZ,
    };
    esp_err_t err = sdm_new_channel(&cfg, &s_chan);
    if (err == ESP_OK) {
        err = sdm_channel_enable(s_chan);
        if (err == ESP_OK) {
            sdm_channel_set_pulse_density(s_chan, -128);
        }
    }
    return err == ESP_OK;
}

void rssi_sdm_set_counts(int counts)
{
    if (!s_chan) {
        return;
    }
    if (counts < 0) counts = 0;
    if (counts > 255) counts = 255;
    float pin_v = RSSI_SDM_V_FLOOR +
                  (RSSI_SDM_V_FULL_SCALE - RSSI_SDM_V_FLOOR) * (counts / 255.0f);
    float duty = pin_v / s_divider_ratio / RSSI_SDM_VDD;
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;
    int v = (int)floorf(duty * 256.0f - 128.0f + 0.5f);
    if (v < -128) v = -128;
    if (v > 127) v = 127;
    sdm_channel_set_pulse_density(s_chan, (int8_t)v);
}

bool rssi_sdm_active(void)
{
    return s_chan != NULL;
}

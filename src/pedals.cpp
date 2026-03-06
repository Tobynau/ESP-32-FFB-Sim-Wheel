#include "pedals.h"

#include <math.h>
#include <HX711.h>

static HX711 brake_loadcell;

static bool pedals_initialized = false;
static bool brake_initialized = false;

static int clutch_pin = -1;
static int gas_pin = -1;
static int clutch_min_adc = 0;
static int clutch_max_adc = 4095;
static int gas_min_adc = 0;
static int gas_max_adc = 4095;

static int32_t brake_zero_counts = 0;
static int32_t brake_fullscale_counts = 100000;

static uint16_t clutch_hid = 0;
static uint16_t gas_hid = 0;
static uint16_t brake_hid = 0;

static uint32_t last_update_ms = 0;

static inline float clamp01(float v) {
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

static inline int clamp_adc(int raw) {
    if (raw < 0) return 0;
    if (raw > 4095) return 4095;
    return raw;
}

static inline float normalize_adc(int raw, int min_adc, int max_adc) {
    int range = max_adc - min_adc;
    if (range == 0) return 0.0f;
    float norm = (float)(raw - min_adc) / (float)range;
    return clamp01(norm);
}

static inline uint16_t unit_to_hid16(float v) {
    float clamped = clamp01(v);
    int32_t scaled = (int32_t)lroundf(clamped * 32767.0f);
    if (scaled < 0) scaled = 0;
    if (scaled > 32767) scaled = 32767;
    return (uint16_t)scaled;
}

void pedals_init(
    int clutch_adc_pin,
    int gas_adc_pin,
    int clutch_adc_min,
    int clutch_adc_max,
    int gas_adc_min,
    int gas_adc_max,
    int brake_hx711_dout_pin,
    int brake_hx711_sck_pin,
    int32_t brake_counts_fullscale) {

    clutch_pin = clutch_adc_pin;
    gas_pin = gas_adc_pin;

    clutch_adc_min = clamp_adc(clutch_adc_min);
    clutch_adc_max = clamp_adc(clutch_adc_max);
    gas_adc_min = clamp_adc(gas_adc_min);
    gas_adc_max = clamp_adc(gas_adc_max);

    if (clutch_adc_max < clutch_adc_min) {
        int t = clutch_adc_min;
        clutch_adc_min = clutch_adc_max;
        clutch_adc_max = t;
    }
    if (gas_adc_max < gas_adc_min) {
        int t = gas_adc_min;
        gas_adc_min = gas_adc_max;
        gas_adc_max = t;
    }

    clutch_min_adc = clutch_adc_min;
    clutch_max_adc = clutch_adc_max;
    gas_min_adc = gas_adc_min;
    gas_max_adc = gas_adc_max;

    if (clutch_pin >= 0) pinMode(clutch_pin, INPUT);
    if (gas_pin >= 0) pinMode(gas_pin, INPUT);

    analogReadResolution(12);

    if (brake_counts_fullscale < 1000) brake_counts_fullscale = 1000;
    brake_fullscale_counts = brake_counts_fullscale;

    brake_initialized = false;
    if (brake_hx711_dout_pin >= 0 && brake_hx711_sck_pin >= 0) {
        brake_loadcell.begin(brake_hx711_dout_pin, brake_hx711_sck_pin);
        delay(50);
        if (brake_loadcell.is_ready()) {
            brake_zero_counts = brake_loadcell.read_average(12);
            brake_initialized = true;
        }
    }

    clutch_hid = 0;
    gas_hid = 0;
    brake_hid = 0;
    last_update_ms = 0;
    pedals_initialized = true;
}

void pedals_update() {
    if (!pedals_initialized) return;

    uint32_t now = millis();
    if ((now - last_update_ms) < 2) {
        return;
    }
    last_update_ms = now;

    if (clutch_pin >= 0) {
        int raw = analogRead(clutch_pin);
        float norm = normalize_adc(raw, clutch_min_adc, clutch_max_adc);
        clutch_hid = unit_to_hid16(norm);
    } else {
        clutch_hid = 0;
    }

    if (gas_pin >= 0) {
        int raw = analogRead(gas_pin);
        float norm = normalize_adc(raw, gas_min_adc, gas_max_adc);
        gas_hid = unit_to_hid16(norm);
    } else {
        gas_hid = 0;
    }

    if (brake_initialized && brake_loadcell.is_ready()) {
        // Use wait_ready_timeout with a short timeout instead of read(),
        // because read() internally calls wait_ready() which is an infinite
        // loop. If the HX711 becomes not-ready between is_ready() and read(),
        // it can block the ffb_task for hundreds of ms — stalling encoder
        // updates and VESC commands.
        if (brake_loadcell.wait_ready_timeout(1)) {
            int32_t raw = brake_loadcell.read();
            int32_t delta = raw - brake_zero_counts;
            float norm = (float)delta / (float)brake_fullscale_counts;
            brake_hid = unit_to_hid16(norm);
        }
    } else {
        brake_hid = 0;
    }
}

uint16_t pedals_get_clutch_hid16() {
    return clutch_hid;
}

uint16_t pedals_get_gas_hid16() {
    return gas_hid;
}

uint16_t pedals_get_brake_hid16() {
    return brake_hid;
}

#include "pedals.h"

#include <math.h>
#include <Wire.h>
#include <MT6701.hpp>
#include <HX711.h>

static MT6701 *clutch_encoder = nullptr;
static MT6701 *gas_encoder = nullptr;
static HX711 brake_loadcell;

static bool pedals_initialized = false;
static bool brake_initialized = false;

static float clutch_zero_turns = 0.0f;
static float gas_zero_turns = 0.0f;
static float clutch_range_turns = 0.25f;
static float gas_range_turns = 0.25f;

static int32_t brake_zero_counts = 0;
static int32_t brake_fullscale_counts = 100000;

static uint16_t clutch_hid = 0;
static uint16_t gas_hid = 0;
static uint16_t brake_hid = 0;

static uint32_t last_update_ms = 0;

static bool i2c_device_present(uint8_t addr) {
    Wire.beginTransmission(addr);
    return (Wire.endTransmission(true) == 0);
}

static inline float clamp01(float v) {
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

static inline uint16_t unit_to_hid16(float v) {
    float clamped = clamp01(v);
    int32_t scaled = (int32_t)lroundf(clamped * 32767.0f);
    if (scaled < 0) scaled = 0;
    if (scaled > 32767) scaled = 32767;
    return (uint16_t)scaled;
}

void pedals_init(
    int i2c_sda_pin,
    int i2c_scl_pin,
    uint8_t clutch_i2c_addr,
    uint8_t gas_i2c_addr,
    float clutch_travel_turns,
    float gas_travel_turns,
    int brake_hx711_dout_pin,
    int brake_hx711_sck_pin,
    int32_t brake_counts_fullscale) {

    if (clutch_encoder) {
        delete clutch_encoder;
        clutch_encoder = nullptr;
    }
    if (gas_encoder) {
        delete gas_encoder;
        gas_encoder = nullptr;
    }

    if (!isfinite(clutch_travel_turns) || clutch_travel_turns <= 0.01f) clutch_travel_turns = 0.25f;
    if (!isfinite(gas_travel_turns) || gas_travel_turns <= 0.01f) gas_travel_turns = 0.25f;
    clutch_range_turns = clutch_travel_turns;
    gas_range_turns = gas_travel_turns;

    if (brake_counts_fullscale < 1000) brake_counts_fullscale = 1000;
    brake_fullscale_counts = brake_counts_fullscale;

    if (i2c_device_present(clutch_i2c_addr)) {
        clutch_encoder = new MT6701(clutch_i2c_addr, 5, MT6701::RPM_THRESHOLD, MT6701::RPM_FILTER_SIZE, i2c_sda_pin, i2c_scl_pin);
        clutch_encoder->begin();
        delay(5);
        float ct = clutch_encoder->getTurns();
        clutch_zero_turns = isfinite(ct) ? ct : 0.0f;
    } else {
        clutch_zero_turns = 0.0f;
    }

    if (i2c_device_present(gas_i2c_addr)) {
        gas_encoder = new MT6701(gas_i2c_addr, 5, MT6701::RPM_THRESHOLD, MT6701::RPM_FILTER_SIZE, i2c_sda_pin, i2c_scl_pin);
        gas_encoder->begin();
        delay(5);
        float gt = gas_encoder->getTurns();
        gas_zero_turns = isfinite(gt) ? gt : 0.0f;
    } else {
        gas_zero_turns = 0.0f;
    }

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

    if (clutch_encoder) {
        float t = clutch_encoder->getTurns();
        if (isfinite(t)) {
            float norm = (t - clutch_zero_turns) / clutch_range_turns;
            clutch_hid = unit_to_hid16(norm);
        } else {
            clutch_hid = 0;
        }
    } else {
        clutch_hid = 0;
    }

    if (gas_encoder) {
        float t = gas_encoder->getTurns();
        if (isfinite(t)) {
            float norm = (t - gas_zero_turns) / gas_range_turns;
            gas_hid = unit_to_hid16(norm);
        } else {
            gas_hid = 0;
        }
    } else {
        gas_hid = 0;
    }

    if (brake_initialized && brake_loadcell.is_ready()) {
        int32_t raw = brake_loadcell.read();
        int32_t delta = raw - brake_zero_counts;
        float norm = (float)delta / (float)brake_fullscale_counts;
        brake_hid = unit_to_hid16(norm);
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

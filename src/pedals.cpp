#include "pedals.h"
#include "config.h"

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
static float ema_alpha = 0.4f;
static float brake_gamma = 1.0f;

// Filtered normalized positions (0..1)
static float clutch_f = 0.0f;
static float gas_f = 0.0f;
static float brake_f = 0.0f;

static uint16_t clutch_hid = 0;
static uint16_t gas_hid = 0;
static uint16_t brake_hid = 0;

// Raw values for the calibration UI
static uint16_t clutch_raw = 0;
static uint16_t gas_raw = 0;
static int32_t brake_raw_delta = 0;

static uint32_t last_update_ms = 0;

// Endpoint margin: the top/bottom 2% of calibrated travel maps to exactly
// 0 / max so games reliably see fully-released and fully-pressed.
static constexpr float END_MARGIN = 0.02f;

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
    // Rescale so the endpoint margins clip to 0 and 1.
    norm = (norm - END_MARGIN) / (1.0f - 2.0f * END_MARGIN);
    return clamp01(norm);
}

static inline uint16_t unit_to_hid16(float v) {
    float clamped = clamp01(v);
    int32_t scaled = (int32_t)lroundf(clamped * 32767.0f);
    if (scaled < 0) scaled = 0;
    if (scaled > 32767) scaled = 32767;
    return (uint16_t)scaled;
}

void pedals_apply_config() {
    int cmin = clamp_adc(g_cfg.clutch_adc_min);
    int cmax = clamp_adc(g_cfg.clutch_adc_max);
    int gmin = clamp_adc(g_cfg.gas_adc_min);
    int gmax = clamp_adc(g_cfg.gas_adc_max);
    if (cmax < cmin) { int t = cmin; cmin = cmax; cmax = t; }
    if (gmax < gmin) { int t = gmin; gmin = gmax; gmax = t; }
    clutch_min_adc = cmin;
    clutch_max_adc = cmax;
    gas_min_adc = gmin;
    gas_max_adc = gmax;

    brake_fullscale_counts = (int32_t)g_cfg.brake_fullscale_counts;
    if (brake_fullscale_counts < 1000) brake_fullscale_counts = 1000;

    // EMA per 2ms sample; level 0 = raw
    static const float alphas[4] = {1.0f, 0.4f, 0.2f, 0.08f};
    ema_alpha = alphas[g_cfg.pedal_filter > 3 ? 3 : g_cfg.pedal_filter];

    // Brake response curve: output = input^gamma of normalized force
    static const float gammas[3] = {1.0f, 1.35f, 1.8f};
    brake_gamma = gammas[g_cfg.brake_curve > 2 ? 2 : g_cfg.brake_curve];
}

void pedals_init(
    int clutch_adc_pin,
    int gas_adc_pin,
    int brake_hx711_dout_pin,
    int brake_hx711_sck_pin) {

    clutch_pin = clutch_adc_pin;
    gas_pin = gas_adc_pin;

    if (clutch_pin >= 0) pinMode(clutch_pin, INPUT);
    if (gas_pin >= 0) pinMode(gas_pin, INPUT);

    analogReadResolution(12);

    pedals_apply_config();

    brake_initialized = false;
    if (brake_hx711_dout_pin >= 0 && brake_hx711_sck_pin >= 0) {
        brake_loadcell.begin(brake_hx711_dout_pin, brake_hx711_sck_pin);
        delay(50);
        if (brake_loadcell.is_ready()) {
            brake_zero_counts = brake_loadcell.read_average(12);
            brake_initialized = true;
        }
    }

    clutch_f = gas_f = brake_f = 0.0f;
    clutch_hid = gas_hid = brake_hid = 0;
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
        clutch_raw = (uint16_t)clamp_adc(raw);
        float norm = normalize_adc(raw, clutch_min_adc, clutch_max_adc);
        clutch_f += ema_alpha * (norm - clutch_f);
        clutch_hid = unit_to_hid16(clutch_f);
    } else {
        clutch_hid = 0;
    }

    if (gas_pin >= 0) {
        int raw = analogRead(gas_pin);
        gas_raw = (uint16_t)clamp_adc(raw);
        float norm = normalize_adc(raw, gas_min_adc, gas_max_adc);
        gas_f += ema_alpha * (norm - gas_f);
        gas_hid = unit_to_hid16(gas_f);
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
            brake_raw_delta = raw - brake_zero_counts;
            float norm = clamp01((float)brake_raw_delta / (float)brake_fullscale_counts);
            if (brake_gamma != 1.0f) norm = powf(norm, brake_gamma);
            // Load cell samples arrive at 10-80Hz; only fold new readings in.
            brake_f += 0.5f * (norm - brake_f);
            brake_hid = unit_to_hid16(brake_f);
        }
    } else if (!brake_initialized) {
        brake_hid = 0;
    }
}

uint16_t pedals_get_clutch_hid16() { return clutch_hid; }
uint16_t pedals_get_gas_hid16() { return gas_hid; }
uint16_t pedals_get_brake_hid16() { return brake_hid; }

uint16_t pedals_get_clutch_raw() { return clutch_raw; }
uint16_t pedals_get_gas_raw() { return gas_raw; }
int32_t pedals_get_brake_raw() { return brake_raw_delta; }

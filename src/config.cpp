#include "config.h"
#include "encoder.h"
#include "pedals.h"
#include "effects.h"
#include "hid.h"
#include <Preferences.h>

WheelSettings g_cfg;

static Preferences prefs;
static const uint32_t CFG_MAGIC = 0x46464232; // "FFB2" — bump when struct layout changes

// Wire format for the vendor config feature report. Fixed-point fields keep
// the report compact and WebHID-friendly (no float endianness concerns).
struct __attribute__((packed)) WheelConfigWire {
    uint8_t  version;              // wire version, currently 1
    uint8_t  command;              // CFG_CMD_* (SET only; reads back as 0)
    uint16_t wheel_range_deg;
    uint16_t max_torque_cnm;       // Nm * 100
    uint16_t torque_to_current_c;  // A/Nm * 100
    uint16_t endstop_k_dnm;        // Nm/rad * 10
    uint16_t endstop_d_cnm;        // Nm/(rad/s) * 100
    uint16_t endstop_max_cnm;      // Nm * 100
    uint8_t  recon_cutoff_hz;      // 0 = off
    uint16_t slew_cnm_per_ms;      // Nm/ms * 100, 0 = off
    uint16_t base_damper_mnm;      // Nm/(rad/s) * 1000
    uint16_t base_friction_mnm;    // Nm * 1000
    uint16_t base_inertia_mnm;     // Nm/(rad/s^2) * 1000
    uint8_t  flags;                // bit0 invert, bit1 soft clip, bit2 osc guard, bit3 breakaway
    uint16_t breakaway_mnm;        // Nm * 1000
    uint16_t clutch_adc_min, clutch_adc_max;
    uint16_t gas_adc_min, gas_adc_max;
    uint32_t brake_fullscale_counts;
    uint8_t  pedal_filter;
    uint8_t  brake_curve;
    uint8_t  reserved[8];
};
static_assert(sizeof(WheelConfigWire) == CONFIG_WIRE_SIZE, "config wire size mismatch");

struct __attribute__((packed)) WheelTelemetryWire {
    int32_t  angle_mrad;       // centered wheel angle, milliradians
    int16_t  vel_crads;        // rad/s * 100
    int16_t  torque_cnm;       // commanded rim torque, Nm * 100
    uint16_t clutch_raw;       // raw ADC
    uint16_t gas_raw;          // raw ADC
    int32_t  brake_raw;        // HX711 counts above zero point
    uint8_t  flags;            // bit0 actuators, bit1 paused, bit2 usb active, bit3 ffb faded
    uint8_t  active_blocks;
    uint8_t  effect_playing;
    uint8_t  version;
};
static_assert(sizeof(WheelTelemetryWire) == TELEMETRY_WIRE_SIZE, "telemetry wire size mismatch");

void config_defaults() {
    g_cfg.wheel_range_deg = 900.0f;
    g_cfg.max_torque_nm = 8.0f;
    g_cfg.torque_to_current = 1.5f;
    g_cfg.endstop_k = 250.0f;
    g_cfg.endstop_d = 1.0f;
    g_cfg.endstop_max_nm = 10.0f;
    g_cfg.recon_cutoff_hz = 50.0f;
    g_cfg.slew_nm_per_ms = 1.0f;
    g_cfg.base_damper = 0.05f;
    g_cfg.base_friction_nm = 0.03f;
    g_cfg.base_inertia = 0.0f;
    g_cfg.invert_ffb = false;
    g_cfg.soft_clip = true;
    g_cfg.osc_guard = true;
    g_cfg.breakaway = false;
    g_cfg.breakaway_nm = 0.05f;
    g_cfg.clutch_adc_min = 700;
    g_cfg.clutch_adc_max = 3300;
    g_cfg.gas_adc_min = 700;
    g_cfg.gas_adc_max = 3300;
    g_cfg.brake_fullscale_counts = 100000;
    g_cfg.pedal_filter = 1;
    g_cfg.brake_curve = 0;
}

static float clampf(float v, float lo, float hi) {
    if (!isfinite(v)) return lo;
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// Keep any applied settings inside safe/sane bounds regardless of source.
static void config_sanitize() {
    g_cfg.wheel_range_deg = clampf(g_cfg.wheel_range_deg, 90.0f, 1800.0f);
    g_cfg.max_torque_nm = clampf(g_cfg.max_torque_nm, 0.5f, 25.0f);
    g_cfg.torque_to_current = clampf(g_cfg.torque_to_current, 0.1f, 5.0f);
    g_cfg.endstop_k = clampf(g_cfg.endstop_k, 10.0f, 1000.0f);
    g_cfg.endstop_d = clampf(g_cfg.endstop_d, 0.0f, 10.0f);
    g_cfg.endstop_max_nm = clampf(g_cfg.endstop_max_nm, 1.0f, 25.0f);
    g_cfg.recon_cutoff_hz = clampf(g_cfg.recon_cutoff_hz, 0.0f, 250.0f);
    g_cfg.slew_nm_per_ms = clampf(g_cfg.slew_nm_per_ms, 0.0f, 50.0f);
    g_cfg.base_damper = clampf(g_cfg.base_damper, 0.0f, 2.0f);
    g_cfg.base_friction_nm = clampf(g_cfg.base_friction_nm, 0.0f, 2.0f);
    g_cfg.base_inertia = clampf(g_cfg.base_inertia, 0.0f, 1.0f);
    g_cfg.breakaway_nm = clampf(g_cfg.breakaway_nm, 0.0f, 1.0f);
    if (g_cfg.brake_fullscale_counts < 1000) g_cfg.brake_fullscale_counts = 1000;
    if (g_cfg.pedal_filter > 3) g_cfg.pedal_filter = 3;
    if (g_cfg.brake_curve > 2) g_cfg.brake_curve = 2;
}

static bool prefs_started = false;

void config_init() {
    config_defaults();
    if (!prefs_started) {
        prefs.begin("ffbwheel", false);
        prefs_started = true;
    }
    if (prefs.getUInt("magic", 0) == CFG_MAGIC &&
        prefs.getBytesLength("cfg") == sizeof(WheelSettings)) {
        prefs.getBytes("cfg", &g_cfg, sizeof(WheelSettings));
    }
    config_sanitize();
}

bool config_save() {
    size_t n = prefs.putBytes("cfg", &g_cfg, sizeof(WheelSettings));
    prefs.putUInt("magic", CFG_MAGIC);
    return n == sizeof(WheelSettings);
}

uint16_t config_fill_wire(uint8_t *buf, uint16_t maxlen) {
    if (!buf || maxlen < sizeof(WheelConfigWire)) return 0;
    WheelConfigWire w = {};
    w.version = 1;
    w.command = 0;
    w.wheel_range_deg = (uint16_t)lroundf(g_cfg.wheel_range_deg);
    w.max_torque_cnm = (uint16_t)lroundf(g_cfg.max_torque_nm * 100.0f);
    w.torque_to_current_c = (uint16_t)lroundf(g_cfg.torque_to_current * 100.0f);
    w.endstop_k_dnm = (uint16_t)lroundf(g_cfg.endstop_k * 10.0f);
    w.endstop_d_cnm = (uint16_t)lroundf(g_cfg.endstop_d * 100.0f);
    w.endstop_max_cnm = (uint16_t)lroundf(g_cfg.endstop_max_nm * 100.0f);
    w.recon_cutoff_hz = (uint8_t)lroundf(g_cfg.recon_cutoff_hz);
    w.slew_cnm_per_ms = (uint16_t)lroundf(g_cfg.slew_nm_per_ms * 100.0f);
    w.base_damper_mnm = (uint16_t)lroundf(g_cfg.base_damper * 1000.0f);
    w.base_friction_mnm = (uint16_t)lroundf(g_cfg.base_friction_nm * 1000.0f);
    w.base_inertia_mnm = (uint16_t)lroundf(g_cfg.base_inertia * 1000.0f);
    w.flags = (g_cfg.invert_ffb ? 0x01 : 0) |
              (g_cfg.soft_clip ? 0x02 : 0) |
              (g_cfg.osc_guard ? 0x04 : 0) |
              (g_cfg.breakaway ? 0x08 : 0);
    w.breakaway_mnm = (uint16_t)lroundf(g_cfg.breakaway_nm * 1000.0f);
    w.clutch_adc_min = g_cfg.clutch_adc_min;
    w.clutch_adc_max = g_cfg.clutch_adc_max;
    w.gas_adc_min = g_cfg.gas_adc_min;
    w.gas_adc_max = g_cfg.gas_adc_max;
    w.brake_fullscale_counts = g_cfg.brake_fullscale_counts;
    w.pedal_filter = g_cfg.pedal_filter;
    w.brake_curve = g_cfg.brake_curve;
    memcpy(buf, &w, sizeof(w));
    return sizeof(w);
}

static uint8_t pending_wire[CONFIG_WIRE_SIZE];
static volatile bool pending_ready = false;

void config_queue_wire(const uint8_t *buf, uint16_t len) {
    if (!buf || len < CONFIG_WIRE_SIZE) return;
    if (pending_ready) return; // one in flight; host will re-read and retry
    memcpy(pending_wire, buf, CONFIG_WIRE_SIZE);
    pending_ready = true;
}

void config_process_pending() {
    if (!pending_ready) return;
    uint8_t local[CONFIG_WIRE_SIZE];
    memcpy(local, pending_wire, CONFIG_WIRE_SIZE);
    pending_ready = false;
    config_apply_wire(local, CONFIG_WIRE_SIZE);
}

void config_apply_wire(const uint8_t *buf, uint16_t len) {
    if (!buf || len < sizeof(WheelConfigWire)) return;
    WheelConfigWire w;
    memcpy(&w, buf, sizeof(w));
    if (w.version != 1) return;

    switch (w.command) {
        case CFG_CMD_SET_CENTER:
            encoder_set_center();
            return;
        case CFG_CMD_DEFAULTS:
            config_defaults();
            break;
        case CFG_CMD_RELOAD:
            config_init();
            break;
        case CFG_CMD_APPLY:
        case CFG_CMD_APPLY_SAVE:
            g_cfg.wheel_range_deg = (float)w.wheel_range_deg;
            g_cfg.max_torque_nm = w.max_torque_cnm / 100.0f;
            g_cfg.torque_to_current = w.torque_to_current_c / 100.0f;
            g_cfg.endstop_k = w.endstop_k_dnm / 10.0f;
            g_cfg.endstop_d = w.endstop_d_cnm / 100.0f;
            g_cfg.endstop_max_nm = w.endstop_max_cnm / 100.0f;
            g_cfg.recon_cutoff_hz = (float)w.recon_cutoff_hz;
            g_cfg.slew_nm_per_ms = w.slew_cnm_per_ms / 100.0f;
            g_cfg.base_damper = w.base_damper_mnm / 1000.0f;
            g_cfg.base_friction_nm = w.base_friction_mnm / 1000.0f;
            g_cfg.base_inertia = w.base_inertia_mnm / 1000.0f;
            g_cfg.invert_ffb = (w.flags & 0x01) != 0;
            g_cfg.soft_clip = (w.flags & 0x02) != 0;
            g_cfg.osc_guard = (w.flags & 0x04) != 0;
            g_cfg.breakaway = (w.flags & 0x08) != 0;
            g_cfg.breakaway_nm = w.breakaway_mnm / 1000.0f;
            g_cfg.clutch_adc_min = w.clutch_adc_min;
            g_cfg.clutch_adc_max = w.clutch_adc_max;
            g_cfg.gas_adc_min = w.gas_adc_min;
            g_cfg.gas_adc_max = w.gas_adc_max;
            g_cfg.brake_fullscale_counts = w.brake_fullscale_counts;
            g_cfg.pedal_filter = w.pedal_filter;
            g_cfg.brake_curve = w.brake_curve;
            break;
        default:
            return;
    }

    config_sanitize();
    pedals_apply_config();
    if (w.command == CFG_CMD_APPLY_SAVE) {
        config_save();
    }
}

uint16_t telemetry_fill_wire(uint8_t *buf, uint16_t maxlen) {
    if (!buf || maxlen < sizeof(WheelTelemetryWire)) return 0;
    WheelTelemetryWire t = {};
    t.version = 1;
    float ang = encoder_read_centered_angle_rad();
    float vel = encoder_read_vel_rads();
    t.angle_mrad = (int32_t)lroundf(ang * 1000.0f);
    float v = vel * 100.0f;
    t.vel_crads = (int16_t)clampf(v, -32767.0f, 32767.0f);
    float tq = g_ffb_torque_nm * 100.0f;
    t.torque_cnm = (int16_t)clampf(tq, -32767.0f, 32767.0f);
    t.clutch_raw = pedals_get_clutch_raw();
    t.gas_raw = pedals_get_gas_raw();
    t.brake_raw = pedals_get_brake_raw();
    t.flags = effects_get_status_flags();
    t.active_blocks = effects_get_active_block_count();
    t.effect_playing = effect_playing;
    memcpy(buf, &t, sizeof(t));
    return sizeof(t);
}

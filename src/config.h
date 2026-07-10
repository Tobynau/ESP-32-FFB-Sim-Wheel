#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Runtime-tunable wheel settings. Loaded from NVS at boot, adjustable live
// over the vendor HID feature report (see CONFIG_PROTOCOL.md) so the web app
// can tune feel without reflashing.
struct WheelSettings {
    // Steering / torque
    float wheel_range_deg;      // lock-to-lock rotation (e.g. 900)
    float max_torque_nm;        // rim torque limit for game effects
    float torque_to_current;    // motor amps per motor Nm (from motor Kt)

    // End-stop (soft lock) — has its own torque budget on top of max_torque_nm
    float endstop_k;            // Nm/rad
    float endstop_d;            // Nm/(rad/s), damping inside the end-stop zone
    float endstop_max_nm;       // cap for the end-stop contribution

    // Output conditioning
    float recon_cutoff_hz;      // LPF on host constant/ramp torque, 0 = off
    float slew_nm_per_ms;       // total-torque slew limit, 0 = off

    // Device-side base feel (always-on, under the game's FFB)
    float base_damper;          // Nm/(rad/s)
    float base_friction_nm;     // Nm
    float base_inertia;         // Nm/(rad/s^2)

    // Behavior flags
    bool invert_ffb;            // flip host-commanded force direction
    bool soft_clip;             // tanh knee at max torque instead of hard clamp
    bool osc_guard;             // hands-off oscillation protection
    bool breakaway;             // static-friction/cogging compensation
    float breakaway_nm;         // boost added near zero output when enabled

    // Pedals
    uint16_t clutch_adc_min, clutch_adc_max;
    uint16_t gas_adc_min, gas_adc_max;
    uint32_t brake_fullscale_counts; // HX711 counts at full braking force
    uint8_t pedal_filter;       // 0..3 (off..heavy EMA)
    uint8_t brake_curve;        // 0 linear, 1 mild, 2 progressive
};

extern WheelSettings g_cfg;

// Vendor HID report IDs (feature reports on the vendor collection)
#define CONFIG_REPORT_ID    0x60
#define TELEMETRY_REPORT_ID 0x61
#define CONFIG_WIRE_SIZE    48
#define TELEMETRY_WIRE_SIZE 20

// Commands accepted in the config SET report's command byte
#define CFG_CMD_APPLY        0  // apply to RAM only
#define CFG_CMD_APPLY_SAVE   1  // apply + persist to NVS
#define CFG_CMD_SET_CENTER   2  // set current wheel position as center
#define CFG_CMD_DEFAULTS     3  // restore factory defaults (RAM)
#define CFG_CMD_RELOAD       4  // reload last saved settings from NVS

void config_init();      // load from NVS (or defaults) — call once in setup()
void config_defaults();
bool config_save();
uint16_t config_fill_wire(uint8_t *buf, uint16_t maxlen);
void config_apply_wire(const uint8_t *buf, uint16_t len);
uint16_t telemetry_fill_wire(uint8_t *buf, uint16_t maxlen);

#endif // CONFIG_H

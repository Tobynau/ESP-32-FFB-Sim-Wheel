#ifndef EFFECTS_H
#define EFFECTS_H

#include <Arduino.h>

// Internal effect types (mapped from the USB HID PID effect type list)
enum EffectType {
    ET_NONE = 0,
    ET_CONSTANT,   // PID 0x26
    ET_RAMP,       // PID 0x27
    ET_SPRING,     // PID 0x40
    ET_DAMPER,     // PID 0x41
    ET_INERTIA,    // PID 0x42
    ET_FRICTION,   // PID 0x43
    ET_PERIODIC    // PID 0x30..0x34
};

// Periodic waveforms
enum WaveType {
    WAVE_SINE = 0,
    WAVE_SQUARE,
    WAVE_TRIANGLE,
    WAVE_SAW_UP,
    WAVE_SAW_DOWN
};

struct EffectBlock {
    uint8_t in_use;
    uint8_t type;            // EffectType
    uint8_t wave;            // WaveType (periodic only)
    bool active;
    bool direction_enable;

    uint32_t start_time_ms;  // effects-clock time playback started
    uint32_t duration_ms;    // 0 = infinite
    uint32_t start_delay_ms;
    uint8_t loop_count;      // remaining plays; 0xFF = infinite

    float magnitude;         // -1..1 (constant force, periodic amplitude, ramp start)
    float magnitude_end;     // -1..1 (ramp end)
    float offset;            // -1..1 (periodic DC offset)
    float direction_rad;     // 0..2PI, PID convention (0=north, 90deg=+X)
    float freq_hz;           // periodic frequency
    float phase0;            // periodic initial phase (rad)
    float phase_acc;         // integrated phase (rad), advanced at CTRL rate
    float gain;              // 0..1 per-effect gain from Set Effect

    // Envelope (0 times = no envelope)
    float attack_level;      // 0..1
    float fade_level;        // 0..1
    uint32_t attack_time_ms;
    uint32_t fade_time_ms;

    // Condition parameters, normalized per the PID spec
    float cp_offset;         // -1..1 of the metric full-scale
    float pos_coeff;         // -1..1
    float neg_coeff;         // -1..1
    float pos_sat;           // 0..1
    float neg_sat;           // 0..1
    float deadband;          // 0..1
};

#define MAX_EFFECT_BLOCKS 16

// One-time init (creates the block mutex). Call from setup() before tasks start.
void effects_init();

// Full torque pipeline, called at CTRL_HZ from the FFB task.
// Returns commanded rim torque in Nm (already gated, clipped, slew-limited).
float effects_compute_torque(float angle_rad, float vel_rads, float dt_s);

bool ffb_has_active_effects();
uint32_t effects_last_host_ms();   // millis() of last FFB report from host
void effects_all_stop();           // stop playback of every effect (USB suspend, safety)

// HID plumbing
void handle_ffb_report(uint8_t report_id, uint8_t *buf, uint16_t len);
uint16_t ffb_get_feature_report(uint8_t report_id, uint8_t *buffer, uint16_t len);

// Telemetry helpers
uint8_t effects_get_status_flags();       // bit0 actuators, bit1 paused, bit2 usb, bit3 faded
uint8_t effects_get_active_block_count();

extern bool ffb_verbose;
extern float device_gain;
extern bool device_paused;
extern volatile float g_ffb_torque_nm;    // last commanded rim torque (for telemetry)

#endif // EFFECTS_H

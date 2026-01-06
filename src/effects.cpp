#include "effects.h"
#include <Arduino.h>
#include <math.h>
#define CTRL_HZ 1000UL
#define EFFECT_TIMEOUT_MS 500

float device_gain = 1.0f;
unsigned long last_effect_time = 0;
bool effect_constant_active = false;
float effect_constant_nm = 0.0f;
bool effect_spring_active = false;
float effect_spring_k = 0.0f;
float effect_spring_center = 0.0f;
bool effect_damper_active = false;
float effect_damper_b = 0.0f;
bool effect_ramp_active = false;
float effect_ramp_start = 0.0f;
float effect_ramp_end = 0.0f;
float effect_ramp_duration = 0.0f;
unsigned long effect_ramp_start_time = 0;
bool effect_inertia_active = false;
float effect_inertia = 0.0f;

// Additional effects
bool effect_friction_active = false;
float effect_friction_coeff = 0.0f;
bool effect_periodic_active = false;
int effect_periodic_type = 0;
float effect_periodic_amp = 0.0f;
float effect_periodic_freq = 1.0f;
float effect_conditional_active = 0.0f;
float effect_conditional_param = 0.0f;

// Configuration
float gear_ratio = 1.0f; // motor:wheel
float max_wheel_angle_rad = 2.0f * M_PI; // default full rotation

bool ffb_verbose = false; // Disabled to avoid USB/Serial conflicts

// Effect block storage for more complete FFB handling
static EffectBlock blocks[MAX_EFFECT_BLOCKS];

// Helper: allocate or get block index
static int find_free_block() {
    for (int i = 0; i < MAX_EFFECT_BLOCKS; ++i) if (!blocks[i].in_use) return i;
    return -1;
}

static EffectBlock* get_block(uint8_t idx) {
    if (idx >= MAX_EFFECT_BLOCKS) return NULL;
    return &blocks[idx];
}

// Helper to clear a block
static void clear_block(EffectBlock *b) {
    if (!b) return;
    memset(b, 0, sizeof(EffectBlock));
}

// Compute a block's contribution to torque based on its type and current time
static float compute_block_force(EffectBlock *b, float angle, float vel) {
    if (!b || !b->in_use || !b->active) return 0.0f;
    uint32_t now = millis();
    if (b->duration_ms > 0 && (now - b->start_time_ms) >= b->duration_ms) {
        b->active = false;
        return 0.0f;
    }
    float tsec = (now - b->start_time_ms) / 1000.0f;
    float mag = b->magnitude;
    switch (b->type) {
        case ET_CONSTANT:
            if (ffb_verbose) Serial.printf("FFB: block const mag=%.3f gain=%.3f\n", mag, b->gain);
            return mag * b->gain;
        case ET_RAMP: {
            float frac = 0.0f;
            if (b->duration_ms > 0) frac = min(1.0f, (now - b->start_time_ms) / (float)b->duration_ms);
            float v = b->magnitude + (b->magnitude_end - b->magnitude) * frac;
            return v * b->gain;
        }
        case ET_SPRING: {
            float k = b->param1; // stiffness
            float center = b->param2; // center
            if (ffb_verbose) Serial.printf("FFB: block spring k=%.3f center=%.3f\n", k, center);
            return k * (center - angle) * b->gain;
        }
        case ET_DAMPER: {
            float bcoef = b->param1;
            return -bcoef * vel * b->gain;
        }
        case ET_FRICTION: {
            float coeff = fabsf(mag);
            return -coeff * (vel >= 0 ? 1.0f : -1.0f) * b->gain;
        }
        case ET_PERIODIC: {
            float phase = b->phase + 2.0f * M_PI * b->freq_hz * tsec;
            float val = 0.0f;
            // b->param1 used for type
            switch ((int)b->param1) {
                case 0: val = sinf(phase); break;
                case 1: val = (sinf(phase) >= 0) ? 1.0f : -1.0f; break;
                case 2: val = asinf(sinf(phase)) * (2.0f / M_PI); break;
                case 3: val = fmodf(phase / (2.0f * M_PI), 1.0f) * 2.0f - 1.0f; break;
            }
            if (ffb_verbose) Serial.printf("FFB: block periodic type=%d freq=%.2f amp=%.3f\n", (int)b->param1, b->freq_hz, mag);
            return val * mag * b->gain;
        }
        case ET_CONDITION: {
            // For a simple condition: use param1 as deadband/threshold modifier
            float threshold = b->param1;
            if (fabsf(angle - b->direction_rad) < threshold) return mag * b->gain;
            return 0.0f;
        }
    }
    return 0.0f;
}

float mix_effects(float angle, float vel) {
    float tau = 0.0f;
    
    // Compute effect blocks (new system)
    for (int i = 0; i < MAX_EFFECT_BLOCKS; ++i) {
        tau += compute_block_force(&blocks[i], angle, vel);
    }
    
    // Legacy effects (backward compatibility)
    if (effect_constant_active) tau += effect_constant_nm;
    if (effect_spring_active)  tau += effect_spring_k * (effect_spring_center - angle);
    if (effect_damper_active)  tau += -effect_damper_b * vel;
    if (effect_ramp_active && effect_ramp_duration > 0) {
        unsigned long now = millis();
        float t = (float)(now - effect_ramp_start_time);
        if (t >= effect_ramp_duration) {
            tau += effect_ramp_end;
            effect_ramp_active = false;
        } else {
            float ramp_val = effect_ramp_start + (effect_ramp_end - effect_ramp_start) * (t / effect_ramp_duration);
            tau += ramp_val;
        }
    }
    static float last_vel = 0.0f;
    float accel = (vel - last_vel) * CTRL_HZ;
    last_vel = vel;
    if (effect_inertia_active) {
        tau += -effect_inertia * accel;
    }
    // friction
    if (effect_friction_active) {
        tau += -effect_friction_coeff * (vel >= 0 ? 1.0f : -1.0f);
    }
    // periodic
    if (effect_periodic_active && effect_periodic_amp != 0.0f) {
        float t = (float)millis() / 1000.0f;
        float pval = 0.0f;
        float phase = 2.0f * M_PI * effect_periodic_freq * t;
        switch (effect_periodic_type) {
            case 0: pval = sinf(phase); break; // sine
            case 1: pval = (sinf(phase) >= 0) ? 1.0f : -1.0f; break; // square
            case 2: pval = asinf(sinf(phase)) * (2.0f / M_PI); break; // triangle (approx)
            case 3: pval = fmodf(phase / (2.0f * M_PI), 1.0f) * 2.0f - 1.0f; break; // saw approx
        }
        tau += effect_periodic_amp * pval;
    }
    
    // Apply device gain
    tau *= device_gain;
    
    // Timeout handling: clear both legacy flags and effect blocks
    if (millis() - last_effect_time > EFFECT_TIMEOUT_MS) {
        effect_constant_active = effect_spring_active = effect_damper_active = false;
        effect_ramp_active = false;
        effect_inertia_active = false;
        effect_friction_active = false;
        effect_periodic_active = false;
        // Clear all effect blocks
        for (int i = 0; i < MAX_EFFECT_BLOCKS; ++i) {
            clear_block(&blocks[i]);
        }
    }
    return tau;
}


// Helper to convert 16-bit signed magnitude (-32767..32767) to float in [-1..1]
static float int16_to_unit(int16_t v) {
    return (float)v / 32767.0f;
}

// Parse incoming FFB report (simple mapping). Many HID FFB reports are more complex,
// but here we support the basic magnitude/value style reports matching the descriptor.
void handle_ffb_report(uint8_t report_id, uint8_t *buf, uint16_t len) {
    // Update last effect time so mix_effects doesn't timeout
    last_effect_time = millis();
    // Many host FFB packets are structured. Here we handle several simple possibilities:
    // - If report is exactly 2 bytes -> treat as magnitude for simple effect (report id maps to type)
    // - If report is longer, try to parse a small DirectInput-style structure where bytes contain
    //   effect block index and parameters. We'll implement a compact parser for common subcommands:
    //   - 0x01: set effect block (index in buf[0]) with type in report_id, magnitude in next 2 bytes
    //   - 0x02: start effect block (index)
    //   - 0x03: stop effect block (index)
    //   - 0x04: set envelope/params (index, param1..)

    if (len >= 2) {
        // Simple 16-bit magnitude (little-endian)
    if (len == 2) {
            int16_t mag = (int16_t)((uint16_t)buf[0] | (uint16_t)buf[1] << 8);
            float val = int16_to_unit(mag);
            // map report id to simple effect
            if (ffb_verbose) Serial.printf("FFB: simple magnitude rpt=%d val=%.3f\n", report_id, val);
            switch (report_id) {
                case 2: {
                    // constant -> try block 0, else find free
                    EffectBlock *b = get_block(0);
                    if (b && !b->in_use) {
                        // use block 0
                    } else if (!b || b->in_use) {
                        // find free block
                        int i = find_free_block();
                        if (i >= 0) b = get_block(i);
                        else b = NULL;
                    }
                    if (b) {
                        b->in_use = 1; b->type = ET_CONSTANT; b->active = true;
                        b->start_time_ms = millis(); b->duration_ms = 0;
                        b->magnitude = val; b->gain = 1.0f;
                    }
                } break;
                case 3: {
                    // ramp -> try block 1, else find free
                    EffectBlock *b = get_block(1);
                    if (b && !b->in_use) {
                        // use block 1
                    } else if (!b || b->in_use) {
                        int i = find_free_block();
                        if (i >= 0) b = get_block(i);
                        else b = NULL;
                    }
                    if (b) {
                        b->in_use = 1; b->type = ET_RAMP; b->active = true;
                        b->start_time_ms = millis(); b->duration_ms = 100; // default
                        b->magnitude = val; b->magnitude_end = val; b->gain = 1.0f;
                    }
                } break;
                case 4: {
                    int i = find_free_block(); if (i >= 0) {
                        EffectBlock *b = get_block(i);
                        b->in_use = 1; b->type = ET_SPRING; b->active = true;
                        b->start_time_ms = millis(); b->duration_ms = 0;
                        b->magnitude = val; b->param1 = fabsf(val) * 20.0f; b->param2 = 0.0f; b->gain = 1.0f;
                    }
                } break;
                case 5: {
                    int i = find_free_block(); if (i >= 0) {
                        EffectBlock *b = get_block(i);
                        b->in_use = 1; b->type = ET_DAMPER; b->active = true;
                        b->start_time_ms = millis(); b->duration_ms = 0;
                        b->param1 = fabsf(val) * 10.0f; b->gain = 1.0f;
                    }
                } break;
                case 6: {
                    int i = find_free_block(); if (i >= 0) {
                        EffectBlock *b = get_block(i);
                        b->in_use = 1; b->type = ET_FRICTION; b->active = true;
                        b->start_time_ms = millis(); b->duration_ms = 0;
                        b->magnitude = val; b->gain = 1.0f;
                    }
                } break;
        case 7: case 8: case 9: case 10: {
                    int i = find_free_block(); if (i >= 0) {
                        EffectBlock *b = get_block(i);
                        b->in_use = 1; b->type = ET_PERIODIC; b->active = true;
                        b->start_time_ms = millis(); b->duration_ms = 0;
                        b->magnitude = val; b->param1 = (float)(report_id - 7); b->freq_hz = 2.0f; b->phase = 0.0f; b->gain = 1.0f;
            if (ffb_verbose) Serial.printf("FFB: created periodic block idx=%d type=%d amp=%.3f\n", i, (int)b->param1, b->magnitude);
                    }
                } break;
                case 11: {
                    int i = find_free_block(); if (i >= 0) {
                        EffectBlock *b = get_block(i);
                        b->in_use = 1; b->type = ET_CONDITION; b->active = true;
                        b->start_time_ms = millis(); b->duration_ms = 0;
                        b->magnitude = val; b->param1 = 0.1f; b->gain = 1.0f;
                    }
                } break;
            }
            return;
        }
        // If len >= 3, attempt a small structured parse: first byte = cmd, second = index
        uint8_t cmd = buf[0];
        uint8_t idx = buf[1];
        if (ffb_verbose) Serial.printf("FFB: structured cmd=0x%02X idx=%d len=%d\n", cmd, idx, len);
        switch (cmd) {
            case 0x01: { // set effect param (index, type==report_id)
                EffectBlock *b = get_block(idx);
                if (b) {
                    b->in_use = 1; b->type = (report_id == 2) ? ET_CONSTANT : (report_id == 3) ? ET_RAMP : b->type;
                    // read magnitude from buf[2..3]
                    if (len >= 4) {
                        int16_t mag = (int16_t)((uint16_t)buf[2] | (uint16_t)buf[3] << 8);
                        b->magnitude = int16_to_unit(mag);
                    }
                    // optional duration
                    if (len >= 8) {
                        uint32_t dur = (uint32_t)buf[4] | ((uint32_t)buf[5] << 8) | ((uint32_t)buf[6] << 16) | ((uint32_t)buf[7] << 24);
                        b->duration_ms = dur;
                    }
                    // If directional data is present (common in CreateEffect), parse next two bytes as direction (0..65535 polar)
                    if (len >= 10) {
                        uint16_t dir = (uint16_t)buf[8] | (uint16_t)buf[9] << 8;
                        // convert 0..65535 to radians (0..2pi) and store in direction_rad
                        float ang = ((float)dir / 65535.0f) * 2.0f * M_PI;
                        b->direction_rad = ang;
                        if (ffb_verbose) Serial.printf("FFB: block idx=%d dir_polar=%u rad=%.3f\n", idx, dir, ang);
                    }
                    b->start_time_ms = millis(); b->active = true;
                }
            } break;
            case 0x02: { // start index
                EffectBlock *b = get_block(idx);
                if (b && b->in_use) { b->active = true; b->start_time_ms = millis(); }
            } break;
            case 0x03: { // stop index
                EffectBlock *b = get_block(idx);
                if (b && b->in_use) { b->active = false; }
            } break;
            case 0x04: { // remove index
                EffectBlock *b = get_block(idx);
                if (b && b->in_use) clear_block(b);
            } break;
            case 0x05: { // set periodic params
                EffectBlock *b = get_block(idx);
                if (b && (len >= 8)) {
                    b->freq_hz = (float)buf[2];
                    b->phase = (float)buf[3] / 255.0f * 2.0f * M_PI;
                    int16_t mag = (int16_t)((uint16_t)buf[4] | (uint16_t)buf[5] << 8);
                    b->magnitude = int16_to_unit(mag);
                    b->param1 = buf[6]; // type
                    b->gain = buf[7] / 255.0f;
                    if (ffb_verbose) Serial.printf("FFB: set periodic idx=%d freq=%d phase=%.2f amp=%.3f type=%d gain=%.3f\n", idx, (int)b->freq_hz, b->phase, b->magnitude, (int)b->param1, b->gain);
                }
            } break;
            default:
                break;
        }
    }
}

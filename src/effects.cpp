#include "effects.h"
#include "hid.h"
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
float max_wheel_angle_deg = 270.0f; // default 270 degrees (typical racing wheel)
float angle_limit_stiffness = 100.0f; // Nm/rad - strength of angle limiting spring

bool ffb_verbose = false; // Disabled to avoid USB/Serial conflicts

// Effect block storage for more complete FFB handling
static EffectBlock blocks[MAX_EFFECT_BLOCKS];

// Linux hid-pidff block-load/pool state
static uint8_t pid_block_load_effect_idx = 1;      // 1..MAX_EFFECT_BLOCKS
static uint8_t pid_block_load_status = 1;          // 1=success,2=full,3=error
static uint16_t pid_ram_pool_size = 4096;
static uint16_t pid_ram_pool_available = 4096;
static uint8_t pid_simultaneous_max = MAX_EFFECT_BLOCKS;
static uint8_t pid_device_managed_pool = 1;

bool ffb_has_active_effects() {
    for (int i = 0; i < MAX_EFFECT_BLOCKS; ++i) {
        if (blocks[i].in_use && blocks[i].active) {
            return true;
        }
    }

    return effect_constant_active ||
           effect_spring_active ||
           effect_damper_active ||
           effect_ramp_active ||
           effect_inertia_active ||
           effect_friction_active ||
           effect_periodic_active;
}

// Helper: allocate or get block index
static int find_free_block() {
    for (int i = 0; i < MAX_EFFECT_BLOCKS; ++i) if (!blocks[i].in_use) return i;
    return -1;
}

static EffectBlock* get_block(uint8_t idx) {
    if (idx >= MAX_EFFECT_BLOCKS) return NULL;
    return &blocks[idx];
}

static uint8_t active_effect_count() {
    uint8_t count = 0;
    for (int i = 0; i < MAX_EFFECT_BLOCKS; ++i) {
        if (blocks[i].in_use) count++;
    }
    return count;
}

static void update_ram_pool_available() {
    // Simple accounting model: each allocated block consumes 256 bytes.
    const uint16_t bytes_per_block = 256;
    uint16_t used = active_effect_count() * bytes_per_block;
    if (used >= pid_ram_pool_size) pid_ram_pool_available = 0;
    else pid_ram_pool_available = pid_ram_pool_size - used;
}

// Helper to clear a block
static void clear_block(EffectBlock *b) {
    if (!b) return;
    memset(b, 0, sizeof(EffectBlock));
}

// Compute a block's contribution to torque based on its type and current time
static float compute_block_force(EffectBlock *b, float angle, float vel, float accel) {
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
            // magnitude is -1..1, scale to Nm (reasonable max ~10Nm for sim wheels)
            return mag * b->gain * 10.0f;
        case ET_RAMP: {
            float frac = 0.0f;
            if (b->duration_ms > 0) frac = min(1.0f, (now - b->start_time_ms) / (float)b->duration_ms);
            float v = b->magnitude + (b->magnitude_end - b->magnitude) * frac;
            return v * b->gain;
        }
        case ET_SPRING: {
            float k = b->param1; // stiffness in Nm/rad
            float center = b->param2; // center in radians
            if (ffb_verbose) Serial.printf("FFB: block spring k=%.3f center=%.3f\n", k, center);
            float error = center - angle;
            return k * error * b->gain;
        }
        case ET_DAMPER: {
            float bcoef = b->param1; // damping coefficient in Nm/(rad/s)
            // damper opposes motion
            return -bcoef * vel * b->gain;
        }
        case ET_INERTIA: {
            float mass = b->param1; // Moment of Inertia
            // inertia opposes acceleration
            return -mass * accel * b->gain;
        }
        case ET_FRICTION: {
            float coeff = b->param1 > 0.0f ? b->param1 : (fabsf(mag) * 5.0f); // friction coefficient scaled to Nm
            // friction opposes motion (velocity-independent coulomb friction)
            if (fabsf(vel) < 0.01f) return 0.0f; // deadzone to avoid chatter
            return -coeff * (vel >= 0 ? 1.0f : -1.0f) * b->gain;
        }
        case ET_PERIODIC: {
            float phase = b->phase + 2.0f * M_PI * b->freq_hz * tsec;
            float val = 0.0f;
            // b->param1 used for type: 0=sine, 1=square, 2=triangle, 3=sawup, 4=sawdown
            int ptype = (int)b->param1;
            switch (ptype) {
                case 0: val = sinf(phase); break; // sine
                case 1: val = (sinf(phase) >= 0) ? 1.0f : -1.0f; break; // square
                case 2: val = asinf(sinf(phase)) * (2.0f / M_PI); break; // triangle
                case 3: val = fmodf(phase / (2.0f * M_PI), 1.0f) * 2.0f - 1.0f; break; // saw up
                case 4: val = -(fmodf(phase / (2.0f * M_PI), 1.0f) * 2.0f - 1.0f); break; // saw down (inverted saw up)
            }
            if (ffb_verbose) Serial.printf("FFB: block periodic type=%d freq=%.2f amp=%.3f\n", ptype, b->freq_hz, mag);
            // magnitude scaled to Nm
            return val * mag * b->gain * 8.0f;
        }
        case ET_CONDITION: {
            // For a simple condition: use param1 as deadband/threshold modifier
            float threshold = b->param1;
            if (fabsf(angle - b->direction_rad) < threshold) return mag * b->gain * 8.0f;
            return 0.0f;
        }
    }
    return 0.0f;
}

float mix_effects(float angle, float vel) {
    // Compute only when there is something to play.
    if (!ffb_has_active_effects()) {
        return 0.0f; // No force if actuators disabled
    }
    
    // Calculate acceleration for Inertia effects
    static float last_vel = 0.0f;
    static unsigned long last_calc = 0;
    unsigned long now = millis();
    float dt = (now - last_calc) / 1000.0f;
    if (dt <= 0) dt = 0.001f;
    last_calc = now;
    
    float accel = (vel - last_vel) / dt; 

    // Fallback velocity from angle derivative to keep condition effects responsive
    // even if encoder-reported velocity is under-scaled or noisy.
    static float last_angle = 0.0f;
    float angle_delta = angle - last_angle;
    while (angle_delta > M_PI) angle_delta -= 2.0f * M_PI;
    while (angle_delta < -M_PI) angle_delta += 2.0f * M_PI;
    float derived_vel = angle_delta / dt;
    last_angle = angle;
    if (fabsf(derived_vel) > fabsf(vel)) {
        vel = derived_vel;
    }

    // Low pass filter acceleration to reduce noise
    static float filtered_accel = 0.0f;
    filtered_accel = 0.1f * accel + 0.9f * filtered_accel;
    last_vel = vel;
    
    float tau = 0.0f;
    
    // Angle limiting: enforce max wheel angle with spring force
    // angle is now centered (-PI..PI) where 0 = center position
    float max_angle_rad = max_wheel_angle_deg * M_PI / 180.0f;
    float half_max = max_angle_rad / 2.0f;
    
    // Apply boundary spring force if exceeding limits
    // angle > 0 is clockwise (right), angle < 0 is counter-clockwise (left)
    if (angle > half_max) {
        float excess = angle - half_max;
        tau += -angle_limit_stiffness * excess; // push back toward limit
    } else if (angle < -half_max) {
        float excess = angle + half_max;
        tau += -angle_limit_stiffness * excess; // push back toward limit
    }
    
    // Compute effect blocks (new system)
    for (int i = 0; i < MAX_EFFECT_BLOCKS; ++i) {
        tau += compute_block_force(&blocks[i], angle, vel, filtered_accel);
    }
    
    // Legacy effects (backward compatibility) - scaled properly to Nm
    if (effect_constant_active) tau += effect_constant_nm * 10.0f; // scale to Nm
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

    if (effect_inertia_active) {
        tau += -effect_inertia * filtered_accel;
    }
    // friction (legacy)
    if (effect_friction_active && fabsf(vel) > 0.01f) {
        tau += -effect_friction_coeff * 5.0f * (vel >= 0 ? 1.0f : -1.0f); // scale to Nm
    }
    // periodic (legacy)
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
        tau += effect_periodic_amp * pval * 8.0f; // scale to Nm
    }
    
    // Apply device gain
    tau *= device_gain;
    
    // Timeout handling: clear stale legacy flags only when no active PID effect exists
    if (millis() - last_effect_time > EFFECT_TIMEOUT_MS) {
        bool any_pid_active = false;
        for (int i = 0; i < MAX_EFFECT_BLOCKS; ++i) {
            if (blocks[i].in_use && blocks[i].active) {
                any_pid_active = true;
                break;
            }
        }

        if (!any_pid_active) {
            effect_constant_active = effect_spring_active = effect_damper_active = false;
            effect_ramp_active = false;
            effect_inertia_active = false;
            effect_friction_active = false;
            effect_periodic_active = false;
        }
    }
    return tau;
}


// Helper to convert 16-bit signed magnitude (-32767..32767) to float in [-1..1]
static float int16_to_unit(int16_t v) {
    return (float)v / 32767.0f;
}

// Helper to read 16-bit value from buffer (little-endian)
static int16_t read_int16(uint8_t *buf, int offset) {
    return (int16_t)((uint16_t)buf[offset] | ((uint16_t)buf[offset+1] << 8));
}

static int16_t read_int16_safe(uint8_t *buf, uint16_t len, int offset, int16_t fallback = 0) {
    if (!buf || offset < 0 || (offset + 1) >= (int)len) return fallback;
    return read_int16(buf, offset);
}

// Helper to read 8-bit unsigned value
static uint8_t read_uint8(uint8_t *buf, int offset) {
    return buf[offset];
}

static uint8_t read_uint8_safe(uint8_t *buf, uint16_t len, int offset, uint8_t fallback = 0) {
    if (!buf || offset < 0 || offset >= (int)len) return fallback;
    return read_uint8(buf, offset);
}

// Parse incoming FFB report according to the USB HID PID specification
void handle_ffb_report(uint8_t report_id, uint8_t *buf, uint16_t len) {
    // Update last effect time so mix_effects doesn't timeout
    last_effect_time = millis();

    // Parse based on Report ID from the descriptor
    switch (report_id) {
        case 0x03: { // Set Effect Report
            // Format (Linux-compatible):
            // [effect_index,
            //  duration_lo, duration_hi,
            //  gain_lo, gain_hi,
            //  trigger_button,
            //  trigger_repeat_lo, trigger_repeat_hi,
            //  dir_enable,
            //  direction_lo, direction_hi,
            //  start_delay_lo, start_delay_hi,
            //  effect_type]
            if (len < 1) break;
            
            uint8_t effect_idx = read_uint8(buf, 0);  // 1-16
            if (effect_idx < 1 || effect_idx > MAX_EFFECT_BLOCKS) break;
            effect_idx--; // Convert to 0-15 array index
            
            EffectBlock *b = get_block(effect_idx);
            if (!b) break;
            
            uint16_t duration = (uint16_t)read_int16_safe(buf, len, 1, 0);
            uint16_t gain16 = (uint16_t)read_int16_safe(buf, len, 3, 65535);
            uint16_t direction = (uint16_t)read_int16_safe(buf, len, 9, 0);
            uint8_t effect_type = read_uint8_safe(buf, len, 13, b->type ? b->type : 4); // 1..11, default sine-like
            
            // Initialize the effect block
            b->in_use = 1;
            
            // Map USB HID effect types to internal EffectType
            switch (effect_type) {
                case 1: b->type = ET_CONSTANT; break;
                case 2: b->type = ET_RAMP; break;
                case 3: b->type = ET_PERIODIC; b->param1 = 1; break; // Square
                case 4: b->type = ET_PERIODIC; b->param1 = 0; break; // Sine
                case 5: b->type = ET_PERIODIC; b->param1 = 2; break; // Triangle
                case 6: b->type = ET_PERIODIC; b->param1 = 3; break; // SawUp
                case 7: b->type = ET_PERIODIC; b->param1 = 4; break; // SawDown
                case 8: b->type = ET_SPRING; break;
                case 9: b->type = ET_DAMPER; break;
                case 10: b->type = ET_INERTIA; break;
                case 11: b->type = ET_FRICTION; break;
                default: b->type = ET_NONE; break;
            }
            
            b->active = false; // Wait for Effect Operation command to start
            // Host duration unit scaling is descriptor-dependent and many stacks send values
            // that are not directly milliseconds. Keep effects persistent by default and rely
            // on Effect Operation STOP / Block Free from host to end playback.
            // Explicitly keep HID "infinite" sentinels as persistent too.
            if (duration == 0 || duration == 0xFFFF || duration == 0x7FFF) {
                b->duration_ms = 0;
            } else if (duration < 32) {
                // Heuristic: tiny values commonly represent seconds after host scaling.
                b->duration_ms = duration * 1000UL;
            } else {
                b->duration_ms = duration;
            }
            b->gain = gain16 / 65535.0f;
            b->start_time_ms = 0;
            // Linux direction is 0..65535, map around circle to radians
            b->direction_rad = (direction / 65535.0f) * (2.0f * M_PI);
            
            if (ffb_verbose) {
                Serial.printf("FFB: Set Effect idx=%d type=%d -> %d dur=%d gain=%.2f dir=%.2f\n", 
                    effect_idx, effect_type, b->type, duration, b->gain, b->direction_rad);
            }
        } break;

        case 0x04: { // Set Envelope Report
            // Format: [effect_index, attack_level, attack_time_lo, attack_time_hi,
            //          fade_level, fade_time_lo, fade_time_hi]
            if (len < 7) break;
            
            uint8_t effect_idx = read_uint8(buf, 0);
            if (effect_idx < 1 || effect_idx > MAX_EFFECT_BLOCKS) break;
            effect_idx--;
            
            // For now, we'll store envelope in param fields (can be expanded later)
            EffectBlock *b = get_block(effect_idx);
            if (!b || !b->in_use) break;
            
            // Could implement envelope shaping here
            // For simplicity, we'll just note it was set
        } break;

        case 0x05: { // Set Condition Report
            // Format: [effect_index, param_block_offset,
            //          cp_offset_lo, cp_offset_hi,
            //          pos_coeff_lo, pos_coeff_hi,
            //          neg_coeff_lo, neg_coeff_hi,
            //          pos_sat_lo, pos_sat_hi,
            //          neg_sat_lo, neg_sat_hi,
            //          deadba                                                                                                                                                                                                                                                                                                              nd_lo, deadband_hi]
            if (len < 8) break;

            uint8_t effect_idx = read_uint8(buf, 0);
            if (effect_idx < 1 || effect_idx > MAX_EFFECT_BLOCKS) break;
            effect_idx--;

            EffectBlock *b = get_block(effect_idx);
            if (!b || !b->in_use) break;                                                                                                                                                                    

            int16_t cp_offset = read_int16_safe(buf, len, 2, 0);
            int16_t pos_coeff = read_int16_safe(buf, len, 4, 0);
            int16_t neg_coeff = read_int16_safe(buf, len, 6, 0);
            int16_t pos_sat = read_int16_safe(buf, len, 8, 32767);
            int16_t neg_sat = read_int16_safe(buf, len, 10, 32767);
            int16_t deadband = read_int16_safe(buf, len, 12, 0);

            float coeff_abs = fmaxf(fabsf(int16_to_unit(pos_coeff)), fabsf(int16_to_unit(neg_coeff)));
            float sat_abs = fmaxf(fabsf(int16_to_unit(pos_sat)), fabsf(int16_to_unit(neg_sat)));
            float deadband_rad = fabsf(int16_to_unit(deadband)) * M_PI;

            if (b->type == ET_SPRING) {
                b->param1 = coeff_abs * 180.0f; // Nm/rad
                b->param2 = int16_to_unit(cp_offset) * M_PI;
            } else if (b->type == ET_DAMPER) {
                b->param1 = coeff_abs * 90.0f; // Nm/(rad/s)
            } else if (b->type == ET_FRICTION) {
                b->param1 = coeff_abs * 12.0f; // Nm static friction
            } else if (b->type == ET_INERTIA) {
                b->param1 = coeff_abs * 6.0f; // inertia-like torque scaling
            }

            // Store optional common condition limits in generic fields.
            b->magnitude = sat_abs;
            b->magnitude_end = deadband_rad;

            if (ffb_verbose) {
                Serial.printf("FFB: Set Condition idx=%d type=%d cp=%d pos=%d neg=%d ps=%d ns=%d db=%d\n",
                    effect_idx, b->type, cp_offset, pos_coeff, neg_coeff, pos_sat, neg_sat, deadband);
            }
        } break;

        case 0x06: { // Set Periodic Report
            // Format: [effect_index, magnitude_lo, magnitude_hi, offset_lo, offset_hi,
            //          phase_lo, phase_hi, period_lo, period_hi]
            if (len < 3) break;

            uint8_t effect_idx = read_uint8(buf, 0);
            if (effect_idx < 1 || effect_idx > MAX_EFFECT_BLOCKS) break;
            effect_idx--;

            EffectBlock *b = get_block(effect_idx);
            if (!b || !b->in_use) break;

            int16_t magnitude = read_int16_safe(buf, len, 1, 0);
            int16_t offset = read_int16_safe(buf, len, 3, 0);
            uint16_t phase = (uint16_t)read_int16_safe(buf, len, 5, 0);
            uint16_t period = (uint16_t)read_int16_safe(buf, len, 7, 0);

            b->type = ET_PERIODIC;
            b->magnitude = int16_to_unit(magnitude);
            b->phase = (phase / 65535.0f) * 2.0f * M_PI;
            // If host/unit conversion collapses period to 0, use a rumble-like fallback.
            b->freq_hz = (period > 0) ? (1000.0f / period) : 80.0f;

            if (ffb_verbose) {
                Serial.printf("FFB: Set Periodic idx=%d mag=%.3f off=%.3f freq=%.2f phase=%.2f\n",
                    effect_idx, b->magnitude, int16_to_unit(offset), b->freq_hz, b->phase);
            }
        } break;

        case 0x07: { // Set Constant Force Report
            // Format: [effect_index, magnitude_lo, magnitude_hi]
            if (len < 3) break;
            
            uint8_t effect_idx = read_uint8(buf, 0);
            if (effect_idx < 1 || effect_idx > MAX_EFFECT_BLOCKS) break;
            effect_idx--;
            
            EffectBlock *b = get_block(effect_idx);
            if (!b || !b->in_use) break;
            
            int16_t magnitude = read_int16(buf, 1);
            b->magnitude = int16_to_unit(magnitude);
            b->type = ET_CONSTANT;
            
            if (ffb_verbose) {
                Serial.printf("FFB: Set Constant idx=%d mag=%.3f\n", effect_idx, b->magnitude);
            }
        } break;

        case 0x08: { // Set Ramp Force Report
            // Format: [effect_index, ramp_start_lo, ramp_start_hi, ramp_end_lo, ramp_end_hi]
            if (len < 5) break;
            
            uint8_t effect_idx = read_uint8(buf, 0);
            if (effect_idx < 1 || effect_idx > MAX_EFFECT_BLOCKS) break;
            effect_idx--;
            
            EffectBlock *b = get_block(effect_idx);
            if (!b || !b->in_use) break;
            
            int16_t ramp_start = read_int16(buf, 1);
            int16_t ramp_end = read_int16(buf, 3);
            b->type = ET_RAMP;
            b->magnitude = int16_to_unit(ramp_start);
            b->magnitude_end = int16_to_unit(ramp_end);
            if (ffb_verbose) {
                Serial.printf("FFB: Set Ramp idx=%d start=%.3f end=%.3f\n",
                    effect_idx, b->magnitude, b->magnitude_end);
            }
        } break;

        case 0x09: { // Effect Operation Report (Start/Stop)
            // Format: [effect_index, operation, loop_count]
            if (len < 2) break;
            
            uint8_t effect_idx = read_uint8(buf, 0);
            if (effect_idx < 1 || effect_idx > MAX_EFFECT_BLOCKS) break;
            effect_idx--;
            
            uint8_t operation = read_uint8(buf, 1); // 1=Start, 2=Stop
            uint8_t loop_count = read_uint8_safe(buf, len, 2, 0);
            
            EffectBlock *b = get_block(effect_idx);
            if (!b || !b->in_use) break;
            
            if (operation == 1) { // Start
                b->active = true;
                b->start_time_ms = millis();
                
                // Update global effect_playing for PID State report
                extern uint8_t effect_playing;
                effect_playing = effect_idx + 1;
                
                if (ffb_verbose) {
                    Serial.printf("FFB: Start Effect idx=%d loops=%u\n", effect_idx, loop_count);
                }
            } else if (operation == 2) { // Stop
                b->active = false;
                
                // Check if any effects still playing
                extern uint8_t effect_playing;
                effect_playing = 0;
                for (int i = 0; i < MAX_EFFECT_BLOCKS; i++) {
                    if (blocks[i].in_use && blocks[i].active) {
                        effect_playing = i + 1;
                        break;
                    }
                }
                
                if (ffb_verbose) {
                    Serial.printf("FFB: Stop Effect idx=%d\n", effect_idx);
                }
            }
        } break;

        case 0x0A: { // Block Free Report
            // Format: [effect_index]
            if (len < 1) break;
            
            uint8_t effect_idx = read_uint8(buf, 0);
            if (effect_idx < 1 || effect_idx > MAX_EFFECT_BLOCKS) break;
            effect_idx--;
            
            EffectBlock *b = get_block(effect_idx);
            if (b) {
                clear_block(b);
                update_ram_pool_available();
                
                if (ffb_verbose) {
                    Serial.printf("FFB: Block Free idx=%d\n", effect_idx);
                }
            }
        } break;

        case 0x0B: { // Device Control Report
            // Format: [control] - 1=Enable, 2=Disable, 3=Stop All, 4=Reset, 5=Pause, 6=Continue
            if (len < 1) break;
            
            uint8_t control = read_uint8(buf, 0);
            extern bool actuators_enabled;
            
            switch (control) {
                case 1: // Enable Actuators - CRITICAL
                    actuators_enabled = true;
                    if (ffb_verbose) Serial.println("FFB: Actuators ENABLED");
                    break;
                    
                case 2: // Disable Actuators
                    actuators_enabled = false;
                    if (ffb_verbose) Serial.println("FFB: Actuators DISABLED");
                    break;
                    
                case 3: // Stop All Effects
                    for (int i = 0; i < MAX_EFFECT_BLOCKS; i++) {
                        if (blocks[i].in_use) blocks[i].active = false;
                    }
                    effect_playing = 0;
                    if (ffb_verbose) Serial.println("FFB: Stop All Effects");
                    break;
                    
                case 4: // Device Reset
                    for (int i = 0; i < MAX_EFFECT_BLOCKS; i++) {
                        clear_block(&blocks[i]);
                    }
                    device_gain = 1.0f;
                    actuators_enabled = true;
                    effect_playing = 0;
                    update_ram_pool_available();
                    if (ffb_verbose) Serial.println("FFB: Device Reset");
                    break;
                    
                case 5: // Pause
                    // Could implement pausing logic
                    break;
                    
                case 6: // Continue
                    // Resume from pause
                    break;
            }
        } break;

        case 0x0C: { // Device Gain Report
            // Format: [gain_lo, gain_hi]
            if (len < 2) break;
            uint16_t gain16 = (uint16_t)read_int16(buf, 0);
            device_gain = gain16 / 65535.0f;
            
            if (ffb_verbose) {
                Serial.printf("FFB: Device Gain set to %.2f\n", device_gain);
            }
        } break;

        case 0x0D: { // Create New Effect (Feature SET)
            // Format: [report-level byte, effect_type]
            // Host implementations vary; detect any valid type byte (1..11).
            if (len < 1) break;
            uint8_t effect_type = 0;
            for (uint16_t i = 0; i < len; ++i) {
                uint8_t candidate = read_uint8(buf, i);
                if (candidate >= 1 && candidate <= 11) {
                    effect_type = candidate;
                    break;
                }
            }
            if (effect_type == 0) {
                pid_block_load_status = 3; // error
                pid_block_load_effect_idx = 1;
                if (ffb_verbose) {
                    Serial.printf("FFB: Create New Effect invalid payload len=%u\n", (unsigned)len);
                }
                break;
            }

            int free_idx = find_free_block();
            if (free_idx < 0) {
                pid_block_load_status = 2; // full
                pid_block_load_effect_idx = 1;
                update_ram_pool_available();
                break;
            }

            EffectBlock *b = get_block((uint8_t)free_idx);
            if (!b) {
                pid_block_load_status = 3; // error
                pid_block_load_effect_idx = 1;
                break;
            }

            clear_block(b);
            b->in_use = 1;
            b->active = false;
            b->gain = 1.0f;
            b->duration_ms = 0;

            switch (effect_type) {
                case 1: b->type = ET_CONSTANT; break;
                case 2: b->type = ET_RAMP; break;
                case 3: b->type = ET_PERIODIC; b->param1 = 1; break;
                case 4: b->type = ET_PERIODIC; b->param1 = 0; break;
                case 5: b->type = ET_PERIODIC; b->param1 = 2; break;
                case 6: b->type = ET_PERIODIC; b->param1 = 3; break;
                case 7: b->type = ET_PERIODIC; b->param1 = 4; break;
                case 8: b->type = ET_SPRING; break;
                case 9: b->type = ET_DAMPER; break;
                case 10: b->type = ET_INERTIA; break;
                case 11: b->type = ET_FRICTION; break;
                default: b->type = ET_NONE; break;
            }

            pid_block_load_effect_idx = (uint8_t)(free_idx + 1);
            pid_block_load_status = (b->type == ET_NONE) ? 3 : 1;
            update_ram_pool_available();

            if (ffb_verbose) {
                Serial.printf("FFB: Create New Effect type=%u -> idx=%u status=%u\n",
                    effect_type, pid_block_load_effect_idx, pid_block_load_status);
            }
        } break;

        default:
            // Unknown report ID
            if (ffb_verbose) {
                Serial.printf("FFB: Unknown report ID 0x%02X len=%d\n", report_id, len);
            }
            break;
    }
}

uint16_t ffb_get_feature_report(uint8_t report_id, uint8_t *buffer, uint16_t len) {
    if (!buffer || len == 0) return 0;

    // Report ID 2: PID State
    if (report_id == 0x02) {
        if (len < 2) return 0;
        uint8_t status = 0;
        uint8_t playing = effect_playing;
        if (playing > MAX_EFFECT_BLOCKS) playing = 0;
        if (actuators_enabled) status |= 0x01;
        if (safety_switch) status |= 0x02;
        if (playing > 0) status |= 0x04;
        buffer[0] = status;
        buffer[1] = playing;
        return 2;
    }

    // Report ID 0x0E: Block Load Report
    if (report_id == 0x0E) {
        if (len < 4) return 0;
        uint8_t idx = pid_block_load_effect_idx;
        if (idx < 1 || idx > MAX_EFFECT_BLOCKS) idx = 1;
        uint8_t status = pid_block_load_status;
        if (status < 1 || status > 3) status = 3;
        buffer[0] = idx;
        buffer[1] = status;
        buffer[2] = (uint8_t)(pid_ram_pool_available & 0xFF);
        buffer[3] = (uint8_t)((pid_ram_pool_available >> 8) & 0xFF);
        return 4;
    }

    // Report ID 0x0F: PID Pool Report
    if (report_id == 0x0F) {
        if (len < 4) return 0;
        buffer[0] = (uint8_t)(pid_ram_pool_size & 0xFF);
        buffer[1] = (uint8_t)((pid_ram_pool_size >> 8) & 0xFF);
        uint8_t max_sim = pid_simultaneous_max;
        if (max_sim < 1 || max_sim > MAX_EFFECT_BLOCKS) max_sim = MAX_EFFECT_BLOCKS;
        buffer[2] = max_sim;
        buffer[3] = pid_device_managed_pool ? 1 : 0;
        return 4;
    }

    return 0;
}

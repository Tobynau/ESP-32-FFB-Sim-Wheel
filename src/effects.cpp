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
        case ET_FRICTION: {
            float coeff = fabsf(mag) * 5.0f; // friction coefficient scaled to Nm
            // friction opposes motion (velocity-independent coulomb friction)
            if (fabsf(vel) < 0.01f) return 0.0f; // deadzone to avoid chatter
            return -coeff * (vel >= 0 ? 1.0f : -1.0f) * b->gain;
        }
        case ET_PERIODIC: {
            float phase = b->phase + 2.0f * M_PI * b->freq_hz * tsec;
            float val = 0.0f;
            // b->param1 used for type: 0=sine, 1=square, 2=triangle, 3=sawtooth
            switch ((int)b->param1) {
                case 0: val = sinf(phase); break;
                case 1: val = (sinf(phase) >= 0) ? 1.0f : -1.0f; break;
                case 2: val = asinf(sinf(phase)) * (2.0f / M_PI); break;
                case 3: val = fmodf(phase / (2.0f * M_PI), 1.0f) * 2.0f - 1.0f; break;
            }
            if (ffb_verbose) Serial.printf("FFB: block periodic type=%d freq=%.2f amp=%.3f\n", (int)b->param1, b->freq_hz, mag);
            // magnitude scaled to Nm
            return val * mag * b->gain * 8.0f;
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
    // Check if actuators are enabled
    extern bool actuators_enabled;
    if (!actuators_enabled) {
        return 0.0f; // No force if actuators disabled
    }
    
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
        tau += compute_block_force(&blocks[i], angle, vel);
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
    static float last_vel = 0.0f;
    float accel = (vel - last_vel) * CTRL_HZ;
    last_vel = vel;
    if (effect_inertia_active) {
        tau += -effect_inertia * accel;
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

// Helper to read 16-bit value from buffer (little-endian)
static int16_t read_int16(uint8_t *buf, int offset) {
    return (int16_t)((uint16_t)buf[offset] | ((uint16_t)buf[offset+1] << 8));
}

// Helper to read 8-bit unsigned value
static uint8_t read_uint8(uint8_t *buf, int offset) {
    return buf[offset];
}

// Parse incoming FFB report according to the USB HID PID specification
void handle_ffb_report(uint8_t report_id, uint8_t *buf, uint16_t len) {
    // Update last effect time so mix_effects doesn't timeout
    last_effect_time = millis();

    // Parse based on Report ID from the descriptor
    switch (report_id) {
        case 0x03: { // Set Effect Report
            // Format: [effect_index, effect_type, duration_lo, duration_hi, trigger_button, 
            //          sample_period_lo, sample_period_hi, gain]
            if (len < 8) break;
            
            uint8_t effect_idx = read_uint8(buf, 0);  // 1-16
            if (effect_idx < 1 || effect_idx > MAX_EFFECT_BLOCKS) break;
            effect_idx--; // Convert to 0-15 array index
            
            EffectBlock *b = get_block(effect_idx);
            if (!b) break;
            
            uint8_t effect_type = read_uint8(buf, 1); // 1=constant, 2=ramp, etc.
            uint16_t duration = (uint16_t)read_int16(buf, 2);
            uint8_t gain = read_uint8(buf, 7);
            
            // Initialize the effect block
            b->in_use = 1;
            b->type = effect_type; // Map directly: 1=constant, 3=spring, 4=damper, etc.
            b->active = false; // Wait for Effect Operation command to start
            b->duration_ms = duration; // 0 = infinite
            b->gain = gain / 255.0f;
            b->start_time_ms = 0;
            
            if (ffb_verbose) {
                Serial.printf("FFB: Set Effect idx=%d type=%d dur=%d gain=%.2f\n", 
                    effect_idx, effect_type, duration, b->gain);
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

        case 0x05: { // Set Constant Force Report
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

        case 0x06: { // Set Condition Report (Spring, Damper, Friction, Inertia)
            // Format: [effect_index, param_block_offset:4, type_specific_offset:4,
            //          cp_offset_lo, cp_offset_hi, pos_coeff_lo, pos_coeff_hi,
            //          neg_coeff_lo, neg_coeff_hi, pos_sat_lo, pos_sat_hi,
            //          neg_sat_lo, neg_sat_hi, deadband_lo, deadband_hi]
            if (len < 15) break;
            
            uint8_t effect_idx = read_uint8(buf, 0);
            if (effect_idx < 1 || effect_idx > MAX_EFFECT_BLOCKS) break;
            effect_idx--;
            
            EffectBlock *b = get_block(effect_idx);
            if (!b || !b->in_use) break;
            
            // Parse condition parameters
            int16_t cp_offset = read_int16(buf, 2);
            int16_t pos_coeff = read_int16(buf, 4);
            int16_t neg_coeff = read_int16(buf, 6);
            int16_t pos_sat = read_int16(buf, 8);
            int16_t neg_sat = read_int16(buf, 10);
            int16_t deadband = read_int16(buf, 12);
            
            // Store parameters based on effect type
            if (b->type == ET_SPRING) {
                // Spring: stiffness in Nm/rad, center in radians
                b->param1 = fabsf(int16_to_unit(pos_coeff)) * 80.0f; // stiffness (Nm/rad)
                b->param2 = int16_to_unit(cp_offset) * M_PI;  // center position in radians
            } else if (b->type == ET_DAMPER) {
                // Damper: coefficient in Nm/(rad/s)
                b->param1 = fabsf(int16_to_unit(pos_coeff)) * 15.0f; // damping (Nm/(rad/s))
            } else if (b->type == ET_FRICTION) {
                // Friction: magnitude in Nm
                b->magnitude = int16_to_unit(pos_coeff);
            }
            
            if (ffb_verbose) {
                Serial.printf("FFB: Set Condition idx=%d cp=%d pos=%d neg=%d\n",
                    effect_idx, cp_offset, pos_coeff, neg_coeff);
            }
        } break;

        case 0x07: { // Set Periodic Report
            // Format: [effect_index, magnitude_lo, magnitude_hi, offset_lo, offset_hi,
            //          phase, period_lo, period_hi]
            if (len < 8) break;
            
            uint8_t effect_idx = read_uint8(buf, 0);
            if (effect_idx < 1 || effect_idx > MAX_EFFECT_BLOCKS) break;
            effect_idx--;
            
            EffectBlock *b = get_block(effect_idx);
            if (!b || !b->in_use) break;
            
            int16_t magnitude = read_int16(buf, 1);
            int16_t offset = read_int16(buf, 3);
            uint8_t phase = read_uint8(buf, 5);
            uint16_t period = (uint16_t)read_int16(buf, 6);
            
            b->type = ET_PERIODIC;
            b->magnitude = int16_to_unit(magnitude);
            b->phase = (phase / 255.0f) * 2.0f * M_PI;
            b->freq_hz = (period > 0) ? (1000.0f / period) : 1.0f; // period in ms
            
            // Determine wave type based on effect type from Set Effect
            // param1 stores waveform: 0=sine, 1=square, 2=triangle, 3=sawtooth
            
            if (ffb_verbose) {
                Serial.printf("FFB: Set Periodic idx=%d mag=%.3f freq=%.2f phase=%.2f\n",
                    effect_idx, b->magnitude, b->freq_hz, b->phase);
            }
        } break;

        case 0x08: { // Effect Operation Report (Start/Stop)
            // Format: [effect_index, operation, loop_count]
            if (len < 3) break;
            
            uint8_t effect_idx = read_uint8(buf, 0);
            if (effect_idx < 1 || effect_idx > MAX_EFFECT_BLOCKS) break;
            effect_idx--;
            
            uint8_t operation = read_uint8(buf, 1); // 1=Start, 2=Start Solo, 3=Stop
            uint8_t loop_count = read_uint8(buf, 2);
            
            EffectBlock *b = get_block(effect_idx);
            if (!b || !b->in_use) break;
            
            if (operation == 1 || operation == 2) { // Start or Start Solo
                if (operation == 2) { // Solo - stop all other effects
                    for (int i = 0; i < MAX_EFFECT_BLOCKS; i++) {
                        if (i != effect_idx && blocks[i].in_use) {
                            blocks[i].active = false;
                        }
                    }
                }
                b->active = true;
                b->start_time_ms = millis();
                
                // Update global effect_playing for PID State report
                extern uint8_t effect_playing;
                effect_playing = effect_idx + 1;
                
                if (ffb_verbose) {
                    Serial.printf("FFB: Start Effect idx=%d solo=%d\n", effect_idx, operation == 2);
                }
            } else if (operation == 3) { // Stop
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

        case 0x09: { // Block Free Report
            // Format: [effect_index]
            if (len < 1) break;
            
            uint8_t effect_idx = read_uint8(buf, 0);
            if (effect_idx < 1 || effect_idx > MAX_EFFECT_BLOCKS) break;
            effect_idx--;
            
            EffectBlock *b = get_block(effect_idx);
            if (b) {
                clear_block(b);
                
                if (ffb_verbose) {
                    Serial.printf("FFB: Block Free idx=%d\n", effect_idx);
                }
            }
        } break;

        case 0x0A: { // Device Control Report
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

        case 0x0B: { // Device Gain Report
            // Format: [gain] (0-255)
            if (len < 1) break;
            
            uint8_t gain = read_uint8(buf, 0);
            device_gain = gain / 255.0f;
            
            if (ffb_verbose) {
                Serial.printf("FFB: Device Gain set to %.2f\n", device_gain);
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

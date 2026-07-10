#include "effects.h"
#include "hid.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

bool ffb_verbose = false;
float device_gain = 1.0f;
bool device_paused = false;
volatile float g_ffb_torque_nm = 0.0f;

// Normalization full-scales for condition-effect metrics. The PID spec works
// in normalized units; these define what "1.0" means physically on this wheel.
static constexpr float DAMPER_VEL_FS = 2.0f * (float)M_PI;  // rad/s -> full damper force
static constexpr float INERTIA_ACC_FS = 40.0f;              // rad/s^2 -> full inertia force
static constexpr float FRICTION_V_KNEE = 0.08f;             // rad/s, friction saturation knee

// Host-silence watchdog: if host-driven forces are active but no FFB report
// has arrived for this long, fade them out (springs/dampers persist — they
// are computed locally from position and are safe without host updates).
static constexpr uint32_t HOST_SILENCE_MS = 2000;
static constexpr float FADE_OUT_S = 0.2f;
static constexpr float FADE_IN_S = 0.05f;

static EffectBlock blocks[MAX_EFFECT_BLOCKS];
static SemaphoreHandle_t block_mutex = nullptr;

static uint32_t last_host_report_ms = 0;
static uint32_t pause_started_ms = 0;
static uint32_t paused_total_ms = 0;

// Linux hid-pidff block-load/pool state
static uint8_t pid_block_load_effect_idx = 1;      // 1..MAX_EFFECT_BLOCKS
static uint8_t pid_block_load_status = 1;          // 1=success,2=full,3=error
static uint16_t pid_ram_pool_size = 4096;
static uint16_t pid_ram_pool_available = 4096;
static uint8_t pid_simultaneous_max = MAX_EFFECT_BLOCKS;
static uint8_t pid_device_managed_pool = 1;

// Pipeline state (only touched from the FFB task)
static float recon_state = 0.0f;      // reconstruction LPF on host magnitudes
static float slew_state = 0.0f;       // last torque out of the slew limiter
static float host_fade = 1.0f;        // watchdog fade factor 0..1
static float filtered_accel = 0.0f;
static float last_vel = 0.0f;
static float osc_level = 0.0f;        // oscillation-guard engagement 0..1

static inline void lock_blocks() {
    if (block_mutex) xSemaphoreTake(block_mutex, portMAX_DELAY);
}
static inline void unlock_blocks() {
    if (block_mutex) xSemaphoreGive(block_mutex);
}

// Effects clock: freezes while the device is paused so effect timelines
// resume where they left off (PID Device Pause / Continue).
static inline uint32_t eff_now_ms() {
    uint32_t now = millis() - paused_total_ms;
    if (device_paused) now -= (millis() - pause_started_ms);
    return now;
}

void effects_init() {
    if (!block_mutex) block_mutex = xSemaphoreCreateMutex();
    last_host_report_ms = millis();
}

uint32_t effects_last_host_ms() {
    return last_host_report_ms;
}

static uint8_t active_effect_count() {
    uint8_t count = 0;
    for (int i = 0; i < MAX_EFFECT_BLOCKS; ++i) {
        if (blocks[i].in_use) count++;
    }
    return count;
}

uint8_t effects_get_active_block_count() {
    return active_effect_count();
}

uint8_t effects_get_status_flags() {
    uint8_t f = 0;
    if (actuators_enabled) f |= 0x01;
    if (device_paused) f |= 0x02;
    if (hid_usb_active()) f |= 0x04;
    if (host_fade < 0.99f) f |= 0x08;
    return f;
}

bool ffb_has_active_effects() {
    for (int i = 0; i < MAX_EFFECT_BLOCKS; ++i) {
        if (blocks[i].in_use && blocks[i].active) return true;
    }
    return false;
}

void effects_all_stop() {
    lock_blocks();
    for (int i = 0; i < MAX_EFFECT_BLOCKS; ++i) {
        blocks[i].active = false;
    }
    unlock_blocks();
    effect_playing = 0;
    hid_notify_pid_state_changed();
}

static int find_free_block() {
    for (int i = 0; i < MAX_EFFECT_BLOCKS; ++i) if (!blocks[i].in_use) return i;
    return -1;
}

static EffectBlock *get_block(uint8_t idx) {
    if (idx >= MAX_EFFECT_BLOCKS) return NULL;
    return &blocks[idx];
}

static void update_ram_pool_available() {
    // Simple accounting model: each allocated block consumes 256 bytes.
    const uint16_t bytes_per_block = 256;
    uint16_t used = active_effect_count() * bytes_per_block;
    if (used >= pid_ram_pool_size) pid_ram_pool_available = 0;
    else pid_ram_pool_available = pid_ram_pool_size - used;
}

static void clear_block(EffectBlock *b) {
    if (!b) return;
    memset(b, 0, sizeof(EffectBlock));
}

static void refresh_effect_playing() {
    effect_playing = 0;
    for (int i = 0; i < MAX_EFFECT_BLOCKS; i++) {
        if (blocks[i].in_use && blocks[i].active) {
            effect_playing = i + 1;
            break;
        }
    }
}

static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// This wheel has a single FFB axis, so the only usable information in the
// PID direction angle is which way the force points along X. PID/DirectInput
// convention: 0 = north, 90deg = +X (right). Hosts that encode sign in the
// magnitude typically send an axis-degenerate direction (0/180), which we
// treat as "trust the signed magnitude". Global inversion for hosts with the
// opposite convention is handled by g_cfg.invert_ffb.
static float direction_sign(const EffectBlock *b) {
    if (!b->direction_enable) return 1.0f;
    float s = sinf(b->direction_rad);
    if (fabsf(s) < 0.05f) return 1.0f;
    return (s > 0.0f) ? 1.0f : -1.0f;
}

// Envelope scaling per the PID spec: magnitude interpolates from attack_level
// at start and to fade_level over the last fade_time of a finite duration.
static float envelope_scale(const EffectBlock *b, uint32_t t_active_ms) {
    float s = 1.0f;
    if (b->attack_time_ms > 0 && t_active_ms < b->attack_time_ms) {
        float f = (float)t_active_ms / (float)b->attack_time_ms;
        s = b->attack_level + (1.0f - b->attack_level) * f;
    }
    if (b->duration_ms > 0 && b->fade_time_ms > 0) {
        uint32_t fade_start = (b->duration_ms > b->fade_time_ms)
                                  ? (b->duration_ms - b->fade_time_ms) : 0;
        if (t_active_ms >= fade_start) {
            float f = (float)(t_active_ms - fade_start) / (float)b->fade_time_ms;
            if (f > 1.0f) f = 1.0f;
            float fs = 1.0f + (b->fade_level - 1.0f) * f;
            if (fs < s) s = fs;
        }
    }
    return s;
}

// PID condition model: force = -coeff * (metric - offset +/- deadband),
// clamped by the side's saturation. All inputs normalized to the metric's
// full scale; returns normalized force -1..1.
static float condition_force(const EffectBlock *b, float q_norm) {
    float x = q_norm - b->cp_offset;
    float t = 0.0f;
    if (x > b->deadband) {
        t = -b->pos_coeff * (x - b->deadband);
        float sat = b->pos_sat > 0.0f ? b->pos_sat : 1.0f;
        t = clampf(t, -sat, sat);
    } else if (x < -b->deadband) {
        t = -b->neg_coeff * (x + b->deadband);
        float sat = b->neg_sat > 0.0f ? b->neg_sat : 1.0f;
        t = clampf(t, -sat, sat);
    }
    return t;
}

static float wave_value(uint8_t wave, float phase) {
    switch (wave) {
        case WAVE_SINE:     return sinf(phase);
        case WAVE_SQUARE:   return (sinf(phase) >= 0.0f) ? 1.0f : -1.0f;
        case WAVE_TRIANGLE: return asinf(sinf(phase)) * (2.0f / (float)M_PI);
        case WAVE_SAW_UP: {
            float f = fmodf(phase / (2.0f * (float)M_PI), 1.0f);
            if (f < 0.0f) f += 1.0f;
            return f * 2.0f - 1.0f;
        }
        case WAVE_SAW_DOWN: {
            float f = fmodf(phase / (2.0f * (float)M_PI), 1.0f);
            if (f < 0.0f) f += 1.0f;
            return -(f * 2.0f - 1.0f);
        }
    }
    return 0.0f;
}

// Compute one block's contribution. Host-magnitude forces (constant/ramp) go
// to *host_out so the reconstruction filter can smooth the host's update
// staircase; locally-synthesized periodic goes to *periodic_out; condition
// forces (computed from live position/velocity) go to *cond_out. Outputs are
// normalized (-1..1 of max torque).
static void compute_block(EffectBlock *b, float angle, float vel, float accel,
                          float half_range, float dt, uint32_t now,
                          float *host_out, float *periodic_out, float *cond_out) {
    if (!b->in_use || !b->active) return;

    uint32_t t = now - b->start_time_ms;
    if (t < b->start_delay_ms) return;
    uint32_t ta = t - b->start_delay_ms;

    if (b->duration_ms > 0 && ta >= b->duration_ms) {
        if (b->loop_count == 0xFF || b->loop_count > 1) {
            if (b->loop_count != 0xFF) b->loop_count--;
            b->start_time_ms = now - b->start_delay_ms;
            b->phase_acc = 0.0f;
            ta = 0;
        } else {
            b->active = false;
            return;
        }
    }

    float env = envelope_scale(b, ta);
    float dsign = direction_sign(b);

    switch (b->type) {
        case ET_CONSTANT:
            *host_out += b->magnitude * env * b->gain * dsign;
            break;
        case ET_RAMP: {
            float frac = (b->duration_ms > 0) ? ((float)ta / (float)b->duration_ms) : 0.0f;
            if (frac > 1.0f) frac = 1.0f;
            float v = b->magnitude + (b->magnitude_end - b->magnitude) * frac;
            *host_out += v * env * b->gain * dsign;
            break;
        }
        case ET_PERIODIC: {
            b->phase_acc += 2.0f * (float)M_PI * b->freq_hz * dt;
            // Wrap early: float resolution above ~1e3 rad would add phase jitter.
            if (b->phase_acc > 128.0f * (float)M_PI) {
                b->phase_acc = fmodf(b->phase_acc, 2.0f * (float)M_PI);
            }
            float w = wave_value(b->wave, b->phase0 + b->phase_acc);
            float v = clampf(b->offset + b->magnitude * env * w, -1.0f, 1.0f);
            *periodic_out += v * b->gain * dsign;
            break;
        }
        case ET_SPRING:
            *cond_out += condition_force(b, angle / half_range) * b->gain;
            break;
        case ET_DAMPER:
            *cond_out += condition_force(b, vel / DAMPER_VEL_FS) * b->gain;
            break;
        case ET_INERTIA:
            *cond_out += condition_force(b, accel / INERTIA_ACC_FS) * b->gain;
            break;
        case ET_FRICTION: {
            // Coulomb friction with a saturated-linear region near zero
            // velocity so it doesn't chatter at rest.
            float c = fmaxf(fabsf(b->pos_coeff), fabsf(b->neg_coeff));
            float s = clampf(vel / FRICTION_V_KNEE, -1.0f, 1.0f);
            *cond_out += -c * s * b->gain;
            break;
        }
    }
}

float effects_compute_torque(float angle, float vel, float dt) {
    if (!(dt > 0.0f) || dt > 0.1f) dt = 0.001f;

    // Acceleration estimate for inertia effects (lightly low-passed)
    float accel = (vel - last_vel) / dt;
    last_vel = vel;
    filtered_accel += 0.08f * (accel - filtered_accel);

    float half_range = 0.5f * g_cfg.wheel_range_deg * (float)M_PI / 180.0f;
    if (half_range < 0.1f) half_range = 0.1f;
    float max_t = g_cfg.max_torque_nm;

    bool gate_open = actuators_enabled && !device_paused && hid_usb_active();

    float host_n = 0.0f, periodic_n = 0.0f, cond_n = 0.0f;
    bool any_host_force = false;

    if (gate_open) {
        uint32_t now = eff_now_ms();
        lock_blocks();
        for (int i = 0; i < MAX_EFFECT_BLOCKS; ++i) {
            EffectBlock *b = &blocks[i];
            compute_block(b, angle, vel, filtered_accel, half_range, dt, now,
                          &host_n, &periodic_n, &cond_n);
            if (b->in_use && b->active &&
                (b->type == ET_CONSTANT || b->type == ET_RAMP || b->type == ET_PERIODIC)) {
                any_host_force = true;
            }
        }
        if (effect_playing > 0 && !ffb_has_active_effects()) {
            effect_playing = 0;
            hid_notify_pid_state_changed();
        }
        unlock_blocks();
    }

    // Host-silence watchdog: fade host-driven forces if the host went quiet.
    bool host_stale = any_host_force &&
                      (millis() - last_host_report_ms) > HOST_SILENCE_MS;
    if (host_stale) {
        host_fade -= dt / FADE_OUT_S;
    } else {
        host_fade += dt / FADE_IN_S;
    }
    host_fade = clampf(host_fade, 0.0f, 1.0f);

    // Reconstruction filter: hosts update constant/ramp magnitudes at their
    // own rate (60-400Hz); smoothing the staircase here is what removes the
    // "grainy" feel. Periodic and condition forces are synthesized locally at
    // full rate and must stay crisp, so they bypass the filter.
    float host_target = host_n * max_t;
    if (g_cfg.recon_cutoff_hz > 0.0f) {
        float rc = 1.0f / (2.0f * (float)M_PI * g_cfg.recon_cutoff_hz);
        float a = dt / (dt + rc);
        recon_state += a * (host_target - recon_state);
    } else {
        recon_state = host_target;
    }

    float game_t = (recon_state + periodic_n * max_t) * host_fade;
    if (g_cfg.invert_ffb) game_t = -game_t;
    game_t += cond_n * max_t;
    game_t *= device_gain;

    // Static-friction/cogging compensation (experimental, default off)
    if (g_cfg.breakaway && fabsf(game_t) > 0.02f) {
        game_t += (game_t > 0.0f ? 1.0f : -1.0f) * g_cfg.breakaway_nm;
    }

    // Device-side base feel under the game's FFB
    game_t += -g_cfg.base_damper * vel;
    game_t += -g_cfg.base_friction_nm * clampf(vel / FRICTION_V_KNEE, -1.0f, 1.0f);
    game_t += -g_cfg.base_inertia * filtered_accel;

    // Hands-off oscillation guard: sustained fast velocity reversals mean the
    // wheel is shaking itself; blend in extra damping until it settles.
    if (g_cfg.osc_guard) {
        static float ema_absvel = 0.0f;
        static uint32_t window_start = 0;
        static uint8_t flips = 0, flips_last = 0;
        static float prev_vel = 0.0f;
        ema_absvel += 0.01f * (fabsf(vel) - ema_absvel);
        if ((vel > 0.5f && prev_vel < -0.5f) || (vel < -0.5f && prev_vel > 0.5f)) flips++;
        if (fabsf(vel) > 0.5f) prev_vel = vel;
        uint32_t nowms = millis();
        if (nowms - window_start >= 250) {
            window_start = nowms;
            flips_last = flips;
            flips = 0;
        }
        bool oscillating = (flips_last >= 4) && (ema_absvel > 6.0f);
        float target = oscillating ? 1.0f : 0.0f;
        float rate = oscillating ? (dt / 0.05f) : (dt / 0.5f);
        osc_level += clampf(target - osc_level, -rate, rate);
        float osc_damper = 0.06f * max_t; // Nm per rad/s at full engagement
        game_t += -osc_level * osc_damper * vel;
    }

    // Clip game torque at the configured max — soft knee keeps clipped curbs
    // and crashes from feeling like hitting a hard wall.
    if (g_cfg.soft_clip) {
        game_t = max_t * tanhf(game_t / max_t);
    } else {
        game_t = clampf(game_t, -max_t, max_t);
    }

    // Slew limit: bounds torque rate to kill discontinuity "clunks" without
    // dulling normal detail.
    if (g_cfg.slew_nm_per_ms > 0.0f) {
        float max_step = g_cfg.slew_nm_per_ms * (dt * 1000.0f);
        game_t = slew_state + clampf(game_t - slew_state, -max_step, max_step);
    }
    slew_state = game_t;

    // End-stop overlay: progressive spring + damping inside the zone, with
    // its own torque budget so it can always overpower game forces.
    float total = game_t;
    float excess = 0.0f;
    if (angle > half_range) excess = angle - half_range;
    else if (angle < -half_range) excess = angle + half_range;
    if (excess != 0.0f) {
        float t_end = -g_cfg.endstop_k * excess - g_cfg.endstop_d * vel;
        t_end = clampf(t_end, -g_cfg.endstop_max_nm, g_cfg.endstop_max_nm);
        total += t_end;
    }
    total = clampf(total, -(max_t + g_cfg.endstop_max_nm), max_t + g_cfg.endstop_max_nm);

    if (!gate_open) {
        recon_state = 0.0f;
        slew_state = 0.0f;
        total = 0.0f;
    }

    g_ffb_torque_nm = total;
    return total;
}

// ----------------------
// HID report parsing
// ----------------------

static float int16_to_unit(int16_t v) {
    return (float)v / 32767.0f;
}

static int16_t read_int16(uint8_t *buf, int offset) {
    return (int16_t)((uint16_t)buf[offset] | ((uint16_t)buf[offset + 1] << 8));
}

static int16_t read_int16_safe(uint8_t *buf, uint16_t len, int offset, int16_t fallback = 0) {
    if (!buf || offset < 0 || (offset + 1) >= (int)len) return fallback;
    return read_int16(buf, offset);
}

static uint8_t read_uint8_safe(uint8_t *buf, uint16_t len, int offset, uint8_t fallback = 0) {
    if (!buf || offset < 0 || offset >= (int)len) return fallback;
    return buf[offset];
}

static void set_block_type_from_hid(EffectBlock *b, uint8_t effect_type) {
    switch (effect_type) {
        case 1:  b->type = ET_CONSTANT; break;
        case 2:  b->type = ET_RAMP; break;
        case 3:  b->type = ET_PERIODIC; b->wave = WAVE_SQUARE; break;
        case 4:  b->type = ET_PERIODIC; b->wave = WAVE_SINE; break;
        case 5:  b->type = ET_PERIODIC; b->wave = WAVE_TRIANGLE; break;
        case 6:  b->type = ET_PERIODIC; b->wave = WAVE_SAW_UP; break;
        case 7:  b->type = ET_PERIODIC; b->wave = WAVE_SAW_DOWN; break;
        case 8:  b->type = ET_SPRING; break;
        case 9:  b->type = ET_DAMPER; break;
        case 10: b->type = ET_INERTIA; break;
        case 11: b->type = ET_FRICTION; break;
        default: b->type = ET_NONE; break;
    }
}

// Parse incoming FFB report according to the USB HID PID descriptor in hid.cpp
void handle_ffb_report(uint8_t report_id, uint8_t *buf, uint16_t len) {
    last_host_report_ms = millis();

    lock_blocks();

    switch (report_id) {
        case 0x03: { // Set Effect
            // [idx, duration(2), gain(2), trigger_btn, trigger_repeat(2),
            //  enables, direction1(2), direction2(2), start_delay(2), type]
            if (len < 1) break;

            uint8_t effect_idx = buf[0];
            if (effect_idx < 1 || effect_idx > MAX_EFFECT_BLOCKS) break;
            effect_idx--;

            EffectBlock *b = get_block(effect_idx);
            if (!b) break;
            bool was_in_use = b->in_use;

            uint16_t duration = (uint16_t)read_int16_safe(buf, len, 1, 0);
            uint16_t gain16 = (uint16_t)read_int16_safe(buf, len, 3, (int16_t)0xFFFF);
            uint8_t enables = read_uint8_safe(buf, len, 8, 0x02);
            uint16_t direction = (uint16_t)read_int16_safe(buf, len, 9, 0);
            uint16_t start_delay = (uint16_t)read_int16_safe(buf, len, 13, 0);
            uint8_t effect_type = read_uint8_safe(buf, len, 15, 0);

            b->in_use = 1;
            if (effect_type >= 1 && effect_type <= 11) {
                set_block_type_from_hid(b, effect_type);
            }

            // Duration is honest milliseconds now that the descriptor declares
            // time units. HID "infinite" sentinels map to 0 (persistent).
            if (duration == 0 || duration == 0xFFFF || duration == 0x7FFF) {
                b->duration_ms = 0;
            } else {
                b->duration_ms = duration;
            }
            b->gain = gain16 / 65535.0f;
            b->direction_enable = (enables & 0x02) != 0;
            // Direction logical units: hundredths of degrees, 0..35999
            if (direction > 35999) direction = 35999;
            b->direction_rad = (float)direction * (2.0f * (float)M_PI / 36000.0f);
            b->start_delay_ms = start_delay;

            // A Set Effect on a playing block is a live parameter update
            // (Linux FF_UPDATE / DirectInput SetParameters) — do not stop it.
            if (!was_in_use) {
                b->active = false;
                b->loop_count = 1;
                update_ram_pool_available();
            }

            if (ffb_verbose) {
                Serial.printf("FFB: Set Effect idx=%d type=%d dur=%u dir=%.2f gain=%.2f\n",
                              effect_idx, b->type, (unsigned)b->duration_ms,
                              b->direction_rad, b->gain);
            }
        } break;

        case 0x04: { // Set Envelope
            // [idx, attack_level(2), attack_time(2), fade_level(2), fade_time(2)]
            if (len < 3) break;

            uint8_t effect_idx = buf[0];
            if (effect_idx < 1 || effect_idx > MAX_EFFECT_BLOCKS) break;
            effect_idx--;

            EffectBlock *b = get_block(effect_idx);
            if (!b || !b->in_use) break;

            uint16_t attack_level = (uint16_t)read_int16_safe(buf, len, 1, 0);
            uint16_t attack_time = (uint16_t)read_int16_safe(buf, len, 3, 0);
            uint16_t fade_level = (uint16_t)read_int16_safe(buf, len, 5, 0);
            uint16_t fade_time = (uint16_t)read_int16_safe(buf, len, 7, 0);

            b->attack_level = clampf(attack_level / 32767.0f, 0.0f, 1.0f);
            b->attack_time_ms = attack_time;
            b->fade_level = clampf(fade_level / 32767.0f, 0.0f, 1.0f);
            b->fade_time_ms = fade_time;

            if (ffb_verbose) {
                Serial.printf("FFB: Set Envelope idx=%d al=%.2f at=%u fl=%.2f ft=%u\n",
                              effect_idx, b->attack_level, attack_time,
                              b->fade_level, fade_time);
            }
        } break;

        case 0x05: { // Set Condition
            // [idx, param_block_offset, cp_offset(2), pos_coeff(2), neg_coeff(2),
            //  pos_sat(2), neg_sat(2), deadband(2)]
            if (len < 4) break;

            uint8_t effect_idx = buf[0];
            if (effect_idx < 1 || effect_idx > MAX_EFFECT_BLOCKS) break;
            effect_idx--;

            EffectBlock *b = get_block(effect_idx);
            if (!b || !b->in_use) break;

            // Single FFB axis: only parameter block 0 applies. Hosts send a
            // second block for the (nonexistent) Y axis — must not clobber X.
            uint8_t block_offset = read_uint8_safe(buf, len, 1, 0);
            if (block_offset != 0) break;

            int16_t cp_offset = read_int16_safe(buf, len, 2, 0);
            int16_t pos_coeff = read_int16_safe(buf, len, 4, 0);
            int16_t neg_coeff = read_int16_safe(buf, len, 6, pos_coeff);
            int16_t pos_sat = read_int16_safe(buf, len, 8, 32767);
            int16_t neg_sat = read_int16_safe(buf, len, 10, 32767);
            int16_t deadband = read_int16_safe(buf, len, 12, 0);

            b->cp_offset = int16_to_unit(cp_offset);
            b->pos_coeff = int16_to_unit(pos_coeff);
            b->neg_coeff = int16_to_unit(neg_coeff);
            b->pos_sat = fabsf(int16_to_unit(pos_sat));
            b->neg_sat = fabsf(int16_to_unit(neg_sat));
            b->deadband = fabsf(int16_to_unit(deadband));

            if (ffb_verbose) {
                Serial.printf("FFB: Set Condition idx=%d cp=%.2f pc=%.2f nc=%.2f ps=%.2f ns=%.2f db=%.2f\n",
                              effect_idx, b->cp_offset, b->pos_coeff, b->neg_coeff,
                              b->pos_sat, b->neg_sat, b->deadband);
            }
        } break;

        case 0x06: { // Set Periodic
            // [idx, magnitude(2), offset(2), phase(2), period(2)]
            if (len < 3) break;

            uint8_t effect_idx = buf[0];
            if (effect_idx < 1 || effect_idx > MAX_EFFECT_BLOCKS) break;
            effect_idx--;

            EffectBlock *b = get_block(effect_idx);
            if (!b || !b->in_use) break;

            int16_t magnitude = read_int16_safe(buf, len, 1, 0);
            int16_t offset = read_int16_safe(buf, len, 3, 0);
            uint16_t phase = (uint16_t)read_int16_safe(buf, len, 5, 0);
            uint16_t period = (uint16_t)read_int16_safe(buf, len, 7, 0);

            b->magnitude = int16_to_unit(magnitude);
            b->offset = int16_to_unit(offset);
            // Phase logical units: hundredths of degrees
            if (phase > 35999) phase = 35999;
            b->phase0 = (float)phase * (2.0f * (float)M_PI / 36000.0f);
            if (period > 0) {
                b->freq_hz = clampf(1000.0f / (float)period, 0.1f, 500.0f);
            } else if (b->freq_hz <= 0.0f) {
                b->freq_hz = 80.0f; // rumble-like fallback if host never set one
            }

            if (ffb_verbose) {
                Serial.printf("FFB: Set Periodic idx=%d mag=%.3f off=%.3f freq=%.2f\n",
                              effect_idx, b->magnitude, b->offset, b->freq_hz);
            }
        } break;

        case 0x07: { // Set Constant Force
            // [idx, magnitude(2)]
            if (len < 3) break;

            uint8_t effect_idx = buf[0];
            if (effect_idx < 1 || effect_idx > MAX_EFFECT_BLOCKS) break;
            effect_idx--;

            EffectBlock *b = get_block(effect_idx);
            if (!b || !b->in_use) break;

            b->magnitude = int16_to_unit(read_int16(buf, 1));

            if (ffb_verbose) {
                Serial.printf("FFB: Set Constant idx=%d mag=%.3f\n", effect_idx, b->magnitude);
            }
        } break;

        case 0x08: { // Set Ramp Force
            // [idx, ramp_start(2), ramp_end(2)]
            if (len < 5) break;

            uint8_t effect_idx = buf[0];
            if (effect_idx < 1 || effect_idx > MAX_EFFECT_BLOCKS) break;
            effect_idx--;

            EffectBlock *b = get_block(effect_idx);
            if (!b || !b->in_use) break;

            b->magnitude = int16_to_unit(read_int16(buf, 1));
            b->magnitude_end = int16_to_unit(read_int16(buf, 3));

            if (ffb_verbose) {
                Serial.printf("FFB: Set Ramp idx=%d start=%.3f end=%.3f\n",
                              effect_idx, b->magnitude, b->magnitude_end);
            }
        } break;

        case 0x09: { // Effect Operation
            // [idx, operation (1=Start, 2=Start Solo, 3=Stop), loop_count]
            if (len < 2) break;

            uint8_t effect_idx = buf[0];
            if (effect_idx < 1 || effect_idx > MAX_EFFECT_BLOCKS) break;
            effect_idx--;

            uint8_t operation = buf[1];
            uint8_t loop_count = read_uint8_safe(buf, len, 2, 0);

            EffectBlock *b = get_block(effect_idx);
            if (!b || !b->in_use) break;

            if (operation == 1 || operation == 2) {
                if (operation == 2) { // Start Solo: stop everything else first
                    for (int i = 0; i < MAX_EFFECT_BLOCKS; i++) {
                        if (i != effect_idx) blocks[i].active = false;
                    }
                }
                b->active = true;
                b->start_time_ms = eff_now_ms();
                b->phase_acc = 0.0f;
                b->loop_count = (loop_count == 0) ? 1 : loop_count;
                effect_playing = effect_idx + 1;
                hid_notify_pid_state_changed();
                if (ffb_verbose) {
                    Serial.printf("FFB: Start%s idx=%d loops=%u\n",
                                  operation == 2 ? " Solo" : "", effect_idx, loop_count);
                }
            } else if (operation == 3) {
                b->active = false;
                refresh_effect_playing();
                hid_notify_pid_state_changed();
                if (ffb_verbose) {
                    Serial.printf("FFB: Stop idx=%d\n", effect_idx);
                }
            }
        } break;

        case 0x0A: { // Block Free
            if (len < 1) break;

            uint8_t effect_idx = buf[0];
            if (effect_idx < 1 || effect_idx > MAX_EFFECT_BLOCKS) break;
            effect_idx--;

            EffectBlock *b = get_block(effect_idx);
            if (b) {
                clear_block(b);
                update_ram_pool_available();
                refresh_effect_playing();
                hid_notify_pid_state_changed();
                if (ffb_verbose) {
                    Serial.printf("FFB: Block Free idx=%d\n", effect_idx);
                }
            }
        } break;

        case 0x0B: { // Device Control
            // [control] 1=Enable, 2=Disable, 3=Stop All, 4=Reset, 5=Pause, 6=Continue
            if (len < 1) break;

            switch (buf[0]) {
                case 1:
                    actuators_enabled = true;
                    hid_notify_pid_state_changed();
                    if (ffb_verbose) Serial.println("FFB: Actuators ENABLED");
                    break;
                case 2:
                    actuators_enabled = false;
                    hid_notify_pid_state_changed();
                    if (ffb_verbose) Serial.println("FFB: Actuators DISABLED");
                    break;
                case 3:
                    for (int i = 0; i < MAX_EFFECT_BLOCKS; i++) {
                        blocks[i].active = false;
                    }
                    effect_playing = 0;
                    hid_notify_pid_state_changed();
                    if (ffb_verbose) Serial.println("FFB: Stop All Effects");
                    break;
                case 4:
                    for (int i = 0; i < MAX_EFFECT_BLOCKS; i++) {
                        clear_block(&blocks[i]);
                    }
                    device_gain = 1.0f;
                    actuators_enabled = true;
                    effect_playing = 0;
                    if (device_paused) {
                        paused_total_ms += millis() - pause_started_ms;
                        device_paused = false;
                    }
                    update_ram_pool_available();
                    hid_notify_pid_state_changed();
                    if (ffb_verbose) Serial.println("FFB: Device Reset");
                    break;
                case 5:
                    if (!device_paused) {
                        device_paused = true;
                        pause_started_ms = millis();
                        hid_notify_pid_state_changed();
                    }
                    break;
                case 6:
                    if (device_paused) {
                        paused_total_ms += millis() - pause_started_ms;
                        device_paused = false;
                        hid_notify_pid_state_changed();
                    }
                    break;
            }
        } break;

        case 0x0C: { // Device Gain
            if (len < 2) break;
            device_gain = (uint16_t)read_int16(buf, 0) / 65535.0f;
            if (ffb_verbose) {
                Serial.printf("FFB: Device Gain %.2f\n", device_gain);
            }
        } break;

        case 0x0D: { // Create New Effect (Feature SET)
            // [effect_block_index (unused), effect_type]
            uint8_t effect_type = 0;
            if (len >= 2) effect_type = buf[1];
            else if (len == 1) effect_type = buf[0];

            if (effect_type < 1 || effect_type > 11) {
                pid_block_load_status = 3; // error
                pid_block_load_effect_idx = 1;
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
            clear_block(b);
            b->in_use = 1;
            b->active = false;
            b->gain = 1.0f;
            b->loop_count = 1;
            b->duration_ms = 0;
            set_block_type_from_hid(b, effect_type);

            pid_block_load_effect_idx = (uint8_t)(free_idx + 1);
            pid_block_load_status = (b->type == ET_NONE) ? 3 : 1;
            update_ram_pool_available();

            if (ffb_verbose) {
                Serial.printf("FFB: Create New Effect type=%u -> idx=%u\n",
                              effect_type, pid_block_load_effect_idx);
            }
        } break;

        default:
            if (ffb_verbose) {
                Serial.printf("FFB: Unknown report ID 0x%02X len=%d\n", report_id, len);
            }
            break;
    }

    unlock_blocks();
}

uint16_t ffb_get_feature_report(uint8_t report_id, uint8_t *buffer, uint16_t len) {
    if (!buffer || len == 0) return 0;

    // Report ID 2: PID State (descriptor exposes this as Input; keep GET fallback)
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

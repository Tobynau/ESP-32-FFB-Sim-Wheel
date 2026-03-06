#include "encoder.h"
#include <Arduino.h>
#include <Wire.h>

static constexpr int MT6701_COUNTS_PER_REV = 16384;
static constexpr float MT6701_COUNT_TO_TURNS = 1.0f / (float)MT6701_COUNTS_PER_REV;

static int i2c_sda = -1;
static int i2c_scl = -1;
static uint8_t i2c_addr = 0x06;
static unsigned long sample_interval_us = 1000; // default 1kHz
static unsigned long last_sample_us = 0;

static int last_encoder_count = 0;
static uint32_t consecutive_read_failures = 0;
static float angle = 0.0f; // wheel angle in radians (wrapped absolute 0..2PI)
static float angle_continuous = 0.0f; // wheel angle in radians (continuous, unbounded)
static float center_angle_continuous = 0.0f; // center position at power-on (continuous radians)
static float vel = 0.0f;   // wheel angular velocity rad/s
static bool initialized = false;
// gearing: encoder -> motor -> wheel
static float encoder_to_motor_ratio = 1.0f;
static float motor_to_wheel_ratio = 1.0f;
static float overall_ratio = 1.0f;

static bool read_encoder_count(int *out_count) {
    if (!out_count) return false;

    Wire.beginTransmission(i2c_addr);
    Wire.write(0x03);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    int requested = Wire.requestFrom((int)i2c_addr, 2);
    if (requested < 2 || Wire.available() < 2) {
        return false;
    }

    int angle_h = Wire.read();
    int angle_l = Wire.read();
    int count = ((angle_h << 6) | angle_l) & 0x3FFF;
    *out_count = count;
    return true;
}

// Initialize MT6701 on given SDA/SCL pins and sampling rate
void encoder_init(int sda_pin, int scl_pin, unsigned long sample_hz, uint8_t i2c_addr, float enc_to_motor_ratio, float motor_to_wheel, int mt_update_ms) {
    i2c_sda = sda_pin;
    i2c_scl = scl_pin;
    ::i2c_addr = i2c_addr;
    encoder_to_motor_ratio = enc_to_motor_ratio;
    motor_to_wheel_ratio = motor_to_wheel;
    if (sample_hz == 0) sample_hz = 1000;
    sample_interval_us = 1000000UL / sample_hz;

    (void)mt_update_ms;

    Wire.begin(i2c_sda, i2c_scl);
    Wire.setClock(400000);
    Wire.setTimeOut(2);

    delay(5);

    int count = 0;
    if (read_encoder_count(&count)) {
        last_encoder_count = count;
        overall_ratio = encoder_to_motor_ratio * motor_to_wheel_ratio; // encoder turns per wheel turn
        if (!isfinite(overall_ratio) || fabsf(overall_ratio) < 1e-6f) {
            overall_ratio = 1.0f;
        }

        float encoder_turns = count * MT6701_COUNT_TO_TURNS;
        float wheel_turns = encoder_turns / overall_ratio;
        angle_continuous = wheel_turns * 2.0f * PI;
        angle = fmodf(angle_continuous, 2.0f * PI);
        if (angle < 0) angle += 2.0f * PI;
        center_angle_continuous = angle_continuous;
        consecutive_read_failures = 0;
    } else {
        last_encoder_count = 0;
        angle = 0.0f;
        angle_continuous = 0.0f;
        center_angle_continuous = 0.0f;
        consecutive_read_failures = 1;
    }

    vel = 0.0f;
    last_sample_us = micros();
    initialized = true;
}

bool encoder_update() {
    if (!initialized) return false;

    unsigned long now = micros();
    unsigned long dt_us = now - last_sample_us;
    if (dt_us < sample_interval_us) return false;

    int new_count = 0;
    if (!read_encoder_count(&new_count)) {
        consecutive_read_failures++;
        if (consecutive_read_failures >= 5) {
            Wire.begin(i2c_sda, i2c_scl);
            Wire.setClock(400000);
            Wire.setTimeOut(2);
            consecutive_read_failures = 0;
        }
        return false;
    }
    consecutive_read_failures = 0;

    // Only advance sampling timestamp after a successful read.
    last_sample_us = now;

    int diff = new_count - last_encoder_count;
    if (diff > (MT6701_COUNTS_PER_REV / 2)) {
        diff -= MT6701_COUNTS_PER_REV;
    } else if (diff < -(MT6701_COUNTS_PER_REV / 2)) {
        diff += MT6701_COUNTS_PER_REV;
    }

    float delta_turns = diff * MT6701_COUNT_TO_TURNS;

    float dt = (float)dt_us / 1000000.0f;
    if (dt <= 0.0f) return false;

    float wheel_delta_turns = delta_turns / overall_ratio;
    float new_vel = (wheel_delta_turns * 2.0f * PI) / dt;

    const float alpha = 0.1f;
    vel = (alpha * new_vel) + (1.0f - alpha) * vel;

    angle_continuous += wheel_delta_turns * 2.0f * PI;
    angle = fmodf(angle_continuous, 2.0f * PI);
    if (angle < 0) angle += 2.0f * PI;

    last_encoder_count = new_count;
    return true;
}

float encoder_read_angle_rad() {
    return angle;
}

// Get angle relative to center (power-on position)
// Returns continuous angle in radians where 0 is center.
// Positive/negative values grow with rotation and are scaled by configured ratios.
float encoder_read_centered_angle_rad() {
    if (!initialized) return 0.0f; // Safety: return center if not initialized
    
    float centered = angle_continuous - center_angle_continuous;
    
    // Safety check
    if (!isfinite(centered)) return 0.0f;
    
    return centered;
}

float encoder_read_vel_rads() {
    return vel;
}

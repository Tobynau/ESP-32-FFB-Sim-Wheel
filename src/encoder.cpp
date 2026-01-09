#include "encoder.h"
#include <Arduino.h>
#include <Wire.h>
#include <MT6701.hpp>
#include <main.cpp>

static int i2c_sda = -1;
static int i2c_scl = -1;
static unsigned long sample_interval_us = 1000; // default 1kHz
static unsigned long last_sample_us = 0;
// Continuous turns from encoder
static float last_encoder_turns = 0.0f;
static float angle = 0.0f; // wheel angle in radians (absolute)
static float center_angle = 0.0f; // center position at power-on (radians)
static float vel = 0.0f;   // wheel angular velocity rad/s
static bool initialized = false;
static MT6701 *encoderPtr = nullptr; // pointer to MT6701 instance
// gearing: encoder -> motor -> wheel


// Internal: read absolute angle from MT6701 in radians [0..2PI)
// read continuous turns from the encoder (can be many revolutions)
static bool read_encoder_turns(float *out_turns) {
    if (!out_turns) return false;
    if (!encoderPtr) return false;
    
    // The library's background task should be updating automatically
    // Just read the value
    float turns = encoderPtr->getTurns();
    if (!isfinite(turns)) return false;
    *out_turns = turns;
    return true;
}

// Initialize MT6701 on given SDA/SCL pins and sampling rate
void encoder_init(int sda_pin, int scl_pin, unsigned long sample_hz, uint8_t i2c_addr, float enc_to_motor_ratio, float motor_to_wheel, int mt_update_ms) {
    i2c_sda = sda_pin;
    i2c_scl = scl_pin;
    encoder_to_motor_ratio = enc_to_motor_ratio;
    motor_to_wheel_ratio = motor_to_wheel;
    if (sample_hz == 0) sample_hz = 1000;
    sample_interval_us = 1000000UL / sample_hz;
    
    // initialize encoder library with provided address and pins
    if (encoderPtr) {
        delete encoderPtr;
        encoderPtr = nullptr;
    }
    
    // MT6701 constructor now accepts pins: (address, update_interval_ms, rpm_threshold, rpm_filter_size, sda, scl)
    if (mt_update_ms > 0) {
        encoderPtr = new MT6701(i2c_addr, mt_update_ms, MT6701::RPM_THRESHOLD, MT6701::RPM_FILTER_SIZE, i2c_sda, i2c_scl);
    } else {
        encoderPtr = new MT6701(i2c_addr, MT6701::UPDATE_INTERVAL, MT6701::RPM_THRESHOLD, MT6701::RPM_FILTER_SIZE, i2c_sda, i2c_scl);
    }
    
    // Now begin() will use our pins
    encoderPtr->begin();
    // small delay to let bus settle
    delay(5);
    float turns = 0.0f;
    if (read_encoder_turns(&turns)) {
        last_encoder_turns = turns;
        // compute wheel turns from encoder turns via ratios:
        // wheel_turns = encoder_turns / (encoder_to_motor_ratio * motor_to_wheel_ratio)
        overall_ratio = encoder_to_motor_ratio * motor_to_wheel_ratio; // encoder turns per wheel turn
        float wheel_turns = turns / overall_ratio;
        angle = fmodf(wheel_turns, 1.0f) * 2.0f * PI;
        if (angle < 0) angle += 2.0f * PI;
        // Store this as center position (assume wheel is centered at power-on)
        center_angle = angle;
    } else {
        angle = 0.0f;
        center_angle = 0.0f;
        last_encoder_turns = 0.0f;
    }
    vel = 0.0f;
    last_sample_us = micros();
    initialized = true;
}

bool encoder_update() {
    if (!initialized) return false;
    unsigned long now = micros();
    if ((now - last_sample_us) < sample_interval_us) return false;
    unsigned long dt_us = now - last_sample_us;
    last_sample_us = now;

    float new_turns = 0.0f;
    if (!read_encoder_turns(&new_turns)) {
        // read failed; keep previous values
        return false;
    }

    // encoder delta in turns (can be >1 if spinning fast between samples)
    float delta_turns = new_turns - last_encoder_turns;

    float dt = (float)dt_us / 1000000.0f;
    if (dt <= 0.0f) return false;
    // compute wheel turns delta from encoder delta using overall_ratio
    float wheel_delta_turns = delta_turns / overall_ratio;
    // wheel angular velocity (rad/s)
    float new_vel = (wheel_delta_turns * 2.0f * PI) / dt;
    // simple low-pass filter on velocity (match previous alpha)
    const float alpha = 0.1f;
    vel = (alpha * new_vel) + (1.0f - alpha) * vel;

    // Update integrated wheel angle (tracks continuous rotations) from wheel_delta_turns
    angle += wheel_delta_turns * 2.0f * PI;
    // normalize angle to 0..2PI
    while (angle < 0) angle += 2.0f * PI;
    while (angle >= 2.0f * PI) angle -= 2.0f * PI;

    last_encoder_turns = new_turns;
    return true;
}

float encoder_read_angle_rad() {
    return angle;
}

// Get angle relative to center (power-on position)
// Returns angle in range -PI..PI where 0 is center
float encoder_read_centered_angle_rad() {
    if (!initialized) return 0.0f; // Safety: return center if not initialized
    
    float centered = angle - center_angle;
    
    // Safety check
    if (!isfinite(centered)) return 0.0f;
    
    // Normalize to -PI..PI
    while (centered > PI) centered -= 2.0f * PI;
    while (centered < -PI) centered += 2.0f * PI;
    
    return centered;
}

float encoder_read_vel_rads() {
    return vel;
}

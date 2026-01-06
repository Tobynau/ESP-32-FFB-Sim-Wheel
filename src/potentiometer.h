#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

#include <Arduino.h>
// MT6701 magnetic encoder (I2C) replacement for analog potentiometer
// Preserves the original API: pot_init(pin, sample_hz), pot_update(), pot_read_angle_rad(), pot_read_vel_rads()
// New pot_init signature: pot_init(sda_pin, scl_pin, sample_hz)


// Initialize encoder pins (SDA,SCL), sampling rate, I2C address and gearing ratios
// i2c_addr: MT6701 I2C address (default 0x06)
// encoder_to_motor_ratio: how many encoder turns per motor turn
// motor_to_wheel_ratio: how many motor turns per wheel turn (e.g. GEAR_RATIO)
// mt_update_ms: optional MT6701 internal update interval in milliseconds (default library value)
void pot_init(int sda_pin, int scl_pin, unsigned long sample_hz, uint8_t i2c_addr = 0x06, float encoder_to_motor_ratio = 1.0f, float motor_to_wheel_ratio = 1.0f, int mt_update_ms = -1);

// Read current angle in radians (0..2*PI)
float pot_read_angle_rad();

// Read current angular velocity in rad/s (simple derivative, filtered)
float pot_read_vel_rads();

// Update sampling (call from loop or timer). Returns true when new sample taken.
bool pot_update();

#endif // POTENTIOMETER_H

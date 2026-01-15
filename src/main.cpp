// main.cpp
// ESP32-S3 (Arduino/PlatformIO) FFB wheel 


#include <Arduino.h>
#include <driver/gpio.h>
#include "encoder.h"
#include "vesc.h"
#include "hid.h"
#include "effects.h"
#include "VescUart.h"

//
// ========== CONFIG ==========
//

#define WHEEL_CPR    4096.0f
// Gearing ratios:
// - Encoder to motor: encoder spins faster than motor (e.g., 5:1)
// - Motor to wheel: motor spins faster than wheel (e.g., 5:1)
// - Overall encoder to wheel: 5 * 5 = 25:1

static float encoder_to_motor_ratio = 5.0f;
static float motor_to_wheel_ratio = 5.0f;
static float overall_ratio = 25.0f; // encoder turns per wheel turn
// Conversion: Nm -> Amps (typical VESC with 8-10 pole motor: ~0.5-1.5 A/Nm)
// Adjust this based on your motor's torque constant
#define TORQUE_TO_CURRENT  1.5f
#define CTRL_HZ     1000UL
#define ANGLE_REPORT_HZ 200U


float wheel_angle_rad = 0.0f;
float wheel_vel_rads = 0.0f;
static inline uint32_t usec() { return micros(); }

// Button pin for reading a user button (active LOW). Set to 0 to disable.
const int Left_padle = 0; 
const int Right_padle = 1; 

// Expose configuration to effects and hid modules
// (declared in effects.h and hid.h)
// set sensible defaults; can be changed at runtime if desired


// VESC helpers are now in vesc.cpp/.h

// VESC commands are now in vesc.cpp/.h

// USB HID and joystick logic is now in hid.cpp/.h

// Effect mixing is now in effects.cpp/.h

//
// ========== CONTROL LOOP ==========

void setup() {
  hid_init();
  
  // Encoder / gearing configuration
  const uint8_t ENCODER_I2C_ADDR = 0x06; // MT6701 default
  const int MT_UPDATE_MS = 2; // MT6701 internal update interval (ms) - faster polling for high speed
  // Recommended ESP32-S3 default I2C pins (check your board docs) - using SDA=8, SCL=9
  const int I2C_SDA_PIN = 8;
  const int I2C_SCL_PIN = 9;
  
  // Initialize encoder on I2C pins at CTRL_HZ sampling
  encoder_init(I2C_SDA_PIN, I2C_SCL_PIN, CTRL_HZ, ENCODER_I2C_ADDR, encoder_to_motor_ratio, motor_to_wheel_ratio, MT_UPDATE_MS);
  
  delay(100);
  // Initialize hardware UART for VESC on GPIO 15 (RX) and 16 (TX) at 921600 (adjust VESC to match)
  // Higher baud rate needed to sustain 1kHz loop with polling
  Serial1.begin(921600, SERIAL_8N1, 15, 16);
  // Attach VescUart to Serial1
  UART.setSerialPort(&Serial1);

  device_gain = 1.0f;
  last_effect_time = millis();

  // configure button pin and FFB gearing
  //button_pin = USER_BUTTON_PIN;
  max_wheel_angle_deg = 270.0f; // 270 degrees max rotation (adjustable)
  angle_limit_stiffness = 300.0f; // Nm/rad spring force at limits (adjustable)
  
  // Set gear ratio for FFB calculations (Motor -> Wheel)
  gear_ratio = motor_to_wheel_ratio;
}



void loop() {
  static uint32_t last_ctrl = 0;
  static uint32_t last_usb = 0;
  static uint32_t last_telemetry = 0;
  static bool first_usb_send = true;
  uint32_t now_us = usec();
  uint32_t now_ms = millis();
  
  // poll telemetry (non-blocking if possible, throttle to 50Hz to save UART bandwidth/time)
  if ((now_ms - last_telemetry) >= 20) {
    last_telemetry = now_ms;
    vesc_request_status5();
  }

  if ((now_us - last_ctrl) >= (1000000UL / CTRL_HZ)) {
    last_ctrl += (1000000UL / CTRL_HZ);
  // update potentiometer sampling
  encoder_update();
  // Use centered angle (relative to power-on position) for all FFB and game output
  float centered_angle = encoder_read_centered_angle_rad();
  float vel_rads = encoder_read_vel_rads();
  wheel_angle_rad = centered_angle;
  wheel_vel_rads = vel_rads;
  
  float tau = mix_effects(wheel_angle_rad, wheel_vel_rads);
  // Convert torque at wheel to motor torque using gear ratio
  float motor_tau = tau / gear_ratio;
  // Map motor torque to current (simple linear scale, user-calibrate TORQUE_TO_CURRENT)
  float amps = motor_tau * TORQUE_TO_CURRENT;
  vesc_set_current(amps);
  }
  if ((millis() - last_usb) >= (1000U / ANGLE_REPORT_HZ)) {
    last_usb = millis();
    // Wait a bit after boot before sending USB data to ensure encoder is ready
    if (first_usb_send && millis() < 500) {
      return; // Skip first 500ms of USB sends
    }
    first_usb_send = false;
    // Now use real encoder value
    usb_send_joystick(wheel_angle_rad);
  }
}

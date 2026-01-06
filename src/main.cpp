// main.cpp
// ESP32-S3 (Arduino/PlatformIO) FFB wheel 
// - USB HID PID descriptor (basic: constant, spring, damper)
// - Parse some PID OUT/Feature reports


#include <Arduino.h>
#include <driver/gpio.h>
#include "potentiometer.h"
#include "vesc.h"
#include "hid.h"
#include "effects.h"
#include "VescUart.h"

//
// ========== CONFIG ==========
//

#define WHEEL_CPR    4096.0f
// User-configurable gearing: motor turns per wheel turn
#define GEAR_RATIO   5.0f
// Conversion placeholder: Nm -> Amps, user should calibrate
#define TORQUE_TO_CURRENT  1.0f
#define CTRL_HZ     1000UL
#define ANGLE_REPORT_HZ 200U


float wheel_angle_rad = 0.0f;
float wheel_vel_rads = 0.0f;
static inline uint32_t usec() { return micros(); }

// Button pin for reading a user button (active LOW). Set to 0 to disable.
const int USER_BUTTON_PIN = 0; // set to actual GPIO if available, e.g., 0

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
  Serial.begin(115200);
  delay(2000); // Give time for serial to initialize
  Serial.println("\n\n=== ESP32 FFB Wheel Starting ===");
  
  Serial.println("Initializing HID...");
  hid_init();
  Serial.println("HID Initialized");
  
  // Encoder / gearing configuration
  const uint8_t ENCODER_I2C_ADDR = 0x06; // MT6701 default
  const float ENCODER_TO_MOTOR_RATIO = 5.0f; // encoder reads 5x motor
  const float MOTOR_TO_WHEEL_RATIO = 5.0f; // motor reads 5x wheel -> overall 25x
  const int MT_UPDATE_MS = 50; // MT6701 internal update interval (ms)
  // Recommended ESP32-S3 default I2C pins (check your board docs) - using SDA=8, SCL=9
  const int I2C_SDA_PIN = 8;
  const int I2C_SCL_PIN = 9;
  
  Serial.println("Initializing Encoder...");
  Serial.printf("  I2C: SDA=%d, SCL=%d, Addr=0x%02X\n", I2C_SDA_PIN, I2C_SCL_PIN, ENCODER_I2C_ADDR);
  // Initialize encoder on I2C pins at CTRL_HZ sampling
  pot_init(I2C_SDA_PIN, I2C_SCL_PIN, CTRL_HZ, ENCODER_I2C_ADDR, ENCODER_TO_MOTOR_RATIO, MOTOR_TO_WHEEL_RATIO, MT_UPDATE_MS);
  Serial.println("Encoder Initialized");
  
  delay(100);
  // Initialize hardware UART for VESC on GPIO 15 (RX) and 16 (TX) at 115200
  // Assumption: VESC RX is connected to GPIO16 and VESC TX to GPIO15 (common wiring: RX<-TX, TX<-RX)
  // If your wiring is reversed, swap the RX/TX pin order below.
  Serial.println("Initializing VESC UART...");
  Serial1.begin(115200, SERIAL_8N1, 15, 16);
  // Attach VescUart to Serial1
  UART.setSerialPort(&Serial1);

  Serial.println("VESC UART (Serial1) Initialized");

  device_gain = 1.0f;
  last_effect_time = millis();

  // configure button pin and effect module gearing
  button_pin = USER_BUTTON_PIN;
  gear_ratio = (float)GEAR_RATIO;
  max_wheel_angle_rad = 2.0f * PI; // default: full rotation

  Serial.println("=== Setup Complete ===\n");
  hid_task();
}



void loop() {
  static uint32_t last_ctrl = 0;
  static uint32_t last_usb = 0;
  static uint32_t last_debug = 0;
  uint32_t now_us = usec();
  
  // Debug heartbeat every 5 seconds
  if (millis() - last_debug > 5000) {
    last_debug = millis();
    Serial.println("Loop running...");
  }
  
  // poll telemetry (non-blocking in this simple approach)
  // vesc_request_status5 will call UART.getVescValues()
  vesc_request_status5();
  if ((now_us - last_ctrl) >= (1000000UL / CTRL_HZ)) {
    last_ctrl += (1000000UL / CTRL_HZ);
  // update potentiometer sampling
  pot_update();
  float angle_rad = pot_read_angle_rad();
  float vel_rads = pot_read_vel_rads();
  wheel_angle_rad = angle_rad;
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
  usb_send_joystick(wheel_angle_rad);
  }
}

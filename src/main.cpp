// main.cpp
// ESP32-S3 (Arduino/PlatformIO) FFB wheel 


#include <Arduino.h>
#include <driver/gpio.h>
#include "encoder.h"
#include "vesc.h"
#include "hid.h"
#include "effects.h"
#include "pedals.h"
#include "VescUart.h"

//
// ========== CONFIG ==========
//

#define WHEEL_CPR    4096.0f
// Gearing ratios:
// - Encoder to motor: encoder spins faster than motor (set >1 if geared)
// - Motor to wheel: motor spins faster than wheel (set >1 if geared)
// - Overall encoder to wheel: encoder_to_motor_ratio * motor_to_wheel_ratio

static float encoder_to_motor_ratio = 5.0f;
static float motor_to_wheel_ratio = 5.0f;
static float overall_ratio = 25.0f; // encoder turns per wheel turn
// Conversion: Nm -> Amps (typical VESC with 8-10 pole motor: ~0.5-1.5 A/Nm)
// Adjust this based on your motor's torque constant
#define TORQUE_TO_CURRENT  1.5f
#define CTRL_HZ     1000UL
#define ANGLE_REPORT_HZ 500U
#define VESC_UART_BAUD 115200UL
#define VESC_ENABLE_TELEMETRY_POLL 0


float wheel_angle_rad = 0.0f;
float wheel_vel_rads = 0.0f;
static inline uint32_t usec() { return micros(); }

// Paddle shifter pins (active LOW with internal pull-ups)
const int LEFT_PADDLE_PIN = 4;
const int RIGHT_PADDLE_PIN = 5;

// Pedals: 2x analog Hall sensors (clutch/gas) + 1x HX711 load cell (brake)
const bool PEDALS_ENABLED = true;
const int CLUTCH_HALL_ADC_PIN = 6;
const int GAS_HALL_ADC_PIN = 7;
// 12-bit ADC calibration endpoints (0..4095); adjust after wiring/calibration.
const int CLUTCH_ADC_MIN = 700;
const int CLUTCH_ADC_MAX = 3300;
const int GAS_ADC_MIN = 700;
const int GAS_ADC_MAX = 3300;
const int BRAKE_HX711_DOUT_PIN = 11;
const int BRAKE_HX711_SCK_PIN = 12;
const int32_t BRAKE_FULLSCALE_COUNTS = 100000;

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
  
  // Encoder / gearing configuration
  const uint8_t ENCODER_I2C_ADDR = 0x06; // MT6701 default
  const int MT_UPDATE_MS = 2; // MT6701 internal update interval (ms) - faster polling for high speed
  // Recommended ESP32-S3 default I2C pins (check your board docs) - using SDA=8, SCL=9
  const int I2C_SDA_PIN = 8;
  const int I2C_SCL_PIN = 9;
  
  // Initialize encoder on I2C pins at CTRL_HZ sampling
  encoder_init(I2C_SDA_PIN, I2C_SCL_PIN, CTRL_HZ, ENCODER_I2C_ADDR, encoder_to_motor_ratio, motor_to_wheel_ratio, MT_UPDATE_MS);
  if (PEDALS_ENABLED) {
    pedals_init(
      CLUTCH_HALL_ADC_PIN,
      GAS_HALL_ADC_PIN,
      CLUTCH_ADC_MIN,
      CLUTCH_ADC_MAX,
      GAS_ADC_MIN,
      GAS_ADC_MAX,
        BRAKE_HX711_DOUT_PIN,
        BRAKE_HX711_SCK_PIN,
        BRAKE_FULLSCALE_COUNTS);
  }

  left_paddle_pin = LEFT_PADDLE_PIN;
  right_paddle_pin = RIGHT_PADDLE_PIN;
  hid_init();
  
  delay(100);
  // Initialize hardware UART for VESC on GPIO 15 (RX) and 16 (TX).
  // Keep default at 115200 for maximum compatibility with VESC UART app defaults.
  Serial1.begin(VESC_UART_BAUD, SERIAL_8N1, 15, 16);
  // Attach VescUart to Serial1
  UART.setSerialPort(&Serial1);

  device_gain = 1.0f;
  last_effect_time = millis();

  // configure button pin and FFB gearing
  //button_pin = USER_BUTTON_PIN;
  max_wheel_angle_deg = 900.0f; // lock-to-lock steering range (adjustable)
  angle_limit_stiffness = 300.0f; // Nm/rad spring force at limits (adjustable)
  
  // Set gear ratio for FFB calculations (Motor -> Wheel)
  gear_ratio = motor_to_wheel_ratio;
}



void loop() {
  static uint32_t last_ctrl = 0;
  static uint32_t last_usb = 0;
  static uint32_t last_telemetry = 0;
  static bool first_usb_send = true;
  static bool vesc_control_active = false;
  uint32_t now_us = usec();
  uint32_t now_ms = millis();
  
  // Poll telemetry sparingly (library call is blocking while waiting for response).
  // Keep disabled by default so force loop timing and outgoing current commands are not starved.
#if VESC_ENABLE_TELEMETRY_POLL
  if ((now_ms - last_telemetry) >= 200) {
    last_telemetry = now_ms;
    vesc_request_status5();
  }
#endif

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

  // Take over motor control whenever an effect is active or there is real force demand.
  // This avoids suppressing valid force commands when host actuator-enable sequencing differs.
  bool force_demand = fabsf(amps) > 0.02f;
  bool active_effects = ffb_has_active_effects();
  bool recent_ffb_activity = (millis() - last_effect_time) < 1500;
  bool ff_should_control = active_effects || force_demand || recent_ffb_activity;
  if (ff_should_control) {
    vesc_set_current(amps);
    vesc_control_active = true;
  } else if (vesc_control_active) {
    // Release motor once when FFB stops, then remain silent.
    vesc_set_current(0.0f);
    vesc_control_active = false;
  }
  }
  if ((now_us - last_usb) >= (1000000UL / ANGLE_REPORT_HZ)) {
    last_usb += (1000000UL / ANGLE_REPORT_HZ);
    // Wait a bit after boot before sending USB data to ensure encoder is ready
    if (first_usb_send && now_ms < 500) {
      return; // Skip first 500ms of USB sends
    }
    first_usb_send = false;
    // Refresh steering right before USB send so reports don't stall on cached values.
    encoder_update();
    float latest_centered_angle = encoder_read_centered_angle_rad();
    wheel_angle_rad = latest_centered_angle;
    usb_send_joystick(latest_centered_angle);
  }
}

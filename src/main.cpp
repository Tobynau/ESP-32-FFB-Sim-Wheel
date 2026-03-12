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
#define VESC_UPDATE_HZ  1000U
#define VESC_UART_BAUD 921600UL
#define VESC_ENABLE_TELEMETRY_POLL 0


float wheel_angle_rad = 0.0f;
float wheel_vel_rads = 0.0f;
static inline uint32_t usec() { return micros(); }
static volatile float wheel_angle_snapshot_rad = 0.0f;
static volatile float wheel_vel_snapshot_rads = 0.0f;

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

static TaskHandle_t ffb_task_handle = nullptr;
static TaskHandle_t comm_task_handle = nullptr;

static void ffb_task(void *arg) {
  (void)arg;
  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1);
  bool vesc_control_active = false;
  uint32_t last_vesc_write_us = 0;
  const uint32_t vesc_interval_us = 1000000UL / VESC_UPDATE_HZ;

  for (;;) {
    vTaskDelayUntil(&last_wake, period);

    encoder_update();
    // Pedals run on core 1 so HX711 critical sections don't stall core 0 USB.
    pedals_update();
    float centered_angle = encoder_read_centered_angle_rad();
    float vel_rads = encoder_read_vel_rads();
    wheel_angle_rad = centered_angle;
    wheel_vel_rads = vel_rads;
    wheel_angle_snapshot_rad = centered_angle;
    wheel_vel_snapshot_rads = vel_rads;

    float tau = mix_effects(wheel_angle_rad, wheel_vel_rads);
    float motor_tau = tau / gear_ratio;
    float amps = motor_tau * TORQUE_TO_CURRENT;

    bool force_demand = fabsf(amps) > 0.02f;
    bool active_effects = ffb_has_active_effects();
    bool recent_ffb_activity = (millis() - last_effect_time) < 1500;
    bool ff_should_control = active_effects || force_demand || recent_ffb_activity;

    // Rate-limit VESC UART writes to VESC_UPDATE_HZ (250Hz) to prevent
    // saturating the 115200-baud UART and blocking this task.
    uint32_t now_us = micros();
    bool vesc_due = (now_us - last_vesc_write_us) >= vesc_interval_us;

    if (ff_should_control) {
      if (vesc_due) {
        vesc_set_current(amps);
        last_vesc_write_us = now_us;
      }
      vesc_control_active = true;
    } else if (vesc_control_active) {
      vesc_set_current(0.0f);
      last_vesc_write_us = now_us;
      vesc_control_active = false;
    }
  }
}

static void comm_task(void *arg) {
  (void)arg;

  uint32_t boot_ms = millis();
  uint32_t last_usb_us = usec();
#if VESC_ENABLE_TELEMETRY_POLL
  uint32_t last_vesc_poll_ms = 0;
#endif

  for (;;) {
    // Process as many queued FFB reports as available each cycle to
    // minimise latency between host effect updates and motor output.
    hid_process_reports(32);

    uint32_t now_us = usec();
    uint32_t now_ms = millis();

    if ((now_us - last_usb_us) >= (1000000UL / ANGLE_REPORT_HZ)) {
      last_usb_us += (1000000UL / ANGLE_REPORT_HZ);
      if ((now_ms - boot_ms) >= 500) {
        usb_send_joystick(wheel_angle_snapshot_rad);
      }
    }

    // PID state every cycle — it self-throttles internally (20ms cadence)
    // and is non-blocking (timeout=0), so it won't stall the loop.
    send_pid_state_if_needed();

#if VESC_ENABLE_TELEMETRY_POLL
    if ((now_ms - last_vesc_poll_ms) >= 100) {
      last_vesc_poll_ms = now_ms;
      vesc_request_status5();
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

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
  // Increase TX buffer before begin() to reduce blocking during setCurrent().
  Serial1.setTxBufferSize(256);
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

  xTaskCreatePinnedToCore(ffb_task, "ffb_task", 8192, nullptr, 4, &ffb_task_handle, 1);
  // comm_task at priority 3: must be above default (1) but below the TinyUSB
  // daemon task (~24) so USB protocol handling is never starved.
  xTaskCreatePinnedToCore(comm_task, "comm_task", 8192, nullptr, 3, &comm_task_handle, 0);
}



void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

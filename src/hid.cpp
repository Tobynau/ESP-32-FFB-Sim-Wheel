#include "hid.h"
#include "effects.h"
#include "config.h"
#include <math.h>
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "encoder.h"
#include "pedals.h"

#include "USB.h"
#include "USBHID.h"
USBHID HID;

struct HidReportMsg {
  uint8_t report_id;
  uint8_t len;
  uint8_t data[32];
};

static QueueHandle_t hid_report_queue = nullptr;

// ----------------------
// HID Report Descriptor
//
// Joystick (report 1) + USB PID force feedback + a vendor config collection.
// PID layout follows the spec closely enough for the Windows PID driver
// (units on time fields, ordinal direction instances, axes-enable collection,
// Start Solo) while keeping the shapes Linux hid-pidff identifies reports by
// (logical collections carrying the report usage).
// ----------------------
static const uint8_t report_descriptor[] = {
  0x05, 0x01,                    // Usage Page (Generic Desktop)
  0x09, 0x04,                    // Usage (Joystick)
  0xA1, 0x01,                    // Collection (Application)

    0x09, 0x01,                  // Usage (Pointer)
    0xA1, 0x00,                  // Collection (Physical)
      0x85, 0x01,                // Report ID 1 (Input)
      0x09, 0x30,                // Usage (X) - Steering
      0x16, 0x01, 0x80,          // Logical Min (-32767)
      0x26, 0xFF, 0x7F,          // Logical Max (32767)
      0x75, 0x10,
      0x95, 0x01,
      0x81, 0x02,

      0x09, 0x31,                // Usage (Y) - Clutch
      0x09, 0x32,                // Usage (Z) - Gas
      0x09, 0x35,                // Usage (Rz) - Brake
      0x15, 0x00,                // Logical Min (0)
      0x26, 0xFF, 0x7F,          // Logical Max (32767)
      0x75, 0x10,
      0x95, 0x03,
      0x81, 0x02,

      0x05, 0x09,                // Usage Page (Button)
      0x19, 0x01,
      0x29, 0x08,
      0x15, 0x00,
      0x25, 0x01,
      0x75, 0x01,
      0x95, 0x08,
      0x81, 0x02,
    0xC0,

    0x05, 0x0F,                  // Usage Page (PID)

    // PID State Input report
    0x09, 0x92,                  // Usage (PID State Report)
    0xA1, 0x02,
      0x85, 0x02,                // Report ID 2 (Input)
      0x09, 0x94,                // Actuators Enabled
      0x09, 0x95,                // Safety Switch
      0x09, 0x97,                // Effect Playing
      0x15, 0x00,
      0x25, 0x01,
      0x75, 0x01,
      0x95, 0x03,
      0x81, 0x02,
      0x75, 0x05,
      0x95, 0x01,
      0x81, 0x03,
      0x09, 0x22,                // Effect Block Index
      0x15, 0x00,
      0x25, 0x10,
      0x75, 0x08,
      0x95, 0x01,
      0x81, 0x02,
    0xC0,

    // Set Effect
    // Payload: [idx, duration(2), gain(2), trigger_btn, trigger_repeat(2),
    //           enables, direction1(2), direction2(2), start_delay(2), type]
    0x09, 0x21,
    0xA1, 0x02,
      0x85, 0x03,                // Report ID 3 (Output)
      0x09, 0x22,                // Effect Block Index
      0x15, 0x01,
      0x25, 0x10,
      0x75, 0x08,
      0x95, 0x01,
      0x91, 0x02,

      0x09, 0x50,                // Duration (ms)
      0x15, 0x00,
      0x26, 0xFF, 0x7F,
      0x66, 0x03, 0x10,          // Unit (Eng Lin: Seconds)
      0x55, 0xFD,                // Unit Exponent (-3)
      0x75, 0x10,
      0x95, 0x01,
      0x91, 0x02,

      0x09, 0x52,                // Gain
      0x66, 0x00, 0x00,          // Unit (None)
      0x55, 0x00,
      0x15, 0x00,
      0x26, 0xFF, 0xFF,
      0x75, 0x10,
      0x95, 0x01,
      0x91, 0x02,

      0x09, 0x53,                // Trigger Button
      0x15, 0x00,
      0x25, 0x08,
      0x75, 0x08,
      0x95, 0x01,
      0x91, 0x02,

      0x09, 0x54,                // Trigger Repeat Interval (ms)
      0x15, 0x00,
      0x26, 0xFF, 0x7F,
      0x66, 0x03, 0x10,
      0x55, 0xFD,
      0x75, 0x10,
      0x95, 0x01,
      0x91, 0x02,
      0x66, 0x00, 0x00,          // Unit (None)
      0x55, 0x00,

      0x09, 0x55,                // Axes Enable
      0xA1, 0x02,
        0x05, 0x01,              //   Usage Page (Generic Desktop)
        0x09, 0x30,              //   Usage (X) - the FFB axis
        0x15, 0x00,
        0x25, 0x01,
        0x75, 0x01,
        0x95, 0x01,
        0x91, 0x02,
      0xC0,
      0x05, 0x0F,                // Usage Page (PID)
      0x09, 0x56,                // Direction Enable (1 bit)
      0x91, 0x02,
      0x95, 0x06,                // 6-bit pad to byte boundary
      0x91, 0x03,

      0x09, 0x57,                // Direction
      0xA1, 0x02,
        0x0B, 0x01, 0x00, 0x0A, 0x00, // Usage (Ordinal: Instance 1)
        0x0B, 0x02, 0x00, 0x0A, 0x00, // Usage (Ordinal: Instance 2)
        0x66, 0x14, 0x00,        //   Unit (Eng Rot: Degrees)
        0x55, 0xFE,              //   Unit Exponent (-2)
        0x15, 0x00,
        0x27, 0x9F, 0x8C, 0x00, 0x00, // Logical Max (35999)
        0x35, 0x00,
        0x47, 0x9F, 0x8C, 0x00, 0x00, // Physical Max (35999)
        0x75, 0x10,
        0x95, 0x02,
        0x91, 0x02,
        0x66, 0x00, 0x00,        //   Unit (None)
        0x55, 0x00,
        0x45, 0x00,              //   Physical Max (0 = default)
      0xC0,

      0x09, 0xA7,                // Start Delay (ms)
      0x15, 0x00,
      0x26, 0xFF, 0x7F,
      0x66, 0x03, 0x10,
      0x55, 0xFD,
      0x75, 0x10,
      0x95, 0x01,
      0x91, 0x02,
      0x66, 0x00, 0x00,
      0x55, 0x00,

      // Effect type selector (array)
      0x09, 0x25,
      0xA1, 0x02,
        0x09, 0x26,              //   ET Constant Force
        0x09, 0x27,              //   ET Ramp
        0x09, 0x30,              //   ET Square
        0x09, 0x31,              //   ET Sine
        0x09, 0x32,              //   ET Triangle
        0x09, 0x33,              //   ET Sawtooth Up
        0x09, 0x34,              //   ET Sawtooth Down
        0x09, 0x40,              //   ET Spring
        0x09, 0x41,              //   ET Damper
        0x09, 0x42,              //   ET Inertia
        0x09, 0x43,              //   ET Friction
        0x15, 0x01,
        0x25, 0x0B,
        0x75, 0x08,
        0x95, 0x01,
        0x91, 0x00,
      0xC0,
    0xC0,

    // Set Envelope
    // Payload: [idx, attack_level(2), attack_time(2), fade_level(2), fade_time(2)]
    0x09, 0x5A,
    0xA1, 0x02,
      0x85, 0x04,
      0x09, 0x22,
      0x15, 0x01,
      0x25, 0x10,
      0x75, 0x08,
      0x95, 0x01,
      0x91, 0x02,
      0x09, 0x5B,                // Attack Level
      0x15, 0x00,
      0x26, 0xFF, 0x7F,
      0x75, 0x10,
      0x95, 0x01,
      0x91, 0x02,
      0x09, 0x5C,                // Attack Time (ms)
      0x66, 0x03, 0x10,
      0x55, 0xFD,
      0x75, 0x10,
      0x95, 0x01,
      0x91, 0x02,
      0x66, 0x00, 0x00,
      0x55, 0x00,
      0x09, 0x5D,                // Fade Level
      0x75, 0x10,
      0x95, 0x01,
      0x91, 0x02,
      0x09, 0x5E,                // Fade Time (ms)
      0x66, 0x03, 0x10,
      0x55, 0xFD,
      0x75, 0x10,
      0x95, 0x01,
      0x91, 0x02,
      0x66, 0x00, 0x00,
      0x55, 0x00,
    0xC0,

    // Set Condition
    // Payload: [idx, param_block_offset, cp_offset(2), pos_coeff(2),
    //           neg_coeff(2), pos_sat(2), neg_sat(2), deadband(2)]
    0x09, 0x5F,
    0xA1, 0x02,
      0x85, 0x05,
      0x09, 0x22,
      0x15, 0x01,
      0x25, 0x10,
      0x75, 0x08,
      0x95, 0x01,
      0x91, 0x02,
      0x09, 0x23,                // Parameter Block Offset
      0x15, 0x00,
      0x25, 0x01,
      0x75, 0x08,
      0x95, 0x01,
      0x91, 0x02,
      0x09, 0x60,                // CP Offset
      0x16, 0x01, 0x80,
      0x26, 0xFF, 0x7F,
      0x75, 0x10,
      0x95, 0x01,
      0x91, 0x02,
      0x09, 0x61,                // Positive Coefficient
      0x16, 0x01, 0x80,
      0x26, 0xFF, 0x7F,
      0x75, 0x10,
      0x95, 0x01,
      0x91, 0x02,
      0x09, 0x62,                // Negative Coefficient
      0x16, 0x01, 0x80,
      0x26, 0xFF, 0x7F,
      0x75, 0x10,
      0x95, 0x01,
      0x91, 0x02,
      0x09, 0x63,                // Positive Saturation
      0x15, 0x00,
      0x26, 0xFF, 0x7F,
      0x75, 0x10,
      0x95, 0x01,
      0x91, 0x02,
      0x09, 0x64,                // Negative Saturation
      0x15, 0x00,
      0x26, 0xFF, 0x7F,
      0x75, 0x10,
      0x95, 0x01,
      0x91, 0x02,
      0x09, 0x65,                // Dead Band
      0x15, 0x00,
      0x26, 0xFF, 0x7F,
      0x75, 0x10,
      0x95, 0x01,
      0x91, 0x02,
    0xC0,

    // Set Periodic
    // Payload: [idx, magnitude(2), offset(2), phase(2), period(2)]
    0x09, 0x6E,
    0xA1, 0x02,
      0x85, 0x06,
      0x09, 0x22,
      0x15, 0x01,
      0x25, 0x10,
      0x75, 0x08,
      0x95, 0x01,
      0x91, 0x02,
      0x09, 0x70,                // Magnitude
      0x16, 0x01, 0x80,
      0x26, 0xFF, 0x7F,
      0x75, 0x10,
      0x95, 0x01,
      0x91, 0x02,
      0x09, 0x6F,                // Offset
      0x16, 0x01, 0x80,
      0x26, 0xFF, 0x7F,
      0x75, 0x10,
      0x95, 0x01,
      0x91, 0x02,
      0x09, 0x71,                // Phase (hundredths of degrees)
      0x66, 0x14, 0x00,
      0x55, 0xFE,
      0x15, 0x00,
      0x27, 0x9F, 0x8C, 0x00, 0x00,
      0x75, 0x10,
      0x95, 0x01,
      0x91, 0x02,
      0x09, 0x72,                // Period (ms)
      0x66, 0x03, 0x10,
      0x55, 0xFD,
      0x15, 0x00,
      0x26, 0xFF, 0x7F,
      0x75, 0x10,
      0x95, 0x01,
      0x91, 0x02,
      0x66, 0x00, 0x00,
      0x55, 0x00,
    0xC0,

    // Set Constant Force
    // Payload: [idx, magnitude(2)]
    0x09, 0x73,
    0xA1, 0x02,
      0x85, 0x07,
      0x09, 0x22,
      0x15, 0x01,
      0x25, 0x10,
      0x75, 0x08,
      0x95, 0x01,
      0x91, 0x02,
      0x09, 0x70,                // Magnitude
      0x16, 0x01, 0x80,
      0x26, 0xFF, 0x7F,
      0x75, 0x10,
      0x95, 0x01,
      0x91, 0x02,
    0xC0,

    // Set Ramp Force
    // Payload: [idx, ramp_start(2), ramp_end(2)]
    0x09, 0x74,
    0xA1, 0x02,
      0x85, 0x08,
      0x09, 0x22,
      0x15, 0x01,
      0x25, 0x10,
      0x75, 0x08,
      0x95, 0x01,
      0x91, 0x02,
      0x09, 0x75,                // Ramp Start
      0x16, 0x01, 0x80,
      0x26, 0xFF, 0x7F,
      0x75, 0x10,
      0x95, 0x01,
      0x91, 0x02,
      0x09, 0x76,                // Ramp End
      0x16, 0x01, 0x80,
      0x26, 0xFF, 0x7F,
      0x75, 0x10,
      0x95, 0x01,
      0x91, 0x02,
    0xC0,

    // Effect Operation
    // Payload: [idx, operation (1=Start, 2=Start Solo, 3=Stop), loop_count]
    0x09, 0x77,
    0xA1, 0x02,
      0x85, 0x09,
      0x09, 0x22,
      0x15, 0x01,
      0x25, 0x10,
      0x75, 0x08,
      0x95, 0x01,
      0x91, 0x02,
      0x09, 0x78,                // Effect Operation (selector)
      0xA1, 0x02,
        0x09, 0x79,              //   Op Effect Start
        0x09, 0x7A,              //   Op Effect Start Solo
        0x09, 0x7B,              //   Op Effect Stop
        0x15, 0x01,
        0x25, 0x03,
        0x75, 0x08,
        0x95, 0x01,
        0x91, 0x00,
      0xC0,
      0x09, 0x7C,                // Loop Count
      0x15, 0x00,
      0x26, 0xFF, 0x00,
      0x75, 0x08,
      0x95, 0x01,
      0x91, 0x02,
    0xC0,

    // Device Gain
    0x09, 0x7D,
    0xA1, 0x02,
      0x85, 0x0C,
      0x09, 0x7E,
      0x15, 0x00,
      0x26, 0xFF, 0xFF,
      0x75, 0x10,
      0x95, 0x01,
      0x91, 0x02,
    0xC0,

    // PID Pool Report (Feature)
    0x09, 0x7F,
    0xA1, 0x02,
      0x85, 0x0F,
      0x09, 0x80,                // RAM Pool Size
      0x15, 0x00,
      0x26, 0xFF, 0x7F,
      0x75, 0x10,
      0x95, 0x01,
      0xB1, 0x02,
      0x09, 0x83,                // Simultaneous Effects Max
      0x15, 0x01,
      0x25, 0x10,
      0x75, 0x08,
      0x95, 0x01,
      0xB1, 0x02,
      0x09, 0xA9,                // Device Managed Pool
      0x15, 0x00,
      0x25, 0x01,
      0x75, 0x08,
      0x95, 0x01,
      0xB1, 0x02,
    0xC0,

    // Block Load Report (Feature)
    0x09, 0x89,
    0xA1, 0x02,
      0x85, 0x0E,
      0x09, 0x22,
      0x15, 0x01,
      0x25, 0x10,
      0x75, 0x08,
      0x95, 0x01,
      0xB1, 0x02,
      0x09, 0x8B,                // Block Load Status (selector)
      0xA1, 0x02,
        0x09, 0x8C,              //   Success
        0x09, 0x8D,              //   Full
        0x09, 0x8E,              //   Error
        0x15, 0x01,
        0x25, 0x03,
        0x75, 0x08,
        0x95, 0x01,
        0xB1, 0x00,
      0xC0,
      0x09, 0xAC,                // RAM Pool Available
      0x15, 0x00,
      0x26, 0xFF, 0x7F,
      0x75, 0x10,
      0x95, 0x01,
      0xB1, 0x02,
    0xC0,

    // Block Free
    0x09, 0x90,
    0xA1, 0x02,
      0x85, 0x0A,
      0x09, 0x22,
      0x15, 0x01,
      0x25, 0x10,
      0x75, 0x08,
      0x95, 0x01,
      0x91, 0x02,
    0xC0,

    // Device Control
    0x09, 0x96,
    0xA1, 0x02,
      0x85, 0x0B,
      0x09, 0x97,                // DC Enable Actuators
      0x09, 0x98,                // DC Disable Actuators
      0x09, 0x99,                // DC Stop All Effects
      0x09, 0x9A,                // DC Device Reset
      0x09, 0x9B,                // DC Device Pause
      0x09, 0x9C,                // DC Device Continue
      0x15, 0x01,
      0x25, 0x06,
      0x75, 0x08,
      0x95, 0x01,
      0x91, 0x00,
    0xC0,

    // Create New Effect (Feature)
    0x09, 0xAB,
    0xA1, 0x02,
      0x85, 0x0D,

      // Keep one field directly in this report collection.
      // Helps Linux identify report usage 0xAB reliably.
      0x09, 0x22,
      0x15, 0x00,
      0x25, 0x10,
      0x75, 0x08,
      0x95, 0x01,
      0xB1, 0x02,

      0x09, 0x25,
      0xA1, 0x02,
        0x09, 0x26,
        0x09, 0x27,
        0x09, 0x30,
        0x09, 0x31,
        0x09, 0x32,
        0x09, 0x33,
        0x09, 0x34,
        0x09, 0x40,
        0x09, 0x41,
        0x09, 0x42,
        0x09, 0x43,
        0x15, 0x01,
        0x25, 0x0B,
        0x75, 0x08,
        0x95, 0x01,
        0xB1, 0x00,
      0xC0,
    0xC0,

  0xC0,

  // Vendor configuration collection (WebHID-accessible from the web app;
  // layouts documented in CONFIG_PROTOCOL.md)
  0x06, 0x00, 0xFF,              // Usage Page (Vendor 0xFF00)
  0x09, 0x01,                    // Usage (Vendor 0x01)
  0xA1, 0x01,                    // Collection (Application)
    0x85, 0x60,                  //   Report ID 0x60: config get/set
    0x09, 0x01,
    0x15, 0x00,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, 48,                    //   CONFIG_WIRE_SIZE
    0xB1, 0x02,
    0x85, 0x61,                  //   Report ID 0x61: telemetry (get only)
    0x09, 0x02,
    0x75, 0x08,
    0x95, 20,                    //   TELEMETRY_WIRE_SIZE
    0xB1, 0x02,
  0xC0
};

// ----------------------
// Helpers
// ----------------------
// Convert centered angle to HID report value (-32767..32767) using a configurable half-range.
auto angle_to_hid16 = [](float angle_rad, float half_range_rad) -> int16_t {
  // Safety check for invalid values
  if (!isfinite(angle_rad) || !isfinite(half_range_rad) || half_range_rad <= 0.0f) {
    return 0; // Return center if NaN or infinite
  }

  // Clamp input to configured output range
  float clamped = angle_rad;
  if (clamped > half_range_rad) clamped = half_range_rad;
  if (clamped < -half_range_rad) clamped = -half_range_rad;

  // Map to -32767..32767 where 0 = center
  float scaled = (clamped / half_range_rad) * 32767.0f;
  int v = (int)roundf(scaled);

  // Extra safety clamp
  if (v < -32767) v = -32767;
  if (v > 32767) v = 32767;

  return (int16_t)v;
};

// ----------------------
// HID Device Class
// ----------------------
// PID State - CRITICAL for Windows FFB detection
bool actuators_enabled = false; // Enabled by host via PID Device Control
bool safety_switch = true;      // Indicates no safety issues
uint8_t effect_playing = 0;     // Currently playing effect index (0 = none)

// Assume active until the USB stack reports suspend/stop; torque only flows
// once the host sends effects anyway, so this errs on the working side.
static volatile bool usb_active_flag = true;

bool hid_usb_active() {
  return usb_active_flag;
}

static void usb_event_callback(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  (void)arg; (void)event_data;
  if (event_base != ARDUINO_USB_EVENTS) return;
  switch (event_id) {
    case ARDUINO_USB_STARTED_EVENT:
    case ARDUINO_USB_RESUME_EVENT:
      usb_active_flag = true;
      break;
    case ARDUINO_USB_STOPPED_EVENT:
    case ARDUINO_USB_SUSPEND_EVENT:
      // Host is gone: kill playback so torque can't persist unsupervised.
      usb_active_flag = false;
      effects_all_stop();
      break;
    default:
      break;
  }
}

class CustomHIDDevice : public USBHIDDevice {
public:
  CustomHIDDevice(void) {
    static bool initialized = false;
    if (!initialized) {
      initialized = true;
      HID.addDevice(this, sizeof(report_descriptor));
    }
  }

  void begin(void) { HID.begin(); }

  uint16_t _onGetDescriptor(uint8_t *buffer) {
    memcpy(buffer, report_descriptor, sizeof(report_descriptor));
    return sizeof(report_descriptor);
  }

  void _onOutput(uint8_t reportId, const uint8_t *buffer, uint16_t len) override {
    on_hid_set_report(reportId, (uint8_t *)buffer, len);
  }

  void _onSetFeature(uint8_t reportId, const uint8_t *buffer, uint16_t len) override {
    if (reportId == CONFIG_REPORT_ID) {
      config_apply_wire(buffer, len);
      return;
    }
    if (reportId == 0x0D) {
      // Create New Effect must be handled synchronously: the host GETs the
      // Block Load result right after this SET, before the comm task would
      // drain the queue. Block state is mutex-protected, so this is safe
      // from the USB task.
      handle_ffb_report(reportId, (uint8_t *)buffer, len);
      return;
    }
    on_hid_set_report(reportId, (uint8_t *)buffer, len);
  }

  uint16_t _onGetFeature(uint8_t report_id, uint8_t *buffer, uint16_t len) override {
    if (report_id == CONFIG_REPORT_ID) {
      return config_fill_wire(buffer, len);
    }
    if (report_id == TELEMETRY_REPORT_ID) {
      return telemetry_fill_wire(buffer, len);
    }
    return ffb_get_feature_report(report_id, buffer, len);
  }

  bool send(uint8_t *value, uint16_t len) {
    return HID.SendReport(1, value, len, 0); // non-blocking: dropped frames at 100Hz are invisible
  }
};

CustomHIDDevice Device;

// ----------------------
// Joystick State
// ----------------------
int left_paddle_pin = -1;
int right_paddle_pin = -1;
uint8_t axis[9]; // X,Y,Z,Rz = 8 bytes + 1 byte buttons

static bool pid_state_dirty = true;
static uint8_t pid_state_last_status = 0xFF;
static uint8_t pid_state_last_playing = 0xFF;
static uint32_t pid_state_last_sent_ms = 0;

// Debounced button state, updated from the FFB task at CTRL rate.
static volatile uint8_t buttons_debounced = 0;

static uint8_t read_button_pins() {
  uint8_t bits = 0;
  if (left_paddle_pin >= 0 && digitalRead(left_paddle_pin) == LOW) {
    bits |= 0x01;
  }
  if (right_paddle_pin >= 0 && digitalRead(right_paddle_pin) == LOW) {
    bits |= 0x02;
  }
  return bits;
}

void hid_update_buttons(uint32_t now_ms) {
  static uint8_t last_raw = 0;
  static uint32_t change_ms = 0;
  static uint32_t both_since = 0;
  static bool center_done = false;

  uint8_t raw = read_button_pins();
  if (raw != last_raw) {
    last_raw = raw;
    change_ms = now_ms;
  } else if ((now_ms - change_ms) >= 5) { // 5ms stable = debounced
    buttons_debounced = raw;
  }

  // Hold both paddles 3s to re-center the wheel (one-shot per hold).
  if (buttons_debounced == 0x03) {
    if (both_since == 0) {
      both_since = now_ms;
    } else if (!center_done && (now_ms - both_since) >= 3000) {
      encoder_set_center();
      center_done = true;
    }
  } else {
    both_since = 0;
    center_done = false;
  }
}

uint8_t hid_get_button_bits() {
  return buttons_debounced;
}

static inline void fill_pid_state_report(uint8_t *status, uint8_t *playing) {
  uint8_t local_status = 0;
  uint8_t local_playing = effect_playing;
  if (actuators_enabled) local_status |= 0x01;
  if (safety_switch) local_status |= 0x02;
  if (local_playing > 0) local_status |= 0x04;
  if (status) *status = local_status;
  if (playing) *playing = local_playing;
}

void send_pid_state_if_needed() {
  uint8_t status = 0;
  uint8_t playing = 0;
  fill_pid_state_report(&status, &playing);

  uint32_t now = millis();
  bool changed = (status != pid_state_last_status) || (playing != pid_state_last_playing);
  bool periodic = (now - pid_state_last_sent_ms) >= 20; // Retry frequently since sends are non-blocking

  if (!(pid_state_dirty || changed || periodic)) {
    return;
  }

  uint8_t payload[2] = {status, playing};
  // Non-blocking: drop rather than stall comm_task. The single HID IN
  // endpoint is shared with the joystick report; blocking here stalls
  // FFB report processing and can cascade into a USB reset.
  HID.SendReport(2, payload, sizeof(payload), 0);

  pid_state_last_status = status;
  pid_state_last_playing = playing;
  pid_state_last_sent_ms = now;
  pid_state_dirty = false;
}

void usb_send_joystick(float angle_rad) {
  // Safety: ensure angle is finite
  float ang = isfinite(angle_rad) ? angle_rad : 0.0f;
  float half_range_rad = (g_cfg.wheel_range_deg * PI / 180.0f) * 0.5f;
  if (!isfinite(half_range_rad) || half_range_rad <= 0.0f) {
    half_range_rad = PI;
  }

  int16_t steering = angle_to_hid16(ang, half_range_rad);

  // pedals_update() is called from ffb_task (core 1) to avoid HX711 bit-bang
  // interrupt-disabling from stalling the USB interrupt handler (core 0).
  uint16_t clutch = pedals_get_clutch_hid16();
  uint16_t gas = pedals_get_gas_hid16();
  uint16_t brake = pedals_get_brake_hid16();

  axis[0] = (uint8_t)(steering & 0xFF);
  axis[1] = (uint8_t)((steering >> 8) & 0xFF);
  axis[2] = (uint8_t)(clutch & 0xFF);
  axis[3] = (uint8_t)((clutch >> 8) & 0xFF);
  axis[4] = (uint8_t)(gas & 0xFF);
  axis[5] = (uint8_t)((gas >> 8) & 0xFF);
  axis[6] = (uint8_t)(brake & 0xFF);
  axis[7] = (uint8_t)((brake >> 8) & 0xFF);
  axis[8] = hid_get_button_bits();

  Device.send(axis, sizeof(axis));
  // PID state is sent separately from comm_task at its own cadence to avoid
  // colliding with the joystick report on the shared HID IN endpoint.
}

void hid_init() {
  if (left_paddle_pin >= 0) {
    pinMode(left_paddle_pin, INPUT_PULLUP);
  }
  if (right_paddle_pin >= 0) {
    pinMode(right_paddle_pin, INPUT_PULLUP);
  }
  // Distinct product string so games/users can bind per-device profiles.
  USB.manufacturerName("OpenFFB");
  USB.productName("ESP32-S3 FFB Wheel");
  USB.onEvent(usb_event_callback);
  USB.begin();
  Device.begin();
  if (hid_report_queue == nullptr) {
    hid_report_queue = xQueueCreate(64, sizeof(HidReportMsg));
  }
}

void hid_process_reports(uint32_t max_reports) {
  if (hid_report_queue == nullptr) return;
  HidReportMsg msg;
  uint32_t handled = 0;
  while (handled < max_reports && xQueueReceive(hid_report_queue, &msg, 0) == pdTRUE) {
    handle_ffb_report(msg.report_id, msg.data, msg.len);
    handled++;
  }
}

void hid_notify_pid_state_changed() {
  pid_state_dirty = true;
}

// ----------------------
// Handle FFB Output Reports
// ----------------------
void on_hid_set_report(uint8_t report_id, uint8_t *buf, uint16_t len) {
  if (hid_report_queue == nullptr || buf == nullptr || len == 0) return;

  HidReportMsg msg;
  msg.report_id = report_id;
  uint16_t copy_len = len;
  if (copy_len > sizeof(msg.data)) copy_len = sizeof(msg.data);
  msg.len = (uint8_t)copy_len;
  memcpy(msg.data, buf, copy_len);

  if (xQueueSend(hid_report_queue, &msg, 0) != pdTRUE) {
    HidReportMsg throwaway;
    (void)xQueueReceive(hid_report_queue, &throwaway, 0);
    (void)xQueueSend(hid_report_queue, &msg, 0);
  }
}

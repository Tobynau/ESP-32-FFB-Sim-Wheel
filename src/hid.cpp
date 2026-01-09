#include "hid.h"
#include "effects.h"
#include <math.h>
#include <Arduino.h>
#include "encoder.h"

#include "USB.h"
#include "USBHID.h"
USBHID HID;

// ----------------------
// HID Report Descriptor
// ----------------------
static const uint8_t report_descriptor[] = {
  // Application Collection: Steering Wheel + FFB
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x04,        // Usage (Multi-axis Controller) 
  0xA1, 0x01,        // Collection (Application)
  
    0x09, 0x01,            // Usage (Pointer)
    0xA1, 0x00,            // Collection (Physical)

    // -------- Input Report: Steering Axis + Button --------
    0x85, 0x01,              // Report ID 1 (INPUT)
    
    // Steering Axis
    0x05, 0x01,              // Usage Page (Generic Desktop)
    0x09, 0x30,              // Usage (X) - Steering
    0x16, 0x01, 0x80,        // Logical Minimum (-32767)
    0x26, 0xFF, 0x7F,        // Logical Maximum (32767)
    0x75, 0x10,              // Report Size (16)
    0x95, 0x01,              // Report Count (1)
    0x81, 0x02,              // Input (Data, Var, Abs)

    // Button
    0x05, 0x09,              // Usage Page (Button)
    0x19, 0x01,              // Usage Minimum (Button 1)
    0x29, 0x01,              // Usage Maximum (Button 1)
    0x15, 0x00,              // Logical Minimum (0)
    0x25, 0x01,              // Logical Maximum (1)
    0x75, 0x01,              // Report Size (1)
    0x95, 0x01,              // Report Count (1)
    0x81, 0x02,              // Input (Data, Var, Abs)

    // Padding to complete 1 byte
    0x75, 0x07,
    0x95, 0x01,
    0x81, 0x03,              // Input (Const)
    
    0xC0,                    // End Collection (Physical)

    // ======== PID (Physical Interface Device) Force Feedback ========
    0x05, 0x0F,              // Usage Page (PID)
    0x09, 0x92,              // Usage (PID State Report)
    0xA1, 0x02,              // Collection (Logical)
    
      // Feature Report: PID State (CRITICAL - Windows checks this)
      0x85, 0x02,              // Report ID 2 (Feature)
      0x09, 0x94,              // Usage (Actuators Enabled)
      0x15, 0x00,              // Logical Minimum (0)
      0x25, 0x01,              // Logical Maximum (1)
      0x75, 0x01,              // Report Size (1 bit)
      0x95, 0x01,              // Report Count (1)
      0xB1, 0x02,              // Feature (Data, Var, Abs)
      
      0x09, 0x95,              // Usage (Safety Switch)
      0x75, 0x01,
      0x95, 0x01,
      0xB1, 0x02,              // Feature
      
      0x09, 0x97,              // Usage (Effect Playing)
      0x75, 0x01,
      0x95, 0x01,
      0xB1, 0x02,              // Feature
      
      // Padding for byte alignment
      0x75, 0x05,
      0x95, 0x01,
      0xB1, 0x03,              // Feature (Const)
      
      0x09, 0x22,              // Usage (Effect Block Index)
      0x15, 0x01,              // Logical Minimum (1)
      0x25, 0x10,              // Logical Maximum (16) - max 16 concurrent effects
      0x75, 0x08,              // Report Size (8 bits)
      0x95, 0x01,              // Report Count (1)
      0xB1, 0x02,              // Feature
      
    0xC0,                    // End Collection (PID State)

    // Output Report: Set Effect
    0x09, 0x21,              // Usage (Set Effect Report)
    0xA1, 0x02,              // Collection (Logical)
      0x85, 0x03,              // Report ID 3 (Output)
      0x09, 0x22,              // Usage (Effect Block Index)
      0x15, 0x01,              // Logical Minimum (1)
      0x25, 0x10,              // Logical Maximum (16)
      0x75, 0x08,              // Report Size (8)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
      0x09, 0x25,              // Usage (Effect Type)
      0xA1, 0x02,              // Collection (Logical)
        0x09, 0x26,              // Usage (ET Constant Force)
        0x09, 0x27,              // Usage (ET Ramp)
        0x09, 0x30,              // Usage (ET Square)
        0x09, 0x31,              // Usage (ET Sine)
        0x09, 0x32,              // Usage (ET Triangle)
        0x09, 0x33,              // Usage (ET Sawtooth Up)
        0x09, 0x34,              // Usage (ET Sawtooth Down)
        0x09, 0x40,              // Usage (ET Spring)
        0x09, 0x41,              // Usage (ET Damper)
        0x09, 0x42,              // Usage (ET Inertia)
        0x09, 0x43,              // Usage (ET Friction)
        0x15, 0x01,              // Logical Minimum (1)
        0x25, 0x0B,              // Logical Maximum (11)
        0x75, 0x08,              // Report Size (8)
        0x95, 0x01,              // Report Count (1)
        0x91, 0x00,              // Output
      0xC0,                    // End Collection (Effect Type)
      
      0x09, 0x50,              // Usage (Duration)
      0x15, 0x00,              // Logical Minimum (0)
      0x26, 0xFF, 0x7F,        // Logical Maximum (32767)
      0x75, 0x10,              // Report Size (16)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
      0x09, 0x54,              // Usage (Trigger Button)
      0x15, 0x01,              // Logical Minimum (1)
      0x25, 0x08,              // Logical Maximum (8)
      0x75, 0x08,              // Report Size (8)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
      0x09, 0x51,              // Usage (Sample Period)
      0x15, 0x00,              // Logical Minimum (0)
      0x26, 0xFF, 0x7F,        // Logical Maximum (32767)
      0x75, 0x10,              // Report Size (16)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
      0x09, 0x52,              // Usage (Gain)
      0x15, 0x00,              // Logical Minimum (0)
      0x25, 0xFF,              // Logical Maximum (255)
      0x75, 0x08,              // Report Size (8)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
    0xC0,                    // End Collection (Set Effect)

    // Output Report: Set Envelope
    0x09, 0x5A,              // Usage (Set Envelope Report)
    0xA1, 0x02,              // Collection (Logical)
      0x85, 0x04,              // Report ID 4 (Output)
      0x09, 0x22,              // Usage (Effect Block Index)
      0x15, 0x01,              // Logical Minimum (1)
      0x25, 0x10,              // Logical Maximum (16)
      0x75, 0x08,              // Report Size (8)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
      0x09, 0x5B,              // Usage (Attack Level)
      0x15, 0x00,              // Logical Minimum (0)
      0x26, 0xFF, 0x00,        // Logical Maximum (255)
      0x75, 0x08,              // Report Size (8)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
      0x09, 0x5C,              // Usage (Attack Time)
      0x15, 0x00,              // Logical Minimum (0)
      0x26, 0xFF, 0x7F,        // Logical Maximum (32767)
      0x75, 0x10,              // Report Size (16)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
      0x09, 0x5D,              // Usage (Fade Level)
      0x15, 0x00,              // Logical Minimum (0)
      0x26, 0xFF, 0x00,        // Logical Maximum (255)
      0x75, 0x08,              // Report Size (8)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
      0x09, 0x5E,              // Usage (Fade Time)
      0x15, 0x00,              // Logical Minimum (0)
      0x26, 0xFF, 0x7F,        // Logical Maximum (32767)
      0x75, 0x10,              // Report Size (16)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
    0xC0,                    // End Collection (Set Envelope)

    // Output Report: Set Constant Force
    0x09, 0x5F,              // Usage (Set Constant Force Report)
    0xA1, 0x02,              // Collection (Logical)
      0x85, 0x05,              // Report ID 5 (Output)
      0x09, 0x22,              // Usage (Effect Block Index)
      0x15, 0x01,              // Logical Minimum (1)
      0x25, 0x10,              // Logical Maximum (16)
      0x75, 0x08,              // Report Size (8)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
      0x09, 0x70,              // Usage (Magnitude)
      0x16, 0x01, 0x80,        // Logical Minimum (-32767)
      0x26, 0xFF, 0x7F,        // Logical Maximum (32767)
      0x75, 0x10,              // Report Size (16)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
    0xC0,                    // End Collection (Set Constant Force)

    // Output Report: Set Condition
    0x09, 0x6E,              // Usage (Set Condition Report)
    0xA1, 0x02,              // Collection (Logical)
      0x85, 0x06,              // Report ID 6 (Output)
      0x09, 0x22,              // Usage (Effect Block Index)
      0x15, 0x01,              // Logical Minimum (1)
      0x25, 0x10,              // Logical Maximum (16)
      0x75, 0x08,              // Report Size (8)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
      0x09, 0x23,              // Usage (Parameter Block Offset)
      0x15, 0x00,              // Logical Minimum (0)
      0x25, 0x01,              // Logical Maximum (1)
      0x75, 0x04,              // Report Size (4)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
      0x09, 0x58,              // Usage (Type Specific Block Offset)
      0x15, 0x00,              // Logical Minimum (0)
      0x25, 0x01,              // Logical Maximum (1)
      0x75, 0x04,              // Report Size (4)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
      0x09, 0x6F,              // Usage (CP Offset)
      0x16, 0x01, 0x80,        // Logical Minimum (-32767)
      0x26, 0xFF, 0x7F,        // Logical Maximum (32767)
      0x75, 0x10,              // Report Size (16)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
      0x09, 0x71,              // Usage (Positive Coefficient)
      0x16, 0x01, 0x80,        // Logical Minimum (-32767)
      0x26, 0xFF, 0x7F,        // Logical Maximum (32767)
      0x75, 0x10,              // Report Size (16)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
      0x09, 0x72,              // Usage (Negative Coefficient)
      0x16, 0x01, 0x80,        // Logical Minimum (-32767)
      0x26, 0xFF, 0x7F,        // Logical Maximum (32767)
      0x75, 0x10,              // Report Size (16)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
      0x09, 0x73,              // Usage (Positive Saturation)
      0x15, 0x00,              // Logical Minimum (0)
      0x26, 0xFF, 0x7F,        // Logical Maximum (32767)
      0x75, 0x10,              // Report Size (16)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
      0x09, 0x74,              // Usage (Negative Saturation)
      0x15, 0x00,              // Logical Minimum (0)
      0x26, 0xFF, 0x7F,        // Logical Maximum (32767)
      0x75, 0x10,              // Report Size (16)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
      0x09, 0x75,              // Usage (Dead Band)
      0x15, 0x00,              // Logical Minimum (0)
      0x26, 0xFF, 0x7F,        // Logical Maximum (32767)
      0x75, 0x10,              // Report Size (16)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
    0xC0,                    // End Collection (Set Condition)

    // Output Report: Set Periodic
    0x09, 0x6B,              // Usage (Set Periodic Report)
    0xA1, 0x02,              // Collection (Logical)
      0x85, 0x07,              // Report ID 7 (Output)
      0x09, 0x22,              // Usage (Effect Block Index)
      0x15, 0x01,              // Logical Minimum (1)
      0x25, 0x10,              // Logical Maximum (16)
      0x75, 0x08,              // Report Size (8)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
      0x09, 0x70,              // Usage (Magnitude)
      0x15, 0x00,              // Logical Minimum (0)
      0x26, 0xFF, 0x7F,        // Logical Maximum (32767)
      0x75, 0x10,              // Report Size (16)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
      0x09, 0x6C,              // Usage (Offset)
      0x16, 0x01, 0x80,        // Logical Minimum (-32767)
      0x26, 0xFF, 0x7F,        // Logical Maximum (32767)
      0x75, 0x10,              // Report Size (16)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
      0x09, 0x6D,              // Usage (Phase)
      0x15, 0x00,              // Logical Minimum (0)
      0x26, 0xFF, 0x00,        // Logical Maximum (255)
      0x75, 0x08,              // Report Size (8)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
      0x09, 0x69,              // Usage (Period)
      0x15, 0x00,              // Logical Minimum (0)
      0x26, 0xFF, 0x7F,        // Logical Maximum (32767)
      0x75, 0x10,              // Report Size (16)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
    0xC0,                    // End Collection (Set Periodic)

    // Output Report: Effect Operation
    0x09, 0x77,              // Usage (Effect Operation Report)
    0xA1, 0x02,              // Collection (Logical)
      0x85, 0x08,              // Report ID 8 (Output)
      0x09, 0x22,              // Usage (Effect Block Index)
      0x15, 0x01,              // Logical Minimum (1)
      0x25, 0x10,              // Logical Maximum (16)
      0x75, 0x08,              // Report Size (8)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
      0x09, 0x78,              // Usage (Effect Operation)
      0xA1, 0x02,              // Collection (Logical)
        0x09, 0x79,              // Usage (Op Effect Start)
        0x09, 0x7A,              // Usage (Op Effect Start Solo)
        0x09, 0x7B,              // Usage (Op Effect Stop)
        0x15, 0x01,              // Logical Minimum (1)
        0x25, 0x03,              // Logical Maximum (3)
        0x75, 0x08,              // Report Size (8)
        0x95, 0x01,              // Report Count (1)
        0x91, 0x00,              // Output
      0xC0,                    // End Collection (Effect Operation)
      
      0x09, 0x7C,              // Usage (Loop Count)
      0x15, 0x00,              // Logical Minimum (0)
      0x26, 0xFF, 0x00,        // Logical Maximum (255)
      0x75, 0x08,              // Report Size (8)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
    0xC0,                    // End Collection (Effect Operation)

    // Output Report: Block Free
    0x09, 0x90,              // Usage (Block Free Report)
    0xA1, 0x02,              // Collection (Logical)
      0x85, 0x09,              // Report ID 9 (Output)
      0x09, 0x22,              // Usage (Effect Block Index)
      0x15, 0x01,              // Logical Minimum (1)
      0x25, 0x10,              // Logical Maximum (16)
      0x75, 0x08,              // Report Size (8)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
    0xC0,                    // End Collection (Block Free)

    // Output Report: Device Control
    0x09, 0x96,              // Usage (Device Control Report)
    0xA1, 0x02,              // Collection (Logical)
      0x85, 0x0A,              // Report ID 10 (Output)
      0x09, 0x97,              // Usage (DC Enable Actuators)
      0x09, 0x98,              // Usage (DC Disable Actuators)
      0x09, 0x99,              // Usage (DC Stop All Effects)
      0x09, 0x9A,              // Usage (DC Device Reset)
      0x09, 0x9B,              // Usage (DC Device Pause)
      0x09, 0x9C,              // Usage (DC Device Continue)
      0x15, 0x01,              // Logical Minimum (1)
      0x25, 0x06,              // Logical Maximum (6)
      0x75, 0x08,              // Report Size (8)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x00,              // Output
      
    0xC0,                    // End Collection (Device Control)

    // Output Report: Device Gain
    0x09, 0x7D,              // Usage (Device Gain Report)
    0xA1, 0x02,              // Collection (Logical)
      0x85, 0x0B,              // Report ID 11 (Output)
      0x09, 0x7E,              // Usage (Device Gain)
      0x15, 0x00,              // Logical Minimum (0)
      0x26, 0xFF, 0x00,        // Logical Maximum (255)
      0x75, 0x08,              // Report Size (8)
      0x95, 0x01,              // Report Count (1)
      0x91, 0x02,              // Output
      
    0xC0,                    // End Collection (Device Gain)

  0xC0  // End Application Collection
};

// ----------------------
// Helpers
// ----------------------
// Convert centered angle (-PI..PI) to HID report value (-32767..32767)
auto angle_to_hid16 = [](float angle_rad) -> int16_t {
  // Safety check for invalid values
  if (!isfinite(angle_rad)) {
    return 0; // Return center if NaN or infinite
  }
  
  // Clamp input to -PI..PI range
  float clamped = angle_rad;
  if (clamped > PI) clamped = PI;
  if (clamped < -PI) clamped = -PI;
  
  // Map to -32767..32767 where 0 = center
  float scaled = (clamped / PI) * 32767.0f;
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
bool actuators_enabled = true;  // Must be true for FFB to work
bool safety_switch = true;      // Indicates no safety issues
uint8_t effect_playing = 0;     // Currently playing effect index (0 = none)

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

  bool _onSetReport(uint8_t reportId, uint8_t *buffer, uint16_t len) {
    on_hid_set_report(reportId, buffer, len);
    return true;
  }

  uint16_t _onGetFeature(uint8_t report_id, uint8_t *buffer, uint16_t len) {
    // PID State Feature Report (Report ID 2)
    if (report_id == 0x02) {
      // Report format:
      // Byte 0: Status bits (actuators_enabled, safety_switch, effect_playing)
      // Byte 1: Effect Block Index (1-16, or 0 if none playing)
      
      uint8_t status = 0;
      if (actuators_enabled) status |= 0x01;  // Bit 0: Actuators Enabled
      if (safety_switch) status |= 0x02;       // Bit 1: Safety Switch
      if (effect_playing > 0) status |= 0x04;  // Bit 2: Effect Playing
      
      buffer[0] = status;
      buffer[1] = effect_playing;
      
      return 2;
    }
    return 0;
  }

  bool send(uint8_t *value, uint16_t len) {
    return HID.SendReport(1, value, len); // input report ID 1 (steering/buttons)
  }
};

CustomHIDDevice Device;

// ----------------------
// Joystick State
// ----------------------
uint8_t axis[3]; // 2 bytes for X, 1 byte for button/padding
int button_pin = 0; // 0 = unused

void usb_send_joystick(float angle_rad) {
  // Safety: ensure angle is finite
  float ang = isfinite(angle_rad) ? angle_rad : 0.0f;
  
  int16_t v = angle_to_hid16(ang);
  axis[0] = (uint8_t)(v & 0xFF);
  axis[1] = (uint8_t)((v >> 8) & 0xFF);
  axis[2] = 0;

  if (button_pin != 0 && digitalRead(button_pin) == LOW) {
    axis[2] |= 0x01;
  }

  Device.send(axis, sizeof(axis));
}

void hid_init() {
  USB.begin();
  Device.begin();
}

void hid_task() {
  // Remove the HID.ready() check - just always send
  encoder_update();
  float ang = encoder_read_centered_angle_rad(); // Use centered angle (-PI..PI)
  int16_t v = angle_to_hid16(ang);
  axis[0] = (uint8_t)(v & 0xFF);
  axis[1] = (uint8_t)((v >> 8) & 0xFF);
  axis[2] = 0;

  if (button_pin != 0 && digitalRead(button_pin) == LOW) axis[2] |= 0x01;

  Device.send(axis, sizeof(axis));
  delay(10);
}

// ----------------------
// Handle FFB Output Reports
// ----------------------
void on_hid_set_report(uint8_t report_id, uint8_t *buf, uint16_t len) {
  handle_ffb_report(report_id, buf, len);
}

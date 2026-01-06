#include "hid.h"
#include "effects.h"
#include <math.h>
#include <Arduino.h>
#include "potentiometer.h"

#include "USB.h"
#include "USBHID.h"
USBHID HID;

// ----------------------
// HID Report Descriptor
// ----------------------
static const uint8_t report_descriptor[] = {
  // Application Collection: Steering Wheel + FFB
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x04,        // Usage (Joystick)
  0xA1, 0x01,        // Collection (Application)

    // -------- Input Report: Steering Axis + Button --------
    0x85, 0x01,              // Report ID 1 (INPUT)
    
    // Steering Axis
    0x09, 0x30,              // Usage (X)
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

    // -------- Force Feedback (PID) Outputs --------
    0x05, 0x0F,              // Usage Page (Physical Interface Device)

    // Constant Force
    0x09, 0x26,              // Usage (Constant Force)
    0x85, 0x02,              // Report ID 2
    0x75, 0x10,              // Report Size (16)
    0x95, 0x01,
    0x91, 0x02,              // Output

    // Ramp Force
    0x09, 0x27,
    0x85, 0x03,
    0x75, 0x10,
    0x95, 0x01,
    0x91, 0x02,

    // Spring
    0x09, 0x40,
    0x85, 0x04,
    0x75, 0x10,
    0x95, 0x01,
    0x91, 0x02,

    // Damper
    0x09, 0x41,
    0x85, 0x05,
    0x75, 0x10,
    0x95, 0x01,
    0x91, 0x02,

    // Friction
    0x09, 0x42,
    0x85, 0x06,
    0x75, 0x10,
    0x95, 0x01,
    0x91, 0x02,

    // Periodic Effects: Sine, Square, Triangle, Saw
    0x09, 0x30,
    0x85, 0x07,
    0x75, 0x10,
    0x95, 0x01,
    0x91, 0x02,

    0x09, 0x31,
    0x85, 0x08,
    0x75, 0x10,
    0x95, 0x01,
    0x91, 0x02,

    0x09, 0x32,
    0x85, 0x09,
    0x75, 0x10,
    0x95, 0x01,
    0x91, 0x02,

    0x09, 0x33,
    0x85, 0x0A,
    0x75, 0x10,
    0x95, 0x01,
    0x91, 0x02,

    // Conditional Effect
    0x09, 0x50,
    0x85, 0x0B,
    0x75, 0x10,
    0x95, 0x01,
    0x91, 0x02,

  0xC0  // End Application Collection
};

// ----------------------
// Helpers
// ----------------------
auto angle_to_hid16 = [](float angle_rad) -> int16_t {
  const float two_pi = 2.0f * PI;
  float r = angle_rad / two_pi; // 0..1
  if (r < 0.0f) r += 1.0f;
  if (r >= 1.0f) r -= 1.0f;
  float scaled = r * 65534.0f;
  int v = (int)roundf(scaled) - 32767;
  if (v < -32767) v = -32767;
  if (v > 32767) v = 32767;
  return (int16_t)v;
};

// ----------------------
// HID Device Class
// ----------------------
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
  if (pot_update()) {
    float ang = pot_read_angle_rad();
    int16_t v = angle_to_hid16(ang);
    axis[0] = (uint8_t)(v & 0xFF);
    axis[1] = (uint8_t)((v >> 8) & 0xFF);
    axis[2] = 0;

    if (button_pin != 0 && digitalRead(button_pin) == LOW) {
      axis[2] |= 0x01;
    }

    Device.send(axis, sizeof(axis));
  }
}

void hid_init() {
  Serial.setDebugOutput(true);
  USB.begin();

  Device.begin();
}

void hid_task() {
  if (HID.ready()) {
    pot_update();
    float ang = pot_read_angle_rad();
    int16_t v = angle_to_hid16(ang);
    axis[0] = (uint8_t)(v & 0xFF);
    axis[1] = (uint8_t)((v >> 8) & 0xFF);
    axis[2] = 0;

    if (button_pin != 0 && digitalRead(button_pin) == LOW) axis[2] |= 0x01;

    Device.send(axis, sizeof(axis));
    
    // Debug output every second
    static unsigned long last_debug = 0;
    if (millis() - last_debug > 1000) {
      last_debug = millis();
      Serial.printf("HID: angle=%.3f rad, HID_value=%d\n", ang, v);
    }
    
    delay(10);
  }
}

// ----------------------
// Handle FFB Output Reports
// ----------------------
void on_hid_set_report(uint8_t report_id, uint8_t *buf, uint16_t len) {
  handle_ffb_report(report_id, buf, len);
}

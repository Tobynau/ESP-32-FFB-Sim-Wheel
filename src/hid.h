#ifndef HID_H
#define HID_H

#include "USB.h"
#include "USBHID.h"

extern USBHID HID;

void usb_send_joystick(float angle_rad);
void hid_init();
void hid_task();
void hid_process_reports(uint32_t max_reports);
uint8_t hid_get_button_bits();

// Called by USB stack when a Set_Report/Output/Feature arrives
void on_hid_set_report(uint8_t report_id, uint8_t *buf, uint16_t len);
void hid_notify_pid_state_changed();

// Configurable pins
extern int left_paddle_pin;
extern int right_paddle_pin;

// PID State globals - accessed from effects.cpp
extern bool actuators_enabled;
extern bool safety_switch;
extern uint8_t effect_playing;

#endif // HID_H

#ifndef HID_H
#define HID_H

#include "USB.h"
#include "USBHID.h"

extern USBHID HID;

void usb_send_joystick(float angle_rad);
void hid_init();
void hid_task();

// Called by USB stack when a Set_Report/Output/Feature arrives
void on_hid_set_report(uint8_t report_id, uint8_t *buf, uint16_t len);

// Configurable pins
extern int button_pin;

#endif // HID_H

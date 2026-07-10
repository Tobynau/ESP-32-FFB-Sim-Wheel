#ifndef PEDALS_H
#define PEDALS_H

#include <Arduino.h>

// Initialize pedals. Calibration (ADC endpoints, brake full-scale, filtering,
// brake curve) comes from g_cfg; call pedals_apply_config() after changing it.
// NOTE: HX711 samples at 10Hz unless its RATE pin is tied high (80Hz).
// 80Hz is the minimum for a usable brake — tie RATE high on the pedal PCB.
void pedals_init(
    int clutch_adc_pin,
    int gas_adc_pin,
    int brake_hx711_dout_pin,
    int brake_hx711_sck_pin);

// Pull current calibration/filter settings from g_cfg (live-tunable).
void pedals_apply_config();

// Sample pedals. Call at CTRL rate from the FFB task (core 1) so the HX711
// bit-bang can't stall USB handling on core 0.
void pedals_update();

// Filtered, curved, HID-scaled outputs (0..32767)
uint16_t pedals_get_clutch_hid16();
uint16_t pedals_get_gas_hid16();
uint16_t pedals_get_brake_hid16();

// Raw values for the calibration UI (telemetry report)
uint16_t pedals_get_clutch_raw();
uint16_t pedals_get_gas_raw();
int32_t pedals_get_brake_raw();

#endif // PEDALS_H

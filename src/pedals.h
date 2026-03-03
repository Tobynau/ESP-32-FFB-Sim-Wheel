#ifndef PEDALS_H
#define PEDALS_H

#include <Arduino.h>

void pedals_init(
    int clutch_adc_pin,
    int gas_adc_pin,
    int clutch_adc_min,
    int clutch_adc_max,
    int gas_adc_min,
    int gas_adc_max,
    int brake_hx711_dout_pin,
    int brake_hx711_sck_pin,
    int32_t brake_fullscale_counts);

void pedals_update();

uint16_t pedals_get_clutch_hid16();
uint16_t pedals_get_gas_hid16();
uint16_t pedals_get_brake_hid16();

#endif // PEDALS_H

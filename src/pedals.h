#ifndef PEDALS_H
#define PEDALS_H

#include <Arduino.h>

void pedals_init(
    int i2c_sda_pin,
    int i2c_scl_pin,
    uint8_t clutch_i2c_addr,
    uint8_t gas_i2c_addr,
    float clutch_travel_turns,
    float gas_travel_turns,
    int brake_hx711_dout_pin,
    int brake_hx711_sck_pin,
    int32_t brake_fullscale_counts);

void pedals_update();

uint16_t pedals_get_clutch_hid16();
uint16_t pedals_get_gas_hid16();
uint16_t pedals_get_brake_hid16();

#endif // PEDALS_H

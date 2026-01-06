#ifndef VESC_H
#define VESC_H
#include <VescUart.h>

// VescUart instance (defined in vesc.cpp)
extern VescUart UART;

void vesc_set_current(float amps);
void vesc_request_status5();
void vesc_parse_status5();
void vesc_set_brake_current(float amps);

#endif // VESC_H

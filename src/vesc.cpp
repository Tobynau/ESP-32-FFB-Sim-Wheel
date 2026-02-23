#include "vesc.h"
#include <math.h>

#define MAX_CURRENT_A 20.0f

// VescUart instance
VescUart UART;

void vesc_set_current(float amps) {
    if (!isfinite(amps)) amps = 0.0f;
    if (amps > MAX_CURRENT_A) amps = MAX_CURRENT_A;
    if (amps < -MAX_CURRENT_A) amps = -MAX_CURRENT_A;

    UART.setCurrent(amps);
}

void vesc_request_status5() {
    // With UART we just request new values by calling getVescValues when needed.
    UART.getVescValues();
}

void vesc_parse_status5() {
    
}

void vesc_set_brake_current(float amps) {
    if (!isfinite(amps)) amps = 0.0f;
    if (amps > MAX_CURRENT_A) amps = MAX_CURRENT_A;
    if (amps < -MAX_CURRENT_A) amps = -MAX_CURRENT_A;
    UART.setBrakeCurrent(amps);
}

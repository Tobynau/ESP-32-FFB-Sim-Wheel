#include "vesc.h"
#include <math.h>

static float max_current_a = 20.0f;
static float last_commanded_current_a = 0.0f;
static float last_motor_current_a = 0.0f;

// VescUart instance
VescUart UART;

void vesc_set_current(float amps) {
    if (!isfinite(amps)) amps = 0.0f;
    if (amps > max_current_a) amps = max_current_a;
    if (amps < -max_current_a) amps = -max_current_a;

    UART.setCurrent(amps);
    last_commanded_current_a = amps;
}

void vesc_request_status5() {
    // With UART we just request new values by calling getVescValues when needed.
    if (UART.getVescValues()) {
        last_motor_current_a = UART.data.avgMotorCurrent;
    }
}

void vesc_parse_status5() {
    
}

void vesc_set_brake_current(float amps) {
    if (!isfinite(amps)) amps = 0.0f;
    if (amps > max_current_a) amps = max_current_a;
    if (amps < -max_current_a) amps = -max_current_a;
    UART.setBrakeCurrent(amps);
    last_commanded_current_a = -amps;
}

void vesc_set_max_current(float amps) {
    if (!isfinite(amps)) return;
    if (amps < 1.0f) amps = 1.0f;
    if (amps > 80.0f) amps = 80.0f;
    max_current_a = amps;
}

float vesc_get_max_current() {
    return max_current_a;
}

float vesc_get_last_commanded_current() {
    return last_commanded_current_a;
}

float vesc_get_last_motor_current() {
    return last_motor_current_a;
}

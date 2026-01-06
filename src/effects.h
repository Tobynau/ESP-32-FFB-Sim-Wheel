#ifndef EFFECTS_H
#define EFFECTS_H

#include <Arduino.h>

float mix_effects(float angle, float vel);
// Handle an incoming HID FFB report (report id + raw buffer)
void handle_ffb_report(uint8_t report_id, uint8_t *buf, uint16_t len);

// Effect block types (partial mapping)
enum EffectType {
	ET_NONE = 0,
	ET_CONSTANT = 1,
	ET_RAMP = 2,
	ET_SPRING = 3,
	ET_DAMPER = 4,
	ET_FRICTION = 5,
	ET_PERIODIC = 6,
	ET_CONDITION = 7
};

// Small effect block structure used by the firmware
struct EffectBlock {
	uint8_t in_use;
	uint8_t type; // EffectType
	bool active;
	uint32_t start_time_ms;
	uint32_t duration_ms; // 0 = infinite
	float magnitude; // -1..1
	float magnitude_end; // for ramp
	float direction_rad; // direction in radians
	float freq_hz; // for periodic
	float phase; // periodic phase
	float gain; // 0..1
	// condition params (spring/damper offsets etc)
	float param1;
	float param2;
};

// Max concurrent effect blocks to track
#define MAX_EFFECT_BLOCKS 16

// Enable verbose FFB debug printing when true
extern bool ffb_verbose;


// Configuration: gearing and max wheel angle (radians)
extern float gear_ratio; // motor:wheel (e.g. 10 means motor turns 10x for 1 wheel turn)
extern float max_wheel_angle_rad; // maximum mechanical wheel travel (radians)

extern float device_gain;
extern unsigned long last_effect_time;
extern bool effect_constant_active;
extern float effect_constant_nm;
extern bool effect_spring_active;
extern float effect_spring_k;
extern float effect_spring_center;
extern bool effect_damper_active;
extern float effect_damper_b;
extern bool effect_ramp_active;
extern float effect_ramp_start;
extern float effect_ramp_end;
extern float effect_ramp_duration;
extern unsigned long effect_ramp_start_time;
extern bool effect_inertia_active;
extern float effect_inertia;
// Friction / Periodic / Conditional effects (simple representations)
extern bool effect_friction_active;
extern float effect_friction_coeff;
extern bool effect_periodic_active;
extern int effect_periodic_type; // 0=sine,1=square,2=triangle,3=saw
extern float effect_periodic_amp;
extern float effect_periodic_freq;
extern float effect_conditional_active;
extern float effect_conditional_param;

#endif // EFFECTS_H

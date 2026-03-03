# Force Feedback Configuration Guide

## Overview
The FFB system has been fully implemented with proper torque scaling, effect handling, and angle limiting. The motor receives force commands through the VESC via UART.

## Web App Tuning (Recommended)

This firmware now exposes a **separate USB CDC serial interface** for configuration/telemetry, while games continue to use the **USB HID FFB interface**.

- HID endpoint: game input + FFB effects
- CDC endpoint: web app config + telemetry
- These are separate endpoints on a composite USB device (no shared HID channel).

### Start the web app

From the project root:

```bash
cd webapp
npm install
npm start
```

Open `http://localhost:3000` in a Chromium-based browser, click **Connect**, select the ESP32 serial port, then tune:

- Max angle
- Recenter button
- Overall strength
- Motor max amps
- Per-effect sliders (constant, spring, damper, inertia, friction, periodic)
- Live telemetry (steering angle, paddle/button state, command current, motor current)

### Firmware scheduling model

- High-priority task: FFB control loop (`1 kHz`)
- Low-priority task: USB CDC command parsing + telemetry + VESC polling
- HID callbacks are non-blocking (reports are queued and processed outside USB callbacks)

## Important: Power-On Centering
**The wheel assumes it is centered at whatever position it's in when powered on.** All angle measurements, game output, and FFB effects work relative to this center position:
- Center position (0°) = where the wheel is at power-on
- Turning right (clockwise) = positive angles
- Turning left (counter-clockwise) = negative angles
- The game sees 0 as center, matching the FFB centering force

**Always power on the wheel with it physically centered for best results.**

## Key Configuration Parameters

### Gear Ratios Explained

Your system has **two separate gear stages**:

1. **Encoder → Motor** (for angle measurement)
   - `ENCODER_TO_MOTOR_RATIO = 5.0f` in setup()
   - Encoder spins 5 times per motor revolution
   - Used to calculate wheel angle from encoder readings

2. **Motor → Wheel** (for FFB torque and angle)
   - `MOTOR_TO_WHEEL_RATIO = 5.0f` in setup()
   - Motor spins 5 times per wheel revolution
   - Used to convert wheel torque to motor torque

3. **Overall: Encoder → Wheel**
   - Combined ratio: 5 × 5 = **25:1**
   - Encoder makes 25 revolutions per wheel revolution
   - Calculated automatically by the firmware

**Important:** The `gear_ratio` variable used in FFB calculations is only the motor-to-wheel ratio (5.0), not the encoder ratio. This is correct because FFB torque is applied at the motor, not the encoder.

### In `main.cpp`:

1. **`max_wheel_angle_deg`** (default: 270°)
   - Sets the maximum wheel rotation angle
   - Enforced by a spring force that pushes the wheel back when exceeded
   - Change in setup(): `max_wheel_angle_deg = 270.0f;`
   - Common values: 270°, 360°, 540°, 900°

2. **`angle_limit_stiffness`** (default: 100 Nm/rad)
   - Spring force strength at angle boundaries
   - Higher = harder wall at limits
   - Change in setup(): `angle_limit_stiffness = 100.0f;`
   - Recommended range: 50-200 Nm/rad

3. **`TORQUE_TO_CURRENT`** (default: 1.5 A/Nm)
   - Converts torque (Nm) to motor current (Amps)
   - Depends on your motor's torque constant (Kt)
   - Typical range: 0.5-2.0 A/Nm
   - To calibrate:
     - Start with 1.5
     - If forces feel weak, increase
     - If motor overheats or stutters, decrease

4. **Encoder and Motor Gear Ratios**
   - `ENCODER_TO_MOTOR_RATIO = 5.0f`: encoder spins 5x motor
   - `MOTOR_TO_WHEEL_RATIO = 5.0f`: motor spins 5x wheel
   - Overall encoder-to-wheel: 25:1
   - **Only change these if your mechanical setup is different!**

## Effect Torque Scaling

All FFB effects are now properly scaled to Newton-meters (Nm):

- **Constant Force**: ±10 Nm max
- **Periodic Effects**: ±8 Nm max
- **Spring**: 80 Nm/rad stiffness coefficient (adjustable per effect)
- **Damper**: 15 Nm/(rad/s) damping coefficient (adjustable per effect)
- **Friction**: 5 Nm max coulomb friction
- **Ramp**: Interpolates between start/end magnitudes

## How It Works

### Power-On Centering:
1. On power-up, `pot_init()` reads the encoder position
2. This position is stored as `center_angle`
3. All subsequent angle measurements are relative to this center:
   - `pot_read_centered_angle_rad()` returns angle - center (-π to +π)
   - 0 = center (power-on position)
   - Positive = turned right (clockwise)
   - Negative = turned left (counter-clockwise)

### Force Flow:
1. Game sends FFB effect → USB HID
2. `handle_ffb_report()` parses effect parameters
3. `mix_effects()` computes total torque in Nm at the **wheel**
   - Adds all active effect blocks
   - Adds angle limiting force if beyond bounds (relative to center)
4. Wheel torque ÷ `gear_ratio` (motor-to-wheel ratio) → motor torque
5. Motor torque × `TORQUE_TO_CURRENT` → motor current
6. Current sent to VESC via `vesc_set_current()`

**Note:** Encoder gearing doesn't affect torque calculation - it's only used to convert encoder revolutions to wheel angle.

### Game Communication:
- `usb_send_joystick()` sends centered angle to game
- -32767 = full left (−max_angle_deg/2)
- 0 = center (power-on position)
- +32767 = full right (+max_angle_deg/2)
- Game's centering spring matches actual center position

### Angle Limiting:
- Works relative to center position (0 = center)
- If angle exceeds ±(max_wheel_angle_deg/2):
  - Spring force applied: `F = -stiffness × excess_angle`
  - Pushes wheel back toward limit
  - Creates realistic "end stop" feel
- Example: 270° limit = ±135° from center

## Tuning Guide

### If forces feel too weak:
1. Increase `TORQUE_TO_CURRENT` (e.g., 1.5 → 2.0)
2. Check `device_gain` in effects (set by game, 0-1)
3. Verify VESC current limits in VESC Tool

### If forces feel too strong:
1. Decrease `TORQUE_TO_CURRENT` (e.g., 1.5 → 1.0)
2. Reduce individual effect scaling in [effects.cpp](effects.cpp):
   - Line 73: Constant force `* 10.0f`
   - Line 95: Friction `* 5.0f`
   - Line 110: Periodic `* 8.0f`

### If angle limits feel wrong:
1. Adjust `max_wheel_angle_deg` (270°, 360°, 540°, 900°)
2. Adjust `angle_limit_stiffness`:
   - Too soft: increase (100 → 150)
   - Too hard: decrease (100 → 75)

### If motor overheats:
1. Reduce `TORQUE_TO_CURRENT`
2. Check VESC max current setting
3. Ensure proper motor cooling

## Testing FFB

1. **Position wheel at center** before powering on
2. **Upload firmware** to ESP32-S3
3. **Connect to PC** - should appear as USB game controller
4. **Verify centering** in Windows "Game Controllers" → Properties
   - Wheel should show centered (middle) position
   - Turning left/right should move indicator symmetrically
5. **Test in game** 
   - Enable FFB in game settings
   - Verify centering spring returns wheel to middle
6. **Verify effects**:
   - Spring centering (car on track)
   - Damping (resistance to fast movements)
   - Constant force (road bumps, forces)
   - Periodic (engine vibration, road texture)

## Troubleshooting

### Wheel not centered in game:
- **Power off and position wheel physically at center before powering on**
- The system remembers power-on position as center
- No calibration needed - just start centered

### FFB centering pulls to wrong position:
- Power cycle with wheel at physical center
- Check that `max_wheel_angle_deg` matches your mechanical setup

### FFB not working at all:
- Check `actuators_enabled` flag (set by Device Control report)
- Games must send "Enable Actuators" command (report 0x0A)
- Verify USB enumeration in Device Manager
- Check serial output for FFB debug messages

### Forces in wrong direction:
- Check motor wiring polarity
- May need to negate torque in `vesc_set_current()`

### Jerky/chattering forces:
- Reduce friction effect deadzone (currently 0.01 rad/s)
- Lower VESC current loop gains
- Reduce `TORQUE_TO_CURRENT` conversion

### Angle reading issues:
- Verify encoder I2C connection
- Check `encoder_to_motor_ratio` and `motor_to_wheel_ratio`
- Confirm MT6701 initialization

## Advanced Tuning

Edit effect scaling factors in [effects.cpp](effects.cpp):

```cpp
// Line 73 - Constant force
return mag * b->gain * 10.0f;  // Change 10.0f

// Line 88 - Spring stiffness multiplier
b->param1 = fabsf(int16_to_unit(pos_coeff)) * 80.0f;  // Change 80.0f

// Line 91 - Damper coefficient multiplier  
b->param1 = fabsf(int16_to_unit(pos_coeff)) * 15.0f;  // Change 15.0f

// Line 95 - Friction scaling
float coeff = fabsf(mag) * 5.0f;  // Change 5.0f

// Line 110 - Periodic force
return val * mag * b->gain * 8.0f;  // Change 8.0f
```

## Notes

- All forces are in Newton-meters (Nm) at the wheel
- Motor sees `wheel_torque / gear_ratio`
- Default values tuned for typical 8-10 pole outrunner with 5:1 gearing
- Adjust parameters based on your specific hardware
- Monitor motor temperature during initial testing

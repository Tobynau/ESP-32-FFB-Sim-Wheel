# USB Configuration Notes

## Self-Powered Configuration

This device is **self-powered** through the VESC motor controller, which provides power for the motor. The ESP32-S3 still draws logic power from USB (500mA max).

### Current Configuration
The USB descriptor currently reports as bus-powered with 500mA draw for the ESP32-S3 logic.

### To Make Device Self-Powered (Optional)
To properly indicate self-powered status in the USB descriptor, you would need to modify the TinyUSB configuration in the Arduino ESP32 core. This requires:

1. Create a custom `tusb_config.h` in your project
2. Override the `CFG_TUD_POWER_CONFIG` define
3. Set bit 6 in bmAttributes (0xC0 instead of 0x80)

However, this is **not required** for functionality - Windows will work correctly with the current bus-powered configuration as long as the device draws less than 500mA from USB.

### Current Power Draw
- ESP32-S3 DevKit: ~200-300mA typical
- Motor powered separately by VESC (not from USB)
- Total USB draw: Well within 500mA limit

## Force Feedback Implementation Status

### ✅ Completed
- Complete USB HID PID descriptor with all required collections
- PID State Feature Report (Report ID 0x02) - **CRITICAL for Windows FFB detection**
  - Actuators Enabled flag
  - Safety Switch flag  
  - Effect Playing status
  - Effect Block Index
- All PID Output Reports:
  - Set Effect (0x03) - Create effect with type, duration, gain
  - Set Envelope (0x04) - Attack/fade shaping
  - Set Constant Force (0x05) - Direct torque control
  - Set Condition (0x06) - Spring/Damper/Friction parameters
  - Set Periodic (0x07) - Sine/Square/Triangle waves
  - Effect Operation (0x08) - Start/Stop/Solo commands
  - Block Free (0x09) - Deallocate effects
  - Device Control (0x0A) - Enable/Disable actuators, Stop All, Reset
  - Device Gain (0x0B) - Global FFB strength multiplier
- Effect block system: 16 concurrent effects
- Actuators enabled check in mix_effects()
- Proper effect start/stop handling
- Global device gain support

### Effect Types Supported
1. **Constant Force** - Direct torque output
2. **Ramp** - Linear torque transition
3. **Spring** - Centering force with adjustable stiffness and center position
4. **Damper** - Velocity-dependent resistance
5. **Friction** - Constant resistance to movement
6. **Periodic** - Sine/Square/Triangle/Sawtooth waveforms
7. **Condition** - Custom condition-based effects

### VESC Integration
- Motor control via UART on Serial1 (pins 15/16, 115200 baud)
- Current limiting: ±2A default
- Real-time torque output at 1kHz
- Self-powered operation (no PC software required)

## Testing Procedure

1. **Device Recognition**
   - Open Device Manager → Should show "USB Input Device" with no errors
   - Open joy.cpl → Device should appear with working steering axis

2. **Force Feedback Detection**
   - Open joy.cpl → Select device → Properties
   - Look for "Force feedback is supported" message
   - If missing, Windows didn't receive proper PID State Feature Report

3. **Test Forces**
   - In joy.cpl Properties → "Test Forces" button
   - Wheel should resist movement when button is clicked
   - Try different force types (constant, spring, damper)

4. **Game Integration**
   - Launch racing sim (MudRunner, Assetto Corsa, etc.)
   - Configure steering axis in game settings
   - Enable force feedback in game options
   - Test in-game: should feel road bumps, centering force, collision effects

## Troubleshooting

### "Force feedback is not supported" in joy.cpl
- Check that actuators_enabled = true in hid.cpp
- Verify _onGetFeature() is properly implemented
- Use USB analyzer to confirm Feature Report 0x02 is being sent

### Forces not working in games
- Check Device Control Report is received (Enable Actuators command)
- Verify Effect Operation Reports are parsed correctly
- Enable ffb_verbose in effects.cpp and check Serial output

### Weak or no resistance
- Check Device Gain Report (should be 0-255, default 255)
- Verify VESC current limits (±2A default, can increase if needed)
- Check mix_effects() is calculating non-zero torque values
- Verify vesc_set_current() is being called from main loop

### Encoder not working
- Check I2C pins: SDA=8, SCL=9
- Verify MT6701 address: 0x06
- Check encoder polling rate: 10ms (100Hz)
- Verify overall_ratio calculation: 25:1 encoder:wheel


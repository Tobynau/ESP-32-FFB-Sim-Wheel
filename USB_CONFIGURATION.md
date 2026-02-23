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

## Force Feedback Implementation Status (Linux HID PID)

### ✅ Completed
- Linux-compatible USB HID PID descriptor aligned with `hid-pidff`
- Required Linux PID reports implemented:
   - Set Effect (`Usage 0x21`, Report ID `0x03`)
   - Effect Operation (`Usage 0x77`, Report ID `0x09`)
   - Device Gain (`Usage 0x7D`, Report ID `0x0C`)
   - PID Pool Feature (`Usage 0x7F`, Report ID `0x0F`)
   - Block Load Feature (`Usage 0x89`, Report ID `0x0E`)
   - Block Free (`Usage 0x90`, Report ID `0x0A`)
   - Device Control (`Usage 0x96`, Report ID `0x0B`)
   - Create New Effect Feature (`Usage 0xAB`, Report ID `0x0D`)
- Optional/extended effect reports implemented:
   - Set Envelope (`0x04`), Set Condition (`0x05`), Set Periodic (`0x06`), Set Constant (`0x07`), Set Ramp (`0x08`)
- PID State Feature Report (Report ID `0x02`) retained for compatibility/diagnostics
  - Actuators Enabled flag
  - Safety Switch flag  
  - Effect Playing status
  - Effect Block Index
- Effect block system: 16 concurrent effects
- Device-managed PID memory pool and block-load status reporting
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

## Linux Testing Procedure (Ubuntu)

1. **Build + Flash**
   - Flash firmware to ESP32-S3 (PlatformIO/Arduino workflow)

2. **Verify Linux sees FF capabilities**
   - Install tools: `sudo apt install -y evtest joystick`
   - Find event node: `grep -H . /sys/class/input/event*/device/name`
   - Check capabilities: `evtest /dev/input/eventX`
   - Confirm FF types include at least: `FF_CONSTANT`, `FF_PERIODIC`, `FF_SPRING`, `FF_DAMPER`, `FF_FRICTION`, `FF_INERTIA`, `FF_GAIN`

3. **Run force feedback test**
   - Run: `fftest /dev/input/eventX`
   - Trigger constant/spring/periodic effects and verify wheel response

4. **Game Integration**
   - Launch game (BeamNG.drive via Proton, MudRunner, etc.)
   - Configure steering axis in game settings
   - Enable force feedback in game options
   - Test in-game: should feel road bumps, centering force, collision effects

## Troubleshooting

### Linux does not expose FF_* capabilities
- Check kernel module path: `dmesg | grep -Ei 'hid|input|ff|pid'`
- Confirm descriptor includes PID usages: `0x21,0x77,0x7D,0x7F,0x89,0x90,0x96,0xAB`
- Verify `Create New Effect` and `Block Load` feature reports respond

### Forces not working in games
- Check Device Control Report is received (Enable Actuators command)
- Verify Effect Operation Reports are parsed correctly
- Enable ffb_verbose in effects.cpp and check Serial output

### Weak or no resistance
- Check Device Gain Report (16-bit, default full scale)
- Verify VESC current limits (±2A default, can increase if needed)
- Check mix_effects() is calculating non-zero torque values
- Verify vesc_set_current() is being called from main loop

### Encoder not working
- Check I2C pins: SDA=8, SCL=9
- Verify MT6701 address: 0x06
- Check encoder polling rate: 10ms (100Hz)
- Verify overall_ratio calculation: 25:1 encoder:wheel


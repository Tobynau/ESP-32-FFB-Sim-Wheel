# ESP32 FFB Wheel Web App

Local browser-based tuning app for the ESP32 FFB wheel using Web Serial.

## Run

```bash
cd webapp
npm install
npm start
```

Open http://localhost:3000 in a Chromium-based browser.

## Why this design

- The game communicates over USB HID (FFB + joystick reports).
- This app communicates over USB CDC Serial (Web Serial API).
- HID and CDC are separate interfaces in the composite USB device, so the game and app do not contend for the same endpoint.

## Serial protocol

Commands sent by the web app:

- `get`
- `recenter`
- `ping`
- `set <key> <value>`

Config keys:

- `max_angle_deg`
- `angle_limit_stiffness`
- `overall_strength`
- `motor_max_amps`
- `device_gain`
- `scale_constant`
- `scale_spring`
- `scale_damper`
- `scale_inertia`
- `scale_friction`
- `scale_periodic`

Firmware emits newline-delimited JSON:

- `{"type":"config", ...}`
- `{"type":"telemetry", ...}` (20 Hz)
- `{"type":"ok", ...}` / `{"type":"error", ...}`

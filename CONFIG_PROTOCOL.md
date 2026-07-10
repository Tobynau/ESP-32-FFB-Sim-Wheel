# Vendor Configuration Protocol (WebHID)

The wheel exposes a vendor HID collection (usage page `0xFF00`) with two
feature reports for live configuration and telemetry. The web app can reach
them via **WebHID** with no driver:

```js
const [device] = await navigator.hid.requestDevice({
  filters: [{ usagePage: 0xFF00 }]
});
await device.open();
const cfg = await device.receiveFeatureReport(0x60);   // read config
await device.sendFeatureReport(0x60, payload);         // write config
const tel = await device.receiveFeatureReport(0x61);   // read telemetry
```

All multi-byte fields are **little-endian**. Fixed-point scale is noted per
field.

## Report 0x60 — Configuration (GET/SET, 48 bytes)

| Offset | Size | Field | Scale / notes |
|---|---|---|---|
| 0  | 1 | version | must be 1 |
| 1  | 1 | command | SET only, see below; reads back 0 |
| 2  | 2 | wheel_range_deg | lock-to-lock degrees (90–1800) |
| 4  | 2 | max_torque_cnm | rim torque limit, Nm × 100 |
| 6  | 2 | torque_to_current_c | motor A per motor Nm × 100 |
| 8  | 2 | endstop_k_dnm | end-stop spring, Nm/rad × 10 |
| 10 | 2 | endstop_d_cnm | end-stop damping, Nm/(rad/s) × 100 |
| 12 | 2 | endstop_max_cnm | end-stop torque cap, Nm × 100 |
| 14 | 1 | recon_cutoff_hz | constant-force smoothing LPF; 0 = off |
| 15 | 2 | slew_cnm_per_ms | torque slew limit, Nm/ms × 100; 0 = off |
| 17 | 2 | base_damper_mnm | always-on damper, Nm/(rad/s) × 1000 |
| 19 | 2 | base_friction_mnm | always-on friction, Nm × 1000 |
| 21 | 2 | base_inertia_mnm | always-on inertia, Nm/(rad/s²) × 1000 |
| 23 | 1 | flags | bit0 invert FFB, bit1 soft clip, bit2 oscillation guard, bit3 breakaway boost |
| 24 | 2 | breakaway_mnm | breakaway boost, Nm × 1000 |
| 26 | 2 | clutch_adc_min | raw 12-bit ADC |
| 28 | 2 | clutch_adc_max | raw 12-bit ADC |
| 30 | 2 | gas_adc_min | raw 12-bit ADC |
| 32 | 2 | gas_adc_max | raw 12-bit ADC |
| 34 | 4 | brake_fullscale_counts | HX711 counts at full braking force |
| 38 | 1 | pedal_filter | 0–3 (off → heavy EMA) |
| 39 | 1 | brake_curve | 0 linear, 1 mild, 2 progressive |
| 40 | 8 | reserved | write 0 |

### Commands (byte 1 on SET)

| Value | Action |
|---|---|
| 0 | Apply settings to RAM only |
| 1 | Apply settings and save to NVS (persists across boots) |
| 2 | Set current wheel position as center (other fields ignored) |
| 3 | Restore factory defaults (RAM only; follow with command 1 to persist) |
| 4 | Discard RAM changes, reload last saved settings |

All values are range-clamped by the firmware on apply. Center can also be set
without the tool by holding **both paddles for 3 seconds**.

## Report 0x61 — Telemetry (GET only, 20 bytes)

| Offset | Size | Field | Scale / notes |
|---|---|---|---|
| 0  | 4 | angle_mrad | centered wheel angle, milliradians (signed) |
| 4  | 2 | vel_crads | wheel velocity, rad/s × 100 (signed) |
| 6  | 2 | torque_cnm | commanded rim torque, Nm × 100 (signed) |
| 8  | 2 | clutch_raw | raw ADC (for calibration UI) |
| 10 | 2 | gas_raw | raw ADC (for calibration UI) |
| 12 | 4 | brake_raw | HX711 counts above zero point (signed) |
| 16 | 1 | flags | bit0 actuators enabled, bit1 paused, bit2 USB active, bit3 FFB watchdog fading |
| 17 | 1 | active_blocks | allocated PID effect blocks |
| 18 | 1 | effect_playing | playing block index, 0 = none |
| 19 | 1 | version | currently 1 |

## Calibration flow (web app)

1. **Pedals:** poll telemetry while the user moves each pedal through full
   travel; record min/max of `clutch_raw`/`gas_raw` and the max of
   `brake_raw` at the user's preferred maximum braking force; write them back
   with command 1.
2. **Center:** ask the user to hold the wheel straight, send command 2.
3. **Feel tuning:** write settings with command 0 while the user drives;
   persist with command 1 once happy.

# Haptic Polish & Game-Compatibility Plan

> **STATUS (2026-07-09): Implemented.** All phases below are now in the firmware
> (commit implementing this plan). What remains is **hardware verification**:
> - Direction convention (B1): verify constant-force sign on Linux `fftest` AND a
>   Windows/DirectInput title; flip the `invert_ffb` config flag if reversed.
> - Descriptor re-enumeration: unplug/replug after flashing; on Windows, delete the
>   cached device entry (Device Manager → HID) if effects fail to enumerate.
> - Tune `max_torque_nm`, end-stop, reconstruction filter, and base-feel defaults on
>   the real rig via the vendor config report (see CONFIG_PROTOCOL.md).
> - HX711 RATE pin (D4): still a hardware action before the pedal PCB.
> - Config/telemetry vendor HID reports are live for the web app (WebHID).

Status of codebase at time of writing: steering + FFB confirmed working (Linux hid-pidff),
pedals/buttons untested (hardware not built). This document lists the issues found in a
full review, then a phased plan to get a polished, commercial-grade haptic feel that works
with as many games as possible out of the box.

---

## Part 1 — Issues found in review

### A. Safety-critical

**A1. `actuators_enabled` is never enforced.**
`USB_CONFIGURATION.md` claims "Actuators enabled check in mix_effects()" but no such check
exists in `effects.cpp`. The host's PID Device Control *Disable Actuators* (report 0x0B,
control=2) sets the flag, and the PID State report echoes it back, but torque keeps
flowing. Games and OS drivers use this as the emergency-stop path.
→ `mix_effects()` (or `ffb_task`) must output 0 Nm when `actuators_enabled == false`.

**A2. No host-disappearance failsafe.**
If a game crashes or the USB cable drops while a constant-force block is active, the block
stays `active` forever and `ffb_task` keeps commanding current indefinitely (the
`recent_ffb_activity` window never matters because `ffb_has_active_effects()` stays true).
The 500 ms `EFFECT_TIMEOUT_MS` only clears the *legacy* flags, which nothing sets anymore.
→ Needed: (1) TinyUSB suspend/unmount callback → stop all effects, zero current;
(2) a "host silence" watchdog: if no FFB output report has arrived for ~2 s while a
constant/ramp force is active, ramp forces to zero (springs/dampers may persist — they are
position-based and safe).

**A3. No torque clamp in Nm — only a 20 A motor-current clamp.**
With `TORQUE_TO_CURRENT = 1.5 A/Nm` and 5:1 motor→wheel gearing, 20 A ≈ 13 Nm at the motor
≈ **66 Nm at the rim**. The end-stop spring (300 Nm/rad, no damping) can command that full
amount and will oscillate against it.
→ Add `MAX_TORQUE_NM` (rim torque, user-configurable, e.g. default 8 Nm) and clamp the
mixed torque *before* gearing/current conversion. Make the 20 A value match the motor's
actual rating.

**A4. End-stop is an undamped spring → violent bounce at the lock.**
→ Add damping inside the end-stop zone (oppose inward velocity only) and cap end-stop
torque separately from game torque.

### B. Game-compatibility bugs (why many games will feel wrong today)

**B1. Effect Direction is parsed but ignored.**
`b->direction_rad` is stored in Set Effect (0x03) but `compute_block_force()` never uses
it for constant, periodic, or ramp forces. DirectInput games conventionally send a
*positive* magnitude plus a direction angle; sign/direction of the force must come from
projecting that angle onto the steering axis. Today, force direction is only correct for
hosts that happen to encode sign in the magnitude (Linux does; many Windows titles don't).
This is the single biggest "works in game X, backwards/one-sided in game Y" source.
→ Apply `torque = magnitude * cos_or_sin(direction)` projection (calibrate the axis
convention against a known-good game + `fftest`/fedit), respect the Direction Enable flag,
fall back to signed magnitude when direction is disabled.

**B2. Set Condition (0x05) mangles spring/damper parameters.**
- Uses `max(|pos_coeff|, |neg_coeff|)` → loses sign (negative springs, used by some games
  for drift/counter-steer effects, become positive) and loses asymmetry.
- **Deadband and saturation are parsed but never applied** — they're stashed into
  `b->magnitude` / `b->magnitude_end`, fields that other code paths also use.
- CP offset is scaled to ±180° (`* M_PI`) instead of the axis logical range (±450° on a
  900° wheel). Games that move the spring center (soft-lock, drift assists) will be 2.5×
  off.
- The *parameter block offset* byte (buf[1]) is ignored, so if a host sends a second-axis
  condition block it overwrites the steering-axis parameters.
→ Implement the full PID condition model per axis 0 only (ignore block offset ≠ 0):

```
if angle < (cp - deadband):  t = -neg_coeff * (angle - (cp - deadband)), clamped to neg_sat
if angle > (cp + deadband):  t = -pos_coeff * (angle - (cp + deadband)), clamped to pos_sat
else: t = 0
```

with cp scaled by the wheel's half-range, and the same structure (velocity/accel in place
of angle) for damper/inertia/friction.

**B3. Duration heuristic corrupts short effects.**
`duration < 32` is treated as *seconds*, so a 20 ms rumble pulse (curb hit, gear knock)
becomes a 20-*second* stuck effect. The right fix is to declare units in the descriptor
(Unit = seconds, Unit Exponent = -3) and trust milliseconds directly; delete the
heuristic.

**B4. Envelope (0x04) is accepted and discarded.**
Attack/fade shaping is what makes gear shifts, curbs and collisions feel like *events*
instead of on/off clicks. Older games (pre-2010 DirectInput titles) rely on it heavily.
→ Store attack_level/attack_time/fade_level/fade_time per block and modulate the block's
magnitude envelope in `compute_block_force()`.

**B5. Periodic offset ignored; start delay ignored; loop count ignored.**
Offset biases engine-rumble effects; start delay + loop count matter for scripted
multi-pulse effects. All are already in the reports — just parse and honor them.

**B6. Create New Effect (0x0D) parse is a byte-scan.**
Scanning the payload for "any byte 1..11" will misfire (e.g. a block-index byte of 1–11).
The descriptor layout is fixed: byte 0 = effect block index field, byte 1 = effect type.
Parse it positionally.

**B7. Windows descriptor gaps.**
The descriptor was tuned for Linux `hid-pidff` (lenient). The Windows PID driver
(dinput → hidusb) is stricter. Known gaps:
- No Unit/Unit Exponent on Duration/Sample Period → hosts guess time units (see B3).
- Direction collection contains a single instance; PID spec wants ordinal instances per
  axis (games sending 2-axis direction will have offsets shifted).
- Effect Operation exposes Start(1)/Stop(2) but not Start Solo — Windows enumerates
  Op Effect Start / Start Solo / Stop as 1/2/3; a host sending "2" may mean Start Solo.
- Axes Enable is a lone 1-byte field rather than a collection referencing the X usage.
→ Validate against Windows with the descriptor tooling below; fix layout while keeping
Linux working (Linux is tolerant of the stricter/correct form).

**B8. Identity: Espressif VID/PID and generic product name.**
Sims with wheel auto-detect (per-device FFB profiles, soft-lock support) key off
VID/PID/name. You can't borrow a commercial VID, but set a distinctive product string
("<YourName> FFB Wheel") via `USB.productName()` so games/users can bind profiles, and
document manual profile setup for the major sims.

### C. Feel quality (works, but feels rough)

**C1. Arbitrary per-effect torque scales.**
Constant → ×10 Nm, periodic → ×8 Nm, spring → ×180 Nm/rad, damper → ×90, friction → ×12,
inertia → ×6: all magic numbers. The *relative balance* between effect types is what games
tune against; with inconsistent scales, any game mixing spring + constant force feels
wrong and every game needs different in-game gain.
→ Define one `MAX_TORQUE_NM`; full-scale constant/periodic/ramp = MAX_TORQUE_NM; full
scale condition coefficient = MAX_TORQUE_NM at full axis deflection (per PID semantics).
One number calibrates everything.

**C2. The `derived_vel` "take whichever is larger" hack injects spikes.**
In `mix_effects()`, velocity is replaced by the raw angle-derivative whenever it's larger
— i.e. it selects for *noise*. Dampers and friction turn grainy. Remove it and fix the
velocity estimate at the source instead (C4).

**C3. No output conditioning: stair-steps and clunks pass straight to the motor.**
Modern sims (iRacing, AC/ACC, rFactor 2, AMS2) send *only constant force*, updated at
60–360 Hz. The firmware samples that at 1 kHz with zero interpolation, so the rim
reproduces the host's update staircase — this is the #1 source of "grainy/notchy" feel on
DIY wheels. Commercial firmware (Simucube "reconstruction filter", VRS, Fanatec
interpolation) smooths/predicts between host updates.
→ Add a torque post-chain: constant-force interpolation/LPF (configurable strength) →
total-torque slew limiter → final clamp. Keep springs/dampers computed at full 1 kHz from
local angle/velocity (they must NOT be smoothed — their crispness is what makes the wheel
feel connected).

**C4. Velocity estimate: 1-pole IIR (α=0.1 @1 kHz) on I2C-quantized deltas.**
~16 Hz effective bandwidth plus quantization jitter → phase-lagged, gritty dampers.
→ Replace with a tracking-loop / alpha-beta estimator on the continuous angle, or a
least-squares slope over the last 8–16 samples. Target: clean velocity to ≥50 Hz.

**C5. Friction is bang-bang coulomb with a dead zone → chatter at low speed.**
→ Saturated-linear model: `t = -coeff * clamp(vel / v_knee, -1, 1)` with `v_knee` ≈
0.05–0.1 rad/s. Smooth, no chatter, still feels like friction.

**C6. No device-side base feel.**
High-end wheels layer small always-on damping/friction/inertia under the game's FFB —
it masks motor cogging and makes hands-off behavior stable. Also: no hands-off oscillation
protection (constant-force games + low grip = wheel shakes itself violently).
→ Add configurable base damper/friction/inertia, plus an oscillation detector (sustained
high-frequency sign-alternating torque at high velocity → temporarily raise damping).

### D. Robustness / hygiene

**D1. Cross-core races on effect blocks.** `comm_task` (core 0) writes blocks while
`ffb_task` (core 1) reads and *also writes* them (`b->active=false` on expiry). Multi-field
updates (type + params) can tear mid-mix. → Guard block mutation + mixing with a
`portMUX` critical section (the mix loop is short) or double-buffer parameters.

**D2. Legacy effect globals are dead code.** Nothing sets `effect_constant_active` et al.
anymore; they add branches, extern spaghetti, and a fake "timeout" path. Delete them.

**D3. `hid_task()` is dead code** (and calls `encoder_update()` from the wrong core if
ever used). Delete.

**D4. HX711 brake: default sample rate is 10 SPS.**
Unless the RATE pin is tied high (80 SPS), the brake axis updates at 10 Hz — unusable for
braking. **Hardware action before the pedal PCB is finalized:** tie RATE high, or use a
faster ADC (ADS1220/ADS1256). 80 SPS is the floor for acceptable braking; consider it a
minimum, not a target.

**D5. Repo is on a detached HEAD.** Commit work to a branch before starting this plan.

**D6. Pedals have no filtering or auto-calibration.** Raw 12-bit `analogRead` per 2 ms →
jittery axes; fixed ADC endpoints require re-flashing to calibrate. → light EMA or median
filter + NVS-stored min/max calibration (see Phase 4).

---

## Part 2 — Phased plan

Priority logic: safety first, then the constant-force path (it's 100% of the signal for
modern sims), then full PID coverage for older titles, then feel tuning, then config
tooling.

### Phase 0 — Safety & correctness (do first, small diffs)
1. Gate all output on `actuators_enabled`; implement Stop All / Reset / **Pause /
   Continue** (pause = freeze playback clocks + zero output; continue = resume).
2. USB suspend/unmount → stop all effects, command 0 A once, stop UART writes (let the
   VESC app timeout be the backstop, and set the VESC timeout to ~100 ms).
3. `MAX_TORQUE_NM` clamp at the rim before current conversion; verify the 20 A clamp
   against the real motor spec.
4. End-stop: add damping term + separate torque cap; make stiffness/damping constants
   config values.
5. Host-silence watchdog for constant/ramp forces (~2 s → ramp to zero over ~200 ms, not
   a hard cut).
6. Wrap effect-block access in a critical section (D1).
7. Delete legacy effect globals and `hid_task()` (D2, D3).

**Acceptance:** yank USB mid-force → wheel goes limp within 100 ms. `fftest` constant
force → clean stop on Ctrl-C. Slam into end-stop → firm, no bounce, no scream.

### Phase 1 — Constant-force path polish (biggest feel win per line of code)
1. Direction projection for constant/periodic/ramp (B1), verified against a signed-
   magnitude host (Linux fftest) *and* a direction-encoding host (Windows game or fedit).
2. Unified torque scaling (C1): one `MAX_TORQUE_NM`, all effect full-scales derived
   from it.
3. Constant-force reconstruction: per-block LPF (first-order, cutoff configurable
   ~20–80 Hz, "off" option) + global torque slew limiter (e.g. max ΔNm/ms configurable).
4. Velocity estimator rewrite (C4); delete the `derived_vel` hack (C2).
5. Duration fix: declare time units in the descriptor, delete the <32→seconds heuristic
   (B3).

**Acceptance:** in a modern sim (AC or BeamNG on Proton, or iRacing on the Windows test
box), road texture feels continuous rather than gritty; no direction reversals; short
pulse effects (curbs) end when they should.

### Phase 2 — Full PID coverage (out-of-box compat with older/complex titles)
1. Set Condition done right (B2): signed asymmetric coefficients, deadband, saturation,
   cp offset scaled to axis range, ignore block offset ≠ 0.
2. Envelope engine (B4): attack/fade applied to constant, ramp, periodic magnitudes.
3. Periodic offset, start delay, loop count (B5). Periodic phase accumulation should be
   incremental (`phase += 2π·f·dt`) so frequency changes don't cause phase jumps.
4. Positional Create New Effect parse (B6).
5. Friction model fix (C5); inertia uses the improved accel estimate.
6. Descriptor hardening for Windows (B7): units, direction ordinals, Start Solo, axes
   enable collection. Validate with:
   - Windows: `fedit.exe` (FFB effect editor, exercises every report), DIView,
     `hidusbf`-style descriptor dumps.
   - Linux: `fftest`, `ffbtools` (`ffbwrap --logger` shows exactly what games send —
     use it to build regression fixtures).
7. Set `USB.productName` / manufacturer (B8).

**Acceptance matrix (test each, note result):**
| Host | Test | Exercises |
|---|---|---|
| Linux | `fftest` all effect types | full PID surface |
| Linux/Proton | Assetto Corsa, BeamNG | constant force @ high rate |
| Linux/Proton | Dirt Rally 2 | spring + periodic + envelope |
| Windows | fedit.exe every effect | descriptor strictness |
| Windows | an old DI title (e.g. NFS, GTR2) | envelope, ramps, triggers |
| Windows | iRacing/ACC if available | 360 Hz constant force |

### Phase 3 — Feel features (what separates "works" from "polished")
1. Base feel layer: configurable device-side damper/friction/inertia mixed under game
   FFB (each defaulting to a small nonzero value).
2. Hands-off oscillation detection → adaptive damping (C6).
3. Optional torque "soft clip" (tanh-style knee near MAX_TORQUE_NM) instead of hard
   clamp — clipping curbs feels dramatically better.
4. Cogging/static-friction compensation experiment: tiny dither or minimum-torque
   breakaway boost at near-zero output (evaluate on hardware; motors vary).
5. Center calibration: hold-button-at-boot or config-tool command to set center, stored
   in NVS — replaces "always power on centered".

### Phase 4 — Configuration & tooling (ties into the WIP web app)
1. NVS-backed settings struct: wheel range, MAX_TORQUE_NM, endstop stiffness/damping,
   reconstruction level, slew limit, base damper/friction/inertia, direction invert,
   pedal calibration, TORQUE_TO_CURRENT.
2. Vendor-defined HID feature report (or a second vendor usage page collection) for
   get/set of that struct → works from the web app via **WebHID** with no driver.
   Include: live telemetry (angle, torque, VESC temp/current when telemetry poll is
   enabled), pedal raw values for calibration UI, "set center", "set range".
3. Wheel-range control per game: expose range set via the config channel; optionally
   auto-honor spring-based soft lock (games like AC set range through their own wheel
   drivers — a per-game profile in the web app is the practical answer).
4. Pedal calibration flow in the web app (min/max capture, brake load-cell max force),
   plus firmware-side EMA filtering (D6).

### Phase 5 — Pedals & inputs hardening (when hardware exists)
1. HX711 RATE pin high (80 SPS) or ADC upgrade — decide **before PCB** (D4).
2. Pedal filtering + deadzone ends (top/bottom 2%) so axes reliably hit 0 and max.
3. Button debounce (paddles are raw `digitalRead` in the report path today) — 5 ms
   debounce, and consider shift-paddle "blip" haptic (brief torque pulse on shift, off
   by default).
4. Brake load-cell: per-user max-force calibration, optional response curve
   (linear/progressive) in config.

---

## Suggested implementation notes

- **Torque pipeline (target architecture, `ffb_task` @1 kHz):**
  `blocks → per-block force (direction, envelope, condition model) → sum → base-feel
  layer → soft clip @ MAX_TORQUE_NM → slew limit → endstop overlay (has its own cap) →
  actuators/safety gate → gearing → A/Nm → VESC`
- Keep everything before the gate in **rim Nm**; convert to motor amps only at the end.
- Springs/dampers/friction/inertia run raw at 1 kHz (never smoothed); only *host-supplied
  magnitudes* (constant/ramp/periodic) go through the reconstruction filter.
- Build a tiny host-side replay harness early: capture real game traffic with
  `ffbwrap --logger` (ffbtools) once, replay it over `hidraw` in CI-on-desk to regression-
  test parsing without launching games.

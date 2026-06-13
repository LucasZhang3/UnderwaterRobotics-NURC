# PID Depth Hold (BOT2 Only)

Depth hold runs **entirely on the top-side** (`Bot2Top.ino`). The bottom side only provides filtered pressure voltage in telemetry channel 5. BOT1 has **no** equivalent closed-loop depth control.

## Architecture

```
Pressure ADC (bottom) → V packet → parse_reply_msg()
    → depthFeetFromPressureVolts() → EMA filters
    → updateDepthHoldAssist() → analogs[RJoyY]
    → translate_controls_to_commands() → vertical thrusters
```

PID output is injected as the right-stick vertical analog **before** thruster mixing.

## Activation Logic

Depth hold is **disabled** when any of the following is true:

- Gamepad not connected (`joystickConnected()` false)
- Telemetry invalid or out of range (`depthTelemetrySampleValid`)
- No valid telemetry for **> 1000 ms** (`TELEMETRY_TIMEOUT_MS`)
- Right stick magnitude **> 5%** (`VERTICAL_DEADBAND = 0.05`)

When the operator releases the vertical stick:

1. `verticalStickReleased` flag set; PID state reset.
2. **750 ms delay** (`DEPTH_HOLD_ACTIVATION_DELAY_MS`) with vertical stick slewed toward zero.
3. Target depth captured: `targetDepthFeet = filteredDepthFeet`.
4. `depthHoldEnabled = true`; LCD PID line updated.

Manual stick input immediately calls `disableDepthHold()`.

## Depth Estimation

```c
depthFeet = (pressureVolts - DEPTH_SURFACE_VOLT) * DEPTH_FEET_PER_VOLT
```

Defaults: `DEPTH_SURFACE_VOLT = 0.0`, `DEPTH_FEET_PER_VOLT = 90.0`.

### Filtering

| Filter | Alpha | Applied to |
|--------|-------|------------|
| `DEPTH_VOLT_FILTER_ALPHA` | 0.12 | Raw pressure voltage |
| `DEPTH_FILTER_ALPHA` | 0.07 | Depth in feet |
| `DEPTH_RATE_FILTER_ALPHA` | 0.25 | Depth rate (D-term) |

### Validation

Sample rejected if:

- Pressure voltage < 0.05 V or > 3.25 V
- Depth < -2 ft or > 250 ft

Invalid sample clears `depthTelemetryValid` and disables hold.

## PID Calculation

```
error = targetDepthFeet - filteredDepthFeet
errorForPid = applyDepthErrorDeadband(error)
```

Deadband: ±0.35 ft hard zero; soft blend over additional 0.20 ft (`DEPTH_HOLD_ERROR_BLEND`).

```
depthRate = d(filteredDepthFeet)/dt  (zeroed if |rate| < 0.08 ft/s)
integral += errorForPid × dt  (clamped ±1.0, only when |error| > deadband)
output = Kp×errorForPid + Ki×integral - Kd×filteredDepthRate
output ×= DEPTH_PID_OUTPUT_SIGN (-1.0)
output = clamp(±0.18)  (MAX_VERTICAL_PID_OUTPUT)
```

Gains: **Kp = 0.20**, **Ki = 0.005**, **Kd = 0.14**.

Anti-windup: integral reduced when output saturates in direction of error.

Output slew: `depthHoldOutput` tracks PID via `limitRateOfChange` at **1.0 /s** (`DEPTH_PID_SLEW_RATE`).

Final: `analogs[RJoyY] = depthHoldOutput`.

## Display

LCD row 3:

```
PID: ±X.XX ft ON/OFF
```

Shows **error** (target − filtered), not absolute depth. Depth row uses `filteredDepthFeet - DEPTH_DISPLAY_OFFSET_FT` (42.7 ft offset for surface zero).

## Safety Behavior

| Event | Response |
|-------|----------|
| Stick deflection | Immediate disable + PID reset |
| Bad sensor sample | Disable hold |
| Telemetry timeout | Disable hold |
| Gamepad disconnect | Disable hold |
| Slow mode active | Reduced accel/decel during pre-hold slew only; PID gains unchanged |

## Manual Override

Any right-stick input beyond vertical deadband bypasses PID until stick is released and the 750 ms activation timer completes again.

## Tuning Parameters

Calibrate at pool:

1. `DEPTH_SURFACE_VOLT`: pressure voltage at surface
2. `DEPTH_FEET_PER_VOLT`: slope at known depth
3. `DEPTH_DISPLAY_OFFSET_FT`: LCD zero at surface
4. `DEPTH_PID_OUTPUT_SIGN`: flip if hold drives wrong direction

## Relation to BOT1

BOT1 displays depth from the same telemetry scaling (`telScale[5] = 90`) but does not close the loop. Operator must maintain depth manually.

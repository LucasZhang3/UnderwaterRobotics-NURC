# BOT1 vs BOT2 Comparison

BOT1 and BOT2 are separate NURC ROV platforms derived from the ROVotron Cadet design. Flash **Bot1Top + Bot1Bottom** or **Bot2Top + Bot2Bottom**. Never cross pairs.

## Summary

| Feature | BOT1 | BOT2 |
|---------|------|------|
| Generation | Original platform | Second-generation platform |
| Depth hold | Manual (right stick) | Automatic PID after stick release |
| Slow mode | No | Yes (LB + RB toggle) |
| Input slew limiting | No | Yes (separate accel/decel rates) |
| Gripper mechanism | Dual PWM outputs (pins 9/10) | 5 V RC servo (pin 21) with slew |
| Strafe direction sign | `StrafeMotDir = +1` | `StrafeMotDir = -1` |
| Camera tilt | Servo on pin 21 (BOT1 bottom) | Pin 17 reserved; disabled (`CAMERA_TILT_ENABLED 0`) |
| Checksum on bottom | Computed but not verified | Verified; bad packets rejected |
| `S` switch section | Top sends; bottom parser skips | Parsed on bottom |
| LCD extras | Battery, depth, temps | Slow mode, PID error line |
| Pitch coupling | None | Drive pitch trim on vertical thrusters |

## Hardware Differences

### Thrusters and ESCs

Both use five BlueRobotics-style ESC channels (motors 1-5) plus a gripper channel (motor 0). Pin assignments for thrusters are identical (pins 3-7). BOT1 drives the gripper with complementary `analogWrite` on pins 9 and 10. BOT2 uses a dedicated claw servo on pin 21 with constrained pulse width (1000-2000 µs) and maximum 80 µs step per 20 ms.

### Sensors (Bottom Side)

Both sample six 12-bit ADC channels:

| Index | Signal | Pin |
|-------|--------|-----|
| 0 | Battery | A9 |
| 1 | Auxiliary / H2O temp path | A3 |
| 2 | Unused / spare | A2 |
| 3 | LED temperature | A1 |
| 4 | H2O temperature | A0 |
| 5 | Depth (pressure transducer) | A6 |

I2C (pins 18/19) is reserved; MS5837 code exists in revision history but is not active in the current sketches.

### Lighting and Camera

- **BOT1**: LED dim via `analogWrite` on pin 22; camera tilt via `Servo` on pin 21.
- **BOT2**: LED dim on pin 22; camera tilt output on pin 17 (PWM or servo selectable) but **disabled in firmware** on both top and bottom.

## Software Differences

### Top Side: Control Mixing

Both share the same stick layout:

- Left X → strafe
- Left Y → forward/aft (with steer on right X)
- Right Y → vertical (BOT1 direct; BOT2 PID-assisted)
- Triggers → gripper

BOT2 adds:

- **Slow mode**: 40% stick scale, reduced accel/decel rates.
- **Depth hold**: Captures target depth 750 ms after right-stick vertical returns to neutral; PID drives vertical thrusters.
- **Gripper**: Latched position incremented by triggers (`GRIPPER_STEP`), not instantaneous trigger differential.
- **Drive pitch trim**: `-0.12 × forward command` added to both vertical thrusters while driving.

### Top Side: Telemetry Display

| LCD Row | BOT1 | BOT2 |
|---------|------|------|
| 0 | Battery | Slow mode ON/OFF |
| 1 | Depth | Battery |
| 2 | LED temp | Depth (offset-calibrated) |
| 3 | H2O temp | PID error + hold status |

BOT2 applies `DEPTH_DISPLAY_OFFSET_FT` (42.7 ft) so the LCD reads ~0 at the surface.

### Bottom Side: Protocol Handling

**BOT1** `parse_command_msg` reads `M` (motors) and `P` (servos), then expects `C` (checksum). It does **not** consume the `S` (switches) field that the top side transmits. This is a protocol mismatch in the repository snapshot. See [Open Questions](#open-questions).

**BOT2** reads `M`, `P`, `S`, and `C`, and rejects packets when `(checksum % 256) != received checksum`.

### Bottom Side: Actuator Mapping

| Channel | BOT1 | BOT2 |
|---------|------|------|
| motors[0] | Dual PWM gripper logic | Claw servo via `applyClawOutput()` |
| servos[0] | LED dim (`DIM_PIN`) | LED dim |
| servos[1] | Camera servo | Camera (disabled) |

## Behavior Differences

### Depth Handling

- **BOT1**: Operator controls depth entirely with the right stick. Depth telemetry is displayed but not used for closed-loop control.
- **BOT2**: Operator may override depth manually at any time (stick beyond 5% deadband disables hold). When released, PID maintains captured depth with filtered pressure input, integral anti-windup, and slew-limited output.

### Safety Systems

- **BOT2 top** disables depth hold when: gamepad disconnects, telemetry invalid/out of range, telemetry timeout (>1 s), or vertical stick active.
- **BOT2 bottom** rejects malformed packets; claw motion is rate-limited to reduce brownout/twitch.
- **BOT1 bottom** has commented-out checksum verification and no communication timeout stop in the main loop despite `stop_all_motors()` existing.

## Shared Elements

- RS-485 at 115200 baud, `Serial1.transmitterEnable` on pin 2
- 20 Hz command cadence from top
- Hex ASCII packet format (see [communication.md](communication.md))
- Xbox One gamepad via Teensy USB Host
- Six telemetry ADC fields in reply packets
- ESC startup: 7 s neutral, 1 s forward pulse, return neutral

## Open Questions

1. **BOT1 `S` section mismatch**: Bot1Top transmits `S` switch nibbles before `C`; Bot1Bottom expects `C` immediately after `P`. Verify deployed BOT1 firmware pairs or intended protocol revision.
2. **BOT1 checksum**: Validation is commented out (`// if (checksum != (unsigned char) val) return 1;`).
3. **Depth sensor calibration**: BOT2 uses tunable `DEPTH_SURFACE_VOLT`, `DEPTH_FEET_PER_VOLT`, and display offset; field calibration values may differ from competition pool conditions.

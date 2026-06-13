# Build Notes

Engineering notes on the NURC ROV platforms, design evolution, and competition context. Summarized from firmware structure and external NURC references, not verbatim copies of source documents.

## Design Evolution

### ROVotron Cadet Lineage

Firmware copyright and revision history trace to **David Forbes** ROVotron Cadet programs (2010, 2023-2024). The stack uses:

- Teensy 4.x top-side USB host + LCD
- Teensy 4.0 bottom-side ESC/sensor controller
- RS-485 hex ASCII protocol inherited from earlier RVCBOT revisions (A through D)

Internal revision comments document hardware spins: RVCBOT-A/B/C pin changes, motor count reductions, pressure telemetry addition (2024-02-22).

### BOT1: Original Platform

- Established six-motor mixing with Xbox One gamepad.
- Dual-PWM gripper on dedicated pins.
- Camera servo on pin 21; LED dim on pin 22.
- Manual depth control; pressure telemetered for display only.
- Legacy gain trims (asymmetric drive L/R) tuned for straight tracking and current draw.

### BOT2: Second Generation

Developed for improved operator assist and competition reliability:

- **Top-side PID depth hold**: closed-loop vertical control using pressure telemetry.
- **Slow mode**: LB+RB toggle for fine maneuvering in mission tasks.
- **Slew rate limiting**: reduces jerky stick inputs and ESC current spikes.
- **RC claw servo**: replaces dual PWM gripper; rate-limited to prevent brownouts.
- **Checksum enforcement** on bottom; full `S` section parsing.
- **Strafe direction** reversed in software (`StrafeMotDir -1`) for updated mechanical mounting.
- Camera tilt **disabled** in firmware (`CAMERA_TILT_ENABLED 0`) while hardware migrated to pin 17.

## Construction Overview

Typical ROVotron-style build (inferred from firmware + NURC practice):

| Subsystem | Notes |
|-----------|-------|
| Frame | Competition-class PVC or modular frame; mission props require payload manipulation |
| Propulsion | Five BlueRobotics-compatible ESCs + horizontal/strafe/vertical mix |
| Control electronics | Teensy 4.x top box; Teensy 4.0 bottom enclosure |
| Sensors | 12-bit ADC: battery, temperature dividers, analog pressure transducer |
| Camera | Top-side tilt command (BOT1 active; BOT2 disabled pending hardware) |
| Lighting | PWM dimmable LED on pin 22 |
| Tether | RJ45 carrying RS-485; separate power conductors |
| Manipulator | BOT1: dual PWM grip; BOT2: 5 V RC servo claw |

Team photos, prototype renders, claw build images, and pool test video are in [`assets/`](../assets/). The RVCBOT-D control board schematic is at [`assets/Gripper Photos/RVCBOT-D-schem.pdf`](../assets/Gripper%20Photos/RVCBOT-D-schem.pdf).

## Engineering Decisions

### Top-Side Depth PID

Placing PID on the top reduces bottom firmware complexity and allows tuning without reflashing the wet electronics. Trade-off: depth control depends on tether latency (~50 ms command + reply); filtering and conservative gains (`MAX_VERTICAL_PID_OUTPUT = 0.18`) mitigate oscillation.

### ESC Startup Delay

Seven-second neutral wait follows **BlueRobotics ESC calibration** guidance referenced in bottom firmware comments. Skipping this risks ESCs not arming correctly.

### 100 MHz CPU on Top

LCD parallel interface requires slower Teensy clock (`set_arm_clock(100 MHz)` when default is at least 100 MHz).

### Checksum Policy Split

BOT2 enforces integrity on critical wet-side parsing. BOT1 retains commented-out check, likely legacy debugging; operators should treat BOT2 as the reference for secure comms.

## Lessons Learned

1. **Gripper**: Direct trigger-to-PWM caused plastic stress; BOT2 latched + slew claw addresses this.
2. **Depth sensor**: MS5837 I2C code was attempted then disabled ("not behaving in top end"); analog pressure on A6 used instead.
3. **Protocol alignment**: Ensure top and bottom parsers match (`S` section). Audit before competition.
4. **Display calibration**: Pool depth zero requires `DEPTH_DISPLAY_OFFSET_FT` and voltage slope tuning on BOT2.
5. **Camera**: Disable unused tilt output when hardware unplugged to avoid floating pins.

## Competition Considerations

### NURC Context

The [National Underwater Robotics Challenge (NURC)](https://www.nurc.us/) is a student underwater robotics competition emphasizing mission tasks, engineering documentation, and practical ROV operation. Resources relevant to this project:

| Resource | Description |
|----------|-------------|
| [Mission props](https://www.nurc.us/misson-props) | Competition task objects and layout |
| [Webcast](https://www.nurc.us/webcast) | Event broadcasts and briefings |
| [Technical PDFs](https://www.nurc.us/) | Rules and engineering guidelines (see README external links) |
| [YouTube: NURC overview](https://www.youtube.com/watch?v=rPA1AVYUPB4) | Competition footage |
| [YouTube: extended session](https://www.youtube.com/watch?v=SjvaYA--C2E&t=2877s) | Long-form event video |

Mission style typically combines **precision piloting**, **manipulation**, and **sensor awareness**, matching BOT2 slow mode, depth hold, and telemetry display investments.

### Operational Checklist

- [ ] Flash matched BOT1 or BOT2 pair
- [ ] Calibrate depth at pool (BOT2)
- [ ] Verify gamepad VID:PID on LCD
- [ ] Run ESC arming dry
- [ ] Confirm gripper/claw direction
- [ ] Test tether comm before submersion
- [ ] Battery voltage alarm threshold understood (~11 V display scale)

## Future Work (From Legacy Comments)

- Non-blocking RS-485 receive on top (`receive_msg` marked for rewrite)
- Improved comm error handling and timeout testing on bottom
- Temperature sensor calibration
- Re-enable I2C pressure if analog transducer insufficient

## Team Documentation

Detailed architecture: [architecture.md](architecture.md). Platform comparison: [bot1-vs-bot2.md](bot1-vs-bot2.md).

# Hardware Mapping

Pin assignments for Teensy 4.x boards in the NURC ROV stacks. BOT1 and BOT2 bottom boards share thruster pins; gripper and camera routing diverge.

## Top Side (Bot1Top / Bot2Top)

| Signal | Pin | Notes |
|--------|-----|-------|
| LCD RS | 5 | 4-bit parallel LCD |
| LCD RW | 6 | Read/write (LiquidCrystalFast) |
| LCD E | 7 | Enable |
| LCD D4-D7 | 24-27 | Data; 1 kΩ series for 3.3 V |
| RS-485 TX enable | 2 | `Serial1.transmitterEnable` |
| RS-485 UART | Serial1 (0/1) | 115200 baud |
| USB debug | Serial (USB) | 115200 baud |
| USB Host | USB OTG | Xbox One gamepad |

CPU clock forced to **100 MHz** when default ≥ 100 MHz (LCD timing).

## Bottom Side: Shared Thruster Mapping

| Motor index | Function | Pin | Output type |
|-------------|----------|-----|-------------|
| 1 | Left vertical | 3 | Servo library → ESC |
| 2 | Right vertical | 4 | Servo library → ESC |
| 3 | Left horizontal | 5 | Servo library → ESC |
| 4 | Right horizontal | 6 | Servo library → ESC |
| 5 | Strafe | 7 | Servo library → ESC |

ESC pulse mapping (`motor_scale`):

| Command byte | Pulse width |
|--------------|-------------|
| 0 | 1900 µs |
| 128 | 1500 µs (neutral) |
| 255 | 1100 µs |

Startup sequence: 7 s @ 1500 µs → 1 s @ 1600 µs → 1500 µs.

## Bottom Side: BOT1 Actuators

| Function | Pin | Control |
|----------|-----|---------|
| Gripper outward | 9 | `analogWrite` (complementary PWM) |
| Gripper inward | 10 | `analogWrite` |
| Camera tilt | 21 | `Servo` (`servo1`) |
| LED dim | 22 | `analogWrite` (8-bit duty) |

Gripper logic derives from `motors[0] - 128`: negative → grip2 active; positive → grip1 active.

## Bottom Side: BOT2 Actuators

| Function | Pin | Control |
|----------|-----|---------|
| Camera tilt | 17 | PWM @ 50 Hz or Servo (disabled) |
| Claw servo | 21 | `Servo` 1000-2000 µs, slew limited |
| LED dim | 22 | `analogWrite` |

Claw update: max **80 µs** change every **20 ms** (`CLAW_MAX_US_STEP`, `CLAW_UPDATE_MS`).

## Bottom Side: Analog Inputs

| Channel | Pin | Signal |
|---------|-----|--------|
| volts[0] | A9 | Battery voltage divider |
| volts[1] | A3 | Auxiliary / H2O temp |
| volts[2] | A2 | Spare |
| volts[3] | A1 | LED temperature |
| volts[4] | A0 | H2O temperature |
| volts[5] | A6 | Pressure transducer (depth) |

ADC: **12-bit** resolution; software smoothing `smooth=80/100`.

## Bottom Side: Bus Pins (Reserved)

| Bus | SDA | SCL | MOSI | MISO | SCK |
|-----|-----|-----|------|------|-----|
| I2C | 18 | 19 | n/a |: | n/a |
| SPI | n/a |: | 11 | 12 | 13 |

## Top Side: Logical Motor Indices

Defined identically in both top sketches:

```c
#define GripperMot 0
#define LUpDownMot 1
#define RUpDownMot 2
#define LForAftMot 3
#define RForAftMot 4
#define StrafeMot  5
```

## Servo Scaling (Bottom)

Standard servo (`servo_scale`): 700-2300 µs span centered at 1500 µs for command 128.

Claw (`claw_servo_scale`): clamped to 1000-2000 µs.

## Power and Tether

- Tether: RJ45, RS-485 on one pair (per ROVotron Cadet design comments)
- Battery monitoring on bottom; top displays computed pack voltage (×11 scale on channel 0)

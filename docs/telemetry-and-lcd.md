# Telemetry and LCD

Telemetry travels from bottom-side ADC sampling to top-side LCD display through the `V…` reply packet. Processing differs between BOT1 and BOT2 on the top side.

## Telemetry Pipeline

```
analogRead (bottom, 12-bit)
    → exponential smooth (80/100 blend)
    → volts[] raw counts
    → build_reply_msg() → 6 × 3 hex digits
    → RS-485 to top
    → parse_reply_msg() → float volts[6]
    → telZeros/telScale → engineering units
    → LCD
```

## Reply Packet Encoding

Each channel encoded as three uppercase hex digits (0-FFF = 0-4095 counts).

Voltage at top: `volts[i] = hex_val × 3.3 / 4096`.

## Channel Definitions

| Index | Bottom source | Display label | Conversion (top) |
|-------|---------------|---------------|------------------|
| 0 | `VBATT_PIN` (A9) | Battery | `(V - 0) × 11.0` → volts |
| 1 | `ANALOG1_PIN` (A3) | H2O temp (BOT1) | `(V - 0) × (256/3.3)` → °C |
| 2 | `ANALOG2_PIN` (A2) | (unused) | `(V - 0.10) × 8` |
| 3 | `ANALOG3_PIN` (A1) | LED temp | `(V - 1.40) × 70` → °C |
| 4 | `ANALOG4_PIN` (A0) | (unused) | `(V - 1.40) × 71` |
| 5 | `PRESSURE_PIN` (A6) | Depth | `(V - 0) × 90` → feet (display) |

Constants `telZeros[]` and `telScale[]` are identical on both top sketches.

## Bottom-Side Filtering

```c
volts[i] = (volts[i]*smooth + analogRead(pin)*(smoothRange-smooth)) / smoothRange;
```

With `smooth=80`, `smoothRange=100`: ~80% prior, ~20% new sample, which reduces tether noise without adding top-side latency.

## BOT2 Depth Processing (Top)

Beyond simple scaling, BOT2 applies:

1. **Validity check** on pressure voltage and computed feet.
2. **EMA** on pressure voltage (α = 0.12).
3. **EMA** on depth feet (α = 0.07).
4. **Rate filter** for PID D-term (α = 0.25).
5. **Display offset**: `depthDisplay = filteredDepthFeet - 42.7`.

Depth hold uses `filteredDepthFeet`, not the raw display value.

## LCD Layout

Hardware: **4 rows × 20 columns**, `LiquidCrystalFast`, parallel 4-bit interface.

### BOT1 (`display_all_telems`)

| Row | Content |
|-----|---------|
| 0 | `Battery  XX.X V` |
| 1 | `Depth    XX.XX Feet` |
| 2 | `LED temp XX.X C` |
| 3 | `H2O temp XX.X C` |

### BOT2 (`display_all_telems`)

| Row | Content |
|-----|---------|
| 0 | `Slow Mode: ON/OFF` |
| 1 | `Battery  XX.X V` |
| 2 | `Depth    XX.XX Feet` (offset-calibrated) |
| 3 | `PID: ±X.XX ft ON/OFF` (error + hold state) |

Rows pad with spaces after sprintf to clear stale characters.

### Startup / Status Screens

Both show splash:

```
   ROVotron Cadet
 ROV Control System
     Rev. 1.21
```

Then `Seeking Gamepad` with USB VID:PID on connect (`PrintDeviceListChanges`).

BOT2 additionally initializes row 2 to `Slow Mode:OFF`.

## Update Behavior

- **BOT1**: LCD refreshed when a complete telemetry line is received (same loop pass as parse).
- **BOT2**: Telemetry parsed **before** command transmit in `loop()`; LCD updated on each valid reply.
- Command rate: 20 Hz regardless of telemetry arrival rate (telemetry typically follows each bottom reply).

## Switch Telemetry Flags

Top encodes Y and B buttons in the `S` section (`switches[0]`, `switches[1]`). Bottom BOT2 parses but does not act on them in current code. They are available for future bottom-side features.

## Debug Serial

`TOP_DEBUG_SERIAL` and `BOTTOM_DEBUG_SERIAL` default to **0**. When enabled, depth and packet debug print to USB Serial.

## Known Limitations

- Telemetry has **no checksum**. Corrupted lines may produce brief bogus readings; BOT2 validates depth range.
- Temperature scaling comments note calibration is incomplete ("Calibrate temp sensors" in legacy notes).
- BOT1 H2O and LED temp channels depend on analog front-end wiring matching `telZeros`/`telScale`.

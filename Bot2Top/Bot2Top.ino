/* RVTOP-Ccode
 * ROVotron Cadet topside control box — main firmware
 *
 * (C) 2010, 2023, 2024 David Forbes
 *
 * Overview
 * --------
 * The topside box drives an ROV over RS-485, reads an Xbox One gamepad via USB,
 * and shows live telemetry on a 4×20 character LCD. Motor and servo mapping is
 * fixed in compile-time tables and gain trims.
 *
 * Hardware
 * --------
 * - Teensy host MCU with USB host port for gamepad
 * - 4×20 LCD on parallel data lines (3.3 V via series resistors)
 * - RJ45 tether carrying RS-485 on one pair
 *
 * Control loop (~20 Hz)
 * ---------------------
 * 1. Poll USB and ingest ROV reply telegrams
 * 2. Filter depth samples and refresh the LCD
 * 3. Map gamepad axes to motion commands (slow mode, slew limits, depth hold)
 * 4. Pack and transmit hex-ASCII motor/servo/switch command frame
 *
 * Depth hold, slow mode, slew limiting, gripper latch, and telemetry validity
 * gates are documented on the functions that implement them.
 *
 * Revision history
 * ----------------
 * 2023-03-26 DF  Forked from rtxa.c; Teensy + Xbox One
 * 2023-09-09 DF  Removed runtime config code
 * 2023-09-10 DF  Direction and trim corrections
 * 2023-09-11 DF  Vector drive made optional (later removed)
 * 2023-11-25 DF  Vector drive removed; fewer motors/servos
 * 2024-02-11 DF  Motor index aligned with build instructions
 * 2024-02-22 DF  telems[5] pressure channel
 * 2024-02-25 DF  Depth scale factor and units updated
 * 2024-05-31 DF  USB declarations for TeensyDuino 1.59
 * 2024-06-02 DF  v1.20 — USB ID on LCD at connect
 * 2024-06-09 DF  v1.21 — Y button temp readout enable
 *
 * Protocol reference: RVdataDescr.txt
 *
 * Terminology
 * -----------
 * Buttons  — 14 face/trigger/bumper inputs on the gamepad
 * Analogs  — ten normalized axes (−1..+1), including synthesized D-pad and face-pad
 * Motors   — ROV thrusters 0..5 (0 = gripper)
 * Servos   — PWM outputs 0..2 (0 = LED dimmer)
 * volts[]  — raw 12-bit ADC readings from the ROV (index 0 = battery)
 * telems[] — user-unit values derived from volts[]
 *
 * Motor modes (per-channel config elsewhere)
 * 0 OFF   — channel disabled
 * 1 SPEED — analog passed through
 * 2 RATE  — analog integrated for position/speed; gain sets ramp rate at full stick
 */

// Teensy I/O libraries
#include <LiquidCrystalFast.h>
#include <math.h>
#include "USBHost_t36.h"

// ------------------ Teensy pin definitions ----------------------- //

// LCD parallel data (D4–D7); 1 kΩ series resistors for 3.3 V compatibility
#define LCD_RS 5
#define LCD_RW 6
#define LCD_E  7
#define LCD_D4 24
#define LCD_D5 25
#define LCD_D6 26
#define LCD_D7 27

// ROV link on Serial1
#define BAUD_RATE 115200

// RS-485 driver transmit-enable
#define SER_TXEN 2
#define TOP_DEBUG_SERIAL 0

// CPU clock override (see setup)
extern "C" uint32_t set_arm_clock(uint32_t frequency); // required prototype

// ------------ Sizes of things --------------------------- //

// Command message channel counts
#define NMOTORS 6
#define NSERVOS 2
#define NSWITCHES 2
// Reply message ADC channel count
#define NVOLTS 6
// Hex digit widths in wire format
#define MOTOR_DIGITS 2
#define SERVO_DIGITS 2
#define VOLTS_DIGITS 3

// Neutral PWM center value sent to ESC/servo outputs
#define Pwm0  128



#define SLOW_MODE_SCALE 0.4
#define ACCEL_RATE_NORMAL 2.5
#define DECEL_RATE_NORMAL 6.0
#define ACCEL_RATE_SLOW 1.0
#define DECEL_RATE_SLOW 3.0
// --- Control tuning constants (see function docs for behavior) ---
#define DEADBAND 3000              // Stick raw units (~9% travel) before axis responds
#define VERTICAL_DEADBAND 0.05f    // Right-stick Y above this cancels depth hold
#define DEPTH_HOLD_DEADBAND 0.35f  // PID treats |error| below this as zero (feet)
#define DEPTH_HOLD_ERROR_BLEND 0.20f // Soft ramp above deadband (feet) to avoid step response
#define DEPTH_RATE_DEADBAND 0.08f  // D-term ignores depth rate below this (ft/s)
#define DEPTH_HOLD_ACTIVATION_DELAY_MS 750
#define MAX_VERTICAL_PID_OUTPUT 0.18f
#define DEPTH_KP 0.20f
#define DEPTH_KI 0.005f
#define DEPTH_KD 0.14f
#define DEPTH_VOLT_FILTER_ALPHA 0.12f  // EMA on pressure voltage before feet conversion
#define DEPTH_FILTER_ALPHA 0.07f       // EMA on depth in feet after conversion
#define DEPTH_RATE_FILTER_ALPHA 0.25f  // EMA on depth rate for D-term
#define DEPTH_PID_SLEW_RATE 1.0f       // Max depth-hold vertical output change per second
#define TELEMETRY_TIMEOUT_MS 1000
// Depth sensor: feet = (pressureVolts - DEPTH_SURFACE_VOLT) * DEPTH_FEET_PER_VOLT
#define DEPTH_SURFACE_VOLT 0.0f
#define DEPTH_FEET_PER_VOLT 90.0f
#define DEPTH_DISPLAY_OFFSET_FT 42.7f   // LCD depth zero at surface
#define DEPTH_PID_OUTPUT_SIGN (-1.0f)
#define DEPTH_INTEGRAL_MAX 1.0f   // Anti-windup clamp on depth-hold integral
#define DPAD_AXIS_STEP 0.03f
#define CAMERA_DPAD_STEP 0.12f
#define CAMERA_DPAD_SNAP 1
#define DEPTH_PRESSURE_V_MIN 0.05f
#define DEPTH_PRESSURE_V_MAX 3.25f
#define DEPTH_FEET_MIN -2.0f
#define DEPTH_FEET_MAX 250.0f
#define CAMERA_TILT_ENABLED 0
// ----------------- Motor configuration ------------------ //

// Motor indices per build instructions
#define GripperMot 0
#define LUpDownMot 1
#define RUpDownMot 2
#define LForAftMot 3
#define RForAftMot 4
#define StrafeMot  5

// Per-motor sign multiplier (flip here if a thruster runs backward)
float motDirs[NMOTORS];

#define GripperMotDir  1. 
#define LUpDownMotDir  1.
#define RUpDownMotDir -1.
#define LForAftMotDir -1.
#define RForAftMotDir  1.
#define StrafeMotDir  (-1.)

// Drive/steer/dive gain trims — tune for straight travel and feel
float DriveGainL  = 0.60;
float DriveGainR  = 0.60;
float SteerGain   = 0.60;
float DiveGainL   = 0.60;
float DiveGainR   = 0.60;
float StrafeGain  = 0.58;
#define DRIVE_PITCH_TRIM (-0.12f)
#define GRIPPER_TRIGGER_ON 0.18f   // Trigger dead zone before gripper moves
#define GRIPPER_STEP 0.07f         // Gripper position delta per loop at full trigger (~20 Hz)

// LCD instance (RW pin enables readback of display RAM)
LiquidCrystalFast lcd(LCD_RS, LCD_RW, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// ------------ USB joystick object ----------------------- // 

USBHost myusb;
USBHub hub1(myusb);
USBHIDParser hid1(myusb);

#define COUNT_JOYSTICKS 4
JoystickController joysticks[COUNT_JOYSTICKS] = {
  JoystickController(myusb), JoystickController(myusb),
  JoystickController(myusb), JoystickController(myusb)
};
int user_axis[64];
uint32_t buttons_prev = 0;

// Buffer for USB product string / device ID on LCD
char namebuf[100] = { 0 };


// Hub supports up to four gamepads; ROVotron typically uses one
USBDriver *drivers[] = {&hub1, &joysticks[0], &joysticks[1], &joysticks[2], &joysticks[3], &hid1};
#define CNT_DEVICES (sizeof(drivers)/sizeof(drivers[0]))
const char * driver_names[CNT_DEVICES] = {"Hub1", "joystick[0D]", "joystick[1D]", "joystick[2D]", "joystick[3D]",  "HID1"};
bool driver_active[CNT_DEVICES] = {false, false, false, false};


// HID-layer connect tracking for joysticks
USBHIDInput *hiddrivers[] = {&joysticks[0], &joysticks[1], &joysticks[2], &joysticks[3]};
#define CNT_HIDDEVICES (sizeof(hiddrivers)/sizeof(hiddrivers[0]))
const char * hid_driver_names[CNT_DEVICES] = {"joystick[0H]", "joystick[1H]", "joystick[2H]", "joystick[3H]"};
bool hid_driver_active[CNT_DEVICES] = {false};

//------------ Global Variables ------------------------------- //

// Main loop period (microseconds); 50000 µs → 20 Hz command rate
unsigned long loopPeriod = 50000;  // in microseconds: 20 loops per second
unsigned long nextLoopMicros = 0;

unsigned long lastSlewMicros = 0;
char inString[100];          // Legacy receive buffer (unused by current loop path)
char inChr = 0;              // Legacy single-char RX staging
char * inPtr;
bool gotInString = false;    // True when a complete newline-terminated reply is ready

// ------------ XBox gamepad control mapping ----------------- //

// Raw gamepad → buttons[] / axes[] → normalized analogs[] → motors[] / servos[]

#define NANALOG 10
#define NBUTTONS 16

// Raw gamepad state
uint32_t butts;    // the single word that contains all button bits 
unsigned char buttons[NBUTTONS];  // button control values: 1 = pressed, 0 = not
int axes[6];     // the value of each real analog control axis from gamepad

// Logical button indices (bit order in butts matches this list)
int PairButton = 1;
int MenuButton = 2;
int ViewButton = 3;
int AButton    = 4;
int BButton    = 5;
int XButton    = 6;
int YButton    = 7;
int DpadUp     = 8;
int DpadDown   = 9;
int DpadLeft   = 10;
int DpadRight  = 11;
int LButton    = 12;
int RButton    = 13;
int LJButton   = 14;
int RJButton   = 15;

// Normalized analog axis indices (−1..+1); triggers and synthesized pads included
int LJoyX = 0;
int LJoyY = 1;
int RJoyX = 2;
int RJoyY = 3;
int LTrig = 4;
int RTrig = 5;
int DPadX = 6;  // synthesized from dpad presses
int DPadY = 7;
int ButsX = 8;  // synthesized from button pad presses
int ButsY = 9;

float analogs[NANALOG];    // analog control values extracted from gamepad

float requestedAnalogs[NANALOG];
// Face buttons B/Y and X/A increment/decrement ButsX and ButsY; D-pad feeds DPadX/DPadY via readDpadDirections()
int up_button[NANALOG] = {0,0,0,0,0,0,0,0,  BButton,YButton};
int dn_button[NANALOG] = {0,0,0,0,0,0,0,0,XButton,AButton};

// Step size for virtual axes updated once per main-loop tick
float button_inc = DPAD_AXIS_STEP;

static int activeGamepadIndex = -1;
static float gripperCmd = 0.0f;
static float depthHoldOutput = 0.0f;

// Outbound ROV command arrays
int motors[NMOTORS];    // motor command values 0=full reverse, FF=full fwd
int servos[NSERVOS];    // servo command values 0=full CCW, FF=full CW
int switches[NSWITCHES];  // on/off switches based on buttons held down

bool slow_mode_enabled = false;
bool lbRbComboWasPressed = false;
bool depthHoldEnabled = false;
bool verticalStickReleased = false;
bool depthTelemetryValid = false;
unsigned long verticalReleaseMillis = 0;
unsigned long lastTelemetryMillis = 0;
float currentDepthFeet = 0.0;
float filteredDepthFeet = 0.0;
float targetDepthFeet = 0.0;
float previousDepthFeet = 0.0;
float filteredPressureVolts = 0.0;
float filteredDepthRate = 0.0;
float depthIntegral = 0.0;
float previousDepthError = 0.0;
float depthPidOutput = 0.0;
char command_msg[100];  // command message, newline, 0
char reply_msg[100];    // reply message, newline, 0
char buf[100];      // temporary string storage

// Inbound ROV telemetry
float volts[NVOLTS];      // telemetry voltages 0.0 to 3.3V (we receive 0..0xFFF)
float telems[NVOLTS];     // telemetry in its units

// --------------- Telemetry calculation ----------------- //

// Per-channel zero and scale for display telems[] (depth uses separate filter path)
float telZeros[NVOLTS] = {   0.00,  0.00,  0.10,   1.40,   1.40,  0.00};
float telScale[NVOLTS] = {  11.00,  256./3.3,  8.00,  70.00,  71.00, 90};

/*
 * Purpose: Convert raw pressure-sensor voltage to depth in feet.
 * Inputs:  pressureVolts — ADC-scaled voltage from volts[5] (typically 0..3.3 V).
 * Outputs: Depth in feet relative to DEPTH_SURFACE_VOLT and DEPTH_FEET_PER_VOLT.
 * System Impact: Feeds depth filtering, PID hold target/error, LCD depth line, and validity checks.
 * Safety Notes: Wrong calibration constants produce incorrect hold depth; tune at known depth.
 * Usage Notes: Linear model only; surface offset is DEPTH_SURFACE_VOLT.
 */
float depthFeetFromPressureVolts(float pressureVolts) {
  return (pressureVolts - DEPTH_SURFACE_VOLT) * DEPTH_FEET_PER_VOLT;
}

/*
 * Purpose: Soft deadband on depth-hold error to suppress PID chatter near the setpoint.
 * Inputs:  errorFeet — targetDepthFeet minus filteredDepthFeet (signed, feet).
 * Outputs: Zero inside DEPTH_HOLD_DEADBAND; full error beyond deadband+blend; linear ramp between.
 * System Impact: Reduces P/I/D activity and motor twitch when hovering close to target depth.
 * Safety Notes: Does not limit maximum error — large deviations still produce full corrective command.
 * Usage Notes: Blend width is DEPTH_HOLD_ERROR_BLEND; applied before integral accumulation.
 */
float applyDepthErrorDeadband(float errorFeet) {
  float magnitude = fabsf(errorFeet);
  if (magnitude <= DEPTH_HOLD_DEADBAND) return 0.0f;
  if (magnitude >= DEPTH_HOLD_DEADBAND + DEPTH_HOLD_ERROR_BLEND) return errorFeet;
  float blend = (magnitude - DEPTH_HOLD_DEADBAND) / DEPTH_HOLD_ERROR_BLEND;
  return (errorFeet >= 0.0f ? 1.0f : -1.0f) * blend * magnitude;
}

/*
 * Purpose: Refresh LCD row 0 with slow-mode ON/OFF status.
 * Inputs:  Global slow_mode_enabled.
 * Outputs:  Updates line 0 via lcd.print; uses shared buf[].
 * System Impact: Operator feedback for LB+RB toggle; also called when combo is pressed.
 * Safety Notes: Display only — does not change control gains by itself.
 * Usage Notes: Invoked from display_all_telems() and updateSlowModeToggle() on edge.
 */
void display_slow_mode_line(void) {
  sprintf(buf, "Slow Mode:%3s     ", slow_mode_enabled ? "ON" : "OFF");
  lcd.setCursor(0, 0);
  lcd.print(buf);
}

/*
 * Purpose: Refresh LCD row 3 with depth-hold error and enable state.
 * Inputs:  targetDepthFeet, filteredDepthFeet, depthHoldEnabled globals.
 * Outputs:  Shows signed error (ft) and ON/OFF on line 3; pads trailing spaces.
 * System Impact: Primary in-water indicator for PID depth hold engagement and tracking error.
 * Safety Notes: OFF may mean disengaged for safety (stick input, lost telem, etc.) — see updateDepthHoldAssist().
 * Usage Notes: Called after telemetry parse and whenever hold state changes.
 */
void display_pid_hold_line(void) {
  sprintf(buf, "PID:%+5.2f ft %3s", targetDepthFeet - filteredDepthFeet,
          depthHoldEnabled ? "ON" : "OFF");
  lcd.setCursor(0, 3);
  lcd.print(buf);
  lcd.print("   ");
}

/*
 * Purpose: Render all operator telemetry lines on the 4×20 LCD.
 * Inputs:  Global volts[], filteredDepthFeet, telZeros/telScale tables.
 * Outputs:  Row 0 slow mode; row 1 battery V; row 2 depth ft; row 3 PID hold line.
 * System Impact: Main human interface for power, depth, slow mode, and hold status each reply cycle.
 * Safety Notes: Depth display uses filtered value minus DEPTH_DISPLAY_OFFSET_FT, not raw ADC.
 * Usage Notes: Called once per successfully queued reply after parse_reply_msg().
 */
void display_all_telems(void) {
  float telems[NVOLTS];
  float depthDisplayFt;
  for (int i=0;i<NVOLTS;i++) {
    telems[i] = (volts[i] - telZeros[i]) * telScale[i];
  }
  depthDisplayFt = filteredDepthFeet - DEPTH_DISPLAY_OFFSET_FT;
  display_slow_mode_line();
  sprintf(buf, "Battery  %5.1f V", telems[0]);
  lcd.setCursor(0, 1);
  lcd.print(buf);
  lcd.print("   ");
  sprintf(buf, "Depth    %5.2f Feet", depthDisplayFt);
  lcd.setCursor(0, 2);
  lcd.print(buf);
  lcd.print("   ");
  display_pid_hold_line();
}

// ------------------- ROV message I/O ------------------ //

// Blocking receive helper — not used by the current non-blocking loop() RX path.

/*
 * Purpose: Blocking read of one null-terminated RS-485 message from Serial1.
 * Inputs:  msg — destination buffer; first_char — expected leading sync byte (e.g. 'V').
 * Outputs:  Returns 0 on success; 1 on start-byte or body timeout; message written to msg.
 * System Impact: Legacy/alternate I/O path; active loop uses interrupt-style char accumulation instead.
 * Safety Notes: Blocking delays starve USB and control loop if called from loop().
 * Usage Notes: Discards bytes until first_char matches, then reads through NUL with shorter timeout.
 */
int receive_msg(char *msg, char first_char) {
  int timer;

  // Hunt for sync byte, dropping anything else
  do {
    timer = 10000;        // 500 millisecond timeout
    while (!Serial1.available()) {
      delayMicroseconds(50);  
      if (!--timer) 
        return 1;
    }
  }
    while ((*msg = Serial1.read()) != first_char);
  msg++;  // save start char

  // Collect remainder of frame
  do {
    timer = 100;        // 5 millisecond timeout
    while (!Serial1.available()) {
      delayMicroseconds(50);  
      if (!--timer) 
        return 1;
    }
  }
    while ((*msg++ = Serial1.read()) != '\0');  // read the rest of message thru NUL
  return 0;
}

// -------------------- Parsing routines ---------------------- //

/*
 * Purpose: Decode one ASCII hex digit to a 0..15 nibble value.
 * Inputs:  chr — single character '0'-'9', 'A'-'F', or 'a'-'f'.
 * Outputs:  Nibble 0..15, or -1 if not hexadecimal.
 * System Impact: Building block for atox() and reply/command parsing.
 * Safety Notes: None — pure parse helper.
 * Usage Notes: Case-insensitive; returns -1 for any other character.
 */
char atoxdigit(char chr) {
  if (('a' <= chr) && (chr <= 'f')) return (chr - 'a' + 10);
  if (('A' <= chr) && (chr <= 'F')) return (chr - 'A' + 10);
  if (('0' <= chr) && (chr <= '9')) return (chr - '0');
  return -1;
}

/*
 * Purpose: Parse n consecutive hex digits from a string into an integer.
 * Inputs:  str — pointer to first digit; n — digit count to consume.
 * Outputs:  Non-negative integer value, or -4 if any digit is invalid.
 * System Impact: Used to unpack motor, servo, switch, checksum, and ADC fields from wire messages.
 * Safety Notes: No overflow guard — caller must keep n small (wire format uses 1–3 digits).
 * Usage Notes: Advances caller's pointer externally after call; does not skip separators.
 */
int atox(char *str, char n) {
  char digit, i;
  int val;
  val = 0;
  for (i = 0; i < n; i++) {
    if ((digit = atoxdigit(*str++)) == -1) return -4;
    val = (val << 4) + digit;
  }
  return val;
}

/*
 * Purpose: Reject out-of-range depth samples before they enter the filter or PID.
 * Inputs:  depthFeet — computed depth; pressureVolts — raw pressure channel voltage.
 * Outputs:  true if both values lie within DEPTH_*_MIN/MAX limits.
 * System Impact: Invalid samples clear depthTelemetryValid and force depth-hold disengage.
 * Safety Notes: Prevents hold using nonsensical sensor readings (open wire, saturation, etc.).
 * Usage Notes: Checked on every parse and again inside updateDepthHoldAssist() on filtered depth.
 */
bool depthTelemetrySampleValid(float depthFeet, float pressureVolts) {
  if (pressureVolts < DEPTH_PRESSURE_V_MIN || pressureVolts > DEPTH_PRESSURE_V_MAX) return false;
  if (depthFeet < DEPTH_FEET_MIN || depthFeet > DEPTH_FEET_MAX) return false;
  return true;
}

/*
 * Purpose: Parse ROV 'V' reply telegram into volts[] and update depth telemetry filters.
 * Inputs:  msg — null-terminated reply starting with 'V' followed by NVOLTS hex ADC fields.
 * Outputs:  Returns 0 on success, 1 on format error; fills volts[], updates depth filter state.
 * System Impact: Drives depth hold, LCD depth/PID lines, and TELEMETRY_TIMEOUT_MS staleness tracking.
 * Safety Notes: Failed or invalid samples set depthTelemetryValid false, which disengages hold.
 * Usage Notes: Applies EMA on pressure voltage then depth feet; first valid sample initializes filters.
 *             Depth filtering: DEPTH_VOLT_FILTER_ALPHA → feet → DEPTH_FILTER_ALPHA on filteredDepthFeet.
 */
int parse_reply_msg(char *msg) {
  int i;
  int val;

  if (*msg++ != 'V') return 1;    // ADC voltages

  for (i=0;i<NVOLTS;i++) {
    if ((val = atox(msg, VOLTS_DIGITS)) < 0) return 1;
    msg += VOLTS_DIGITS;
    volts[i] = (float)(val) * 3.3/4096.;  // save it
  }

  if (*msg++ != '\n') return 1; 
  if (*msg++ != '\0') return 1; 
  currentDepthFeet = depthFeetFromPressureVolts(volts[5]);
  if (depthTelemetrySampleValid(currentDepthFeet, volts[5])) {
    if (!depthTelemetryValid) {
      filteredPressureVolts = volts[5];
      filteredDepthFeet = currentDepthFeet;
      previousDepthFeet = currentDepthFeet;
      filteredDepthRate = 0.0f;
    } else {
      filteredPressureVolts = filteredPressureVolts * (1.0f - DEPTH_VOLT_FILTER_ALPHA)
                            + volts[5] * DEPTH_VOLT_FILTER_ALPHA;
      float depthFromFilteredV = depthFeetFromPressureVolts(filteredPressureVolts);
      filteredDepthFeet = filteredDepthFeet * (1.0f - DEPTH_FILTER_ALPHA)
                        + depthFromFilteredV * DEPTH_FILTER_ALPHA;
    }
    depthTelemetryValid = true;
    lastTelemetryMillis = millis();
#if TOP_DEBUG_SERIAL
    Serial.printf("DEPTH V=%.3f ft=%.2f filt=%.2f hold=%d\n",
                  volts[5], currentDepthFeet, filteredDepthFeet, depthHoldEnabled ? 1 : 0);
#endif
  } else {
    depthTelemetryValid = false;
  }
  return 0;
}

// ----------------- translation code -------------------- //

/*
 * Purpose: Apply stick deadband and scale 16-bit joystick ADC to −1..+1 float.
 * Inputs:  stick — raw axis value from gamepad (typically ±32768 range).
 * Outputs:  Normalized command; zero inside ±DEADBAND.
 * System Impact: All stick-driven requestedAnalogs[] values pass through here before slew/hold.
 * Safety Notes: Deadband prevents drift from mechanical center error.
 * Usage Notes: Used for left/right stick X/Y; right stick Y axis index differs in hardware mapping.
 */
float joyScale(int stick) {
  int val = 0;
  if (stick >  DEADBAND) val = (stick - DEADBAND)/256;
  if (stick < -DEADBAND) val = (stick + DEADBAND)/256;
  return (float)(val) * 32768. / (32768.-DEADBAND)/128.; // scale out the deadband
}

/*
 * Purpose: Scale 10-bit trigger ADC reading to 0..1 float.
 * Inputs:  stick — raw trigger axis (0..1023 typical).
 * Outputs:  Normalized trigger value in 0..1 range.
 * System Impact: Feeds gripper latch in translate_controls_to_commands() and GRIPPER_TRIGGER_ON gate.
 * Safety Notes: Triggers below GRIPPER_TRIGGER_ON are ignored for gripper motion.
 * Usage Notes: Triggers are not symmetric sticks — different scaling than joyScale().
 */
float trigScale(int stick) {
  return (float)(stick)/1024.;
}

/*
 * Purpose: Aggregate D-pad direction booleans from button bits, bitmask, and hat switch.
 * Inputs:  References up/down/left/right filled by caller; uses buttons[], butts, activeGamepadIndex.
 * Outputs:  Sets the four direction flags true when any source asserts that direction.
 * System Impact: Feeds updateVirtualPadAxes() for LED dim (DPadX) and optional camera tilt (DPadY).
 * Safety Notes: None — read-only gamepad decode.
 * Usage Notes: Hat axis 9 on the active pad is decoded for diagonal combinations.
 */
void readDpadDirections(bool &up, bool &down, bool &left, bool &right) {
  up = buttons[DpadUp] || (butts & 0x10000);
  down = buttons[DpadDown] || (butts & 0x40000);
  left = buttons[DpadLeft] || (butts & 0x80000);
  right = buttons[DpadRight] || (butts & 0x20000);
  if (activeGamepadIndex >= 0 && (bool)joysticks[activeGamepadIndex]) {
    int hat = joysticks[activeGamepadIndex].getAxis(9);
    if (hat >= 0 && hat <= 7) {
      up |= (hat == 0 || hat == 1 || hat == 7);
      right |= (hat == 1 || hat == 2 || hat == 3);
      down |= (hat == 3 || hat == 4 || hat == 5);
      left |= (hat == 5 || hat == 6 || hat == 7);
    }
  }
}

/*
 * Purpose: Integrate D-pad and face-button inputs into virtual analog axes DPadX/Y and ButsX/Y.
 * Inputs:  buttons[], analogs[] current values, button_inc step, compile-time camera options.
 * Outputs:  Updates analogs[DPadX], analogs[DPadY], analogs[ButsX], analogs[ButsY] clamped to ±1.
 * System Impact: DPadX → servos[0] LED dim; DPadY → servos[1] when CAMERA_TILT_ENABLED.
 * Safety Notes: Values saturate at ±1 — no runaway from held buttons.
 * Usage Notes: Called each control tick after raw stick/trigger scaling in translate_response_to_controls().
 */
void updateVirtualPadAxes(void) {
  bool dUp, dDown, dLeft, dRight;
  readDpadDirections(dUp, dDown, dLeft, dRight);
  float val = analogs[DPadX];
  if (dRight) val += button_inc;
  if (dLeft) val -= button_inc;
  if (val > 1.0f) val = 1.0f;
  if (val < -1.0f) val = -1.0f;
  analogs[DPadX] = val;
  val = analogs[DPadY];
#if CAMERA_TILT_ENABLED
#if CAMERA_DPAD_SNAP
  if (dUp) val = 1.0f;
  else if (dDown) val = -1.0f;
#else
  if (dUp) val += CAMERA_DPAD_STEP;
  if (dDown) val -= CAMERA_DPAD_STEP;
  if (val > 1.0f) val = 1.0f;
  if (val < -1.0f) val = -1.0f;
#endif
#endif
  analogs[DPadY] = val;
  for (int i = ButsX; i < NANALOG; i++) {
    val = analogs[i];
    if (buttons[up_button[i]]) val += button_inc;
    if (buttons[dn_button[i]]) val -= button_inc;
    if (val > 1.0f) val = 1.0f;
    if (val < -1.0f) val = -1.0f;
    analogs[i] = val;
  }
}

/*
 * Purpose: Select which connected gamepad index supplies axis and button data.
 * Inputs:  joysticks[] connection and available() state.
 * Outputs:  Index 0..COUNT_JOYSTICKS-1, or -1 if none connected.
 * System Impact: Determines activeGamepadIndex for hat/D-pad reads and control translation.
 * Safety Notes: Returns first available pad; multi-pad setups use lowest index wins.
 * Usage Notes: Prefers pads reporting available(); falls back to merely connected instance.
 */
int findActiveGamepadIndex(void) {
  for (int i = 0; i < COUNT_JOYSTICKS; i++) {
    if ((bool)joysticks[i] && joysticks[i].available()) return i;
  }
  for (int i = 0; i < COUNT_JOYSTICKS; i++) {
    if ((bool)joysticks[i]) return i;
  }
  return -1;
}

/*
 * Purpose: Slew-limit a scalar toward a target using separate accel and decel rates.
 * Inputs:  current, target — present and desired values; accelRate, decelRate — units per second;
 *          dtSeconds — elapsed time since last update.
 * Outputs:  New value moved toward target by at most rate*dt; may stop at zero on sign reversal.
 * System Impact: Core of applySlewRate() stick smoothing and depth-hold output ramping.
 * Safety Notes: Sign reversal forces intermediate target 0 with decel rate to avoid instant reverse thrust.
 * Usage Notes: Decel rate applies when reducing magnitude or crossing zero; used for slow-mode feel.
 */
float limitRateOfChange(float current, float target, float accelRate, float decelRate, float dtSeconds) {
  float rate = accelRate;
  if ((current > 0.0 && target < current) || (current < 0.0 && target > current)) {
    rate = decelRate;
  }
  if ((current > 0.0 && target < 0.0) || (current < 0.0 && target > 0.0)) {
    target = 0.0;
    rate = decelRate;
  }
  float maxStep = rate * dtSeconds;
  float delta = target - current;
  if (delta > maxStep) return current + maxStep;
  if (delta < -maxStep) return current - maxStep;
  return target;
}

/*
 * Purpose: Toggle slow mode on rising edge of LB+RB bumper combo.
 * Inputs:  buttons[LButton], buttons[RButton], lbRbComboWasPressed latch.
 * Outputs:  Flips slow_mode_enabled; refreshes LCD row 0 on toggle.
 * System Impact: Slow mode scales stick requests (SLOW_MODE_SCALE) and lowers slew accel/decel rates.
 * Safety Notes: Momentary combo only — holding both does not repeat toggle.
 * Usage Notes: Polled every control frame inside translate_response_to_controls().
 */
void updateSlowModeToggle(void) {
  bool lbRbComboPressed = buttons[LButton] && buttons[RButton];
  if (lbRbComboPressed && !lbRbComboWasPressed) {
    slow_mode_enabled = !slow_mode_enabled;
    lbRbComboWasPressed = true;
    display_slow_mode_line();
  }
  if (!lbRbComboPressed) {
    lbRbComboWasPressed = false;
  }
}

/*
 * Purpose: Clamp a float to an inclusive min/max range.
 * Inputs:  val — value to limit; minVal, maxVal — bounds.
 * Outputs:  val constrained to [minVal, maxVal].
 * System Impact: Used for PID output, integral anti-windup, and gripper command saturation.
 * Safety Notes: Hard clamp — no wrapping or modulo behavior.
 * Usage Notes: Prefer over Arduino constrain() for float paths in this file.
 */
float constrainFloat(float val, float minVal, float maxVal) {
  if (val < minVal) return minVal;
  if (val > maxVal) return maxVal;
  return val;
}

/*
 * Purpose: Report whether any USB gamepad driver is currently connected.
 * Inputs:  driver_active[] and hid_driver_active[] connection flags.
 * Outputs:  true if any joystick slot shows active at driver or HID layer.
 * System Impact: Loss of connection triggers depth-hold disengage in updateDepthHoldAssist().
 * Safety Notes: Disconnected pad stops slew application but loop continues sending last motor frame until next command build.
 * Usage Notes: Checked before applySlewRate() and inside depth-hold safety gates.
 */
bool joystickConnected(void) {
  return driver_active[1] || driver_active[2] || driver_active[3] || driver_active[4] ||
         hid_driver_active[0] || hid_driver_active[1] || hid_driver_active[2] || hid_driver_active[3];
}

/*
 * Purpose: Clear depth-hold PID state and held vertical output accumulator.
 * Inputs:  None (uses module globals).
 * Outputs:  Zeros depthIntegral, previousDepthError, depthPidOutput, depthHoldOutput, filteredDepthRate.
 * System Impact: Called on hold disengage, stick release edge, and before re-arming hold.
 * Safety Notes: Does not change depthHoldEnabled flag — use disableDepthHold() for full off.
 * Usage Notes: Invoke whenever hold must restart integrator from a clean state.
 */
void resetDepthHoldPid(void) {
  depthIntegral = 0.0;
  previousDepthError = 0.0;
  depthPidOutput = 0.0;
  depthHoldOutput = 0.0;
  filteredDepthRate = 0.0f;
}

/*
 * Purpose: Turn off depth hold, reset PID, and refresh the PID LCD line if state changed.
 * Inputs:  depthHoldEnabled and related globals.
 * Outputs:  depthHoldEnabled false; verticalStickReleased false; PID state cleared; LCD updated when was ON.
 * System Impact: Returns vertical axis to manual slew path in applySlewRate().
 * Safety Notes: Central disengage helper — all automatic safety exits should pass through here.
 * Usage Notes: Called when telemetry invalid, stick deflected, joystick lost, or sample out of range.
 */
void disableDepthHold(void) {
  if (depthHoldEnabled) {
    depthHoldEnabled = false;
    display_pid_hold_line();
  }
  verticalStickReleased = false;
  resetDepthHoldPid();
}

/*
 * Purpose: Automatic depth-hold assist on right-stick Y using filtered telemetry and PID.
 * Inputs:  dtSeconds — loop delta; globals for telem, sticks, and PID state.
 * Outputs:  Sets analogs[RJoyY] to slewed hold output when engaged; may call disableDepthHold().
 * System Impact: Overrides manual vertical slew while hold active; couples to vertical thrusters via RJoyY.
 * Safety Notes: Disengages when: joystick disconnected; telemetry invalid or stale (>TELEMETRY_TIMEOUT_MS);
 *              filtered sample out of range; |requestedAnalogs[RJoyY]| > VERTICAL_DEADBAND (operator override).
 * Usage Notes: After stick centered, waits DEPTH_HOLD_ACTIVATION_DELAY_MS then latches targetDepthFeet.
 *             PID: P on deadbanded error, I with anti-windup, D on filtered depth rate; output slewed
 *             at DEPTH_PID_SLEW_RATE and clamped to MAX_VERTICAL_PID_OUTPUT; sign via DEPTH_PID_OUTPUT_SIGN.
 */
void updateDepthHoldAssist(float dtSeconds) {
  float accelRate = slow_mode_enabled ? ACCEL_RATE_SLOW : ACCEL_RATE_NORMAL;
  float decelRate = slow_mode_enabled ? DECEL_RATE_SLOW : DECEL_RATE_NORMAL;

  if (depthTelemetryValid && (millis() - lastTelemetryMillis > TELEMETRY_TIMEOUT_MS)) {
    depthTelemetryValid = false;
  }

  if (!joystickConnected() || !depthTelemetryValid) {
    disableDepthHold();
    return;
  }

  if (!depthTelemetrySampleValid(filteredDepthFeet, volts[5])) {
    disableDepthHold();
    return;
  }

  if (fabsf(requestedAnalogs[RJoyY]) > VERTICAL_DEADBAND) {
    disableDepthHold();
    return;
  }

  if (!verticalStickReleased) {
    verticalStickReleased = true;
    verticalReleaseMillis = millis();
    resetDepthHoldPid();
    return;
  }

  if (!depthHoldEnabled) {
    if (millis() - verticalReleaseMillis < DEPTH_HOLD_ACTIVATION_DELAY_MS) {
      analogs[RJoyY] = limitRateOfChange(analogs[RJoyY], 0.0f, accelRate, decelRate, dtSeconds);
      resetDepthHoldPid();
      return;
    }
    targetDepthFeet = filteredDepthFeet;
    depthHoldEnabled = true;
    resetDepthHoldPid();
    depthHoldOutput = analogs[RJoyY];
    previousDepthFeet = filteredDepthFeet;
    display_pid_hold_line();
  }

  float error = targetDepthFeet - filteredDepthFeet;
  float errorForPid = applyDepthErrorDeadband(error);

  float depthRate = 0.0f;
  if (dtSeconds > 0.0f) depthRate = (filteredDepthFeet - previousDepthFeet) / dtSeconds;
  previousDepthFeet = filteredDepthFeet;
  if (fabsf(depthRate) < DEPTH_RATE_DEADBAND) depthRate = 0.0f;
  filteredDepthRate = filteredDepthRate * (1.0f - DEPTH_RATE_FILTER_ALPHA)
                    + depthRate * DEPTH_RATE_FILTER_ALPHA;

  if (fabsf(error) > DEPTH_HOLD_DEADBAND) {
    depthIntegral += errorForPid * dtSeconds;
    depthIntegral = constrainFloat(depthIntegral, -DEPTH_INTEGRAL_MAX, DEPTH_INTEGRAL_MAX);
  }

  depthPidOutput = errorForPid * DEPTH_KP + depthIntegral * DEPTH_KI - filteredDepthRate * DEPTH_KD;
  depthPidOutput *= DEPTH_PID_OUTPUT_SIGN;
  float pidUnclamped = depthPidOutput;
  depthPidOutput = constrainFloat(depthPidOutput, -MAX_VERTICAL_PID_OUTPUT, MAX_VERTICAL_PID_OUTPUT);

  if (pidUnclamped > MAX_VERTICAL_PID_OUTPUT && errorForPid > 0.0f) {
    depthIntegral -= errorForPid * dtSeconds;
  } else if (pidUnclamped < -MAX_VERTICAL_PID_OUTPUT && errorForPid < 0.0f) {
    depthIntegral -= errorForPid * dtSeconds;
  }

  previousDepthError = errorForPid;
  depthHoldOutput = limitRateOfChange(depthHoldOutput, depthPidOutput,
                                      DEPTH_PID_SLEW_RATE, DEPTH_PID_SLEW_RATE, dtSeconds);
  analogs[RJoyY] = depthHoldOutput;
}

/*
 * Purpose: Apply slow-mode scaling and slew limits to stick analog outputs.
 * Inputs:  dtSeconds — time since last slew update; requestedAnalogs[], slow_mode_enabled.
 * Outputs:  Updates analogs[LJoyX/Y], analogs[RJoyX], and analogs[RJoyY] when hold is off.
 * System Impact: Shapes all manual drive/steer/dive commands before motor mixing.
 * Safety Notes: RJoyY skipped while depthHoldEnabled — hold owns vertical axis.
 * Usage Notes: Slow mode uses SLOW_MODE_SCALE on targets and ACCEL/DECEL_RATE_SLOW pair.
 */
void applySlewRate(float dtSeconds) {
  float accelRate = slow_mode_enabled ? ACCEL_RATE_SLOW : ACCEL_RATE_NORMAL;
  float decelRate = slow_mode_enabled ? DECEL_RATE_SLOW : DECEL_RATE_NORMAL;
  float scale = slow_mode_enabled ? SLOW_MODE_SCALE : 1.0;

  analogs[LJoyY] = limitRateOfChange(analogs[LJoyY], requestedAnalogs[LJoyY] * scale, accelRate, decelRate, dtSeconds);
  analogs[LJoyX] = limitRateOfChange(analogs[LJoyX], requestedAnalogs[LJoyX] * scale, accelRate, decelRate, dtSeconds);
  analogs[RJoyX] = limitRateOfChange(analogs[RJoyX], requestedAnalogs[RJoyX] * scale, accelRate, decelRate, dtSeconds);
  if (!depthHoldEnabled) {
    analogs[RJoyY] = limitRateOfChange(analogs[RJoyY], requestedAnalogs[RJoyY] * scale, accelRate, decelRate, dtSeconds);
  }
}

/*
 * Purpose: Read active gamepad, populate control arrays, and run slew/depth-hold processing.
 * Inputs:  USB joysticks[] state; prior analogs and timing globals.
 * Outputs:  Updates buttons[], axes[], requestedAnalogs[], analogs[], switches[], activeGamepadIndex.
 * System Impact: Single entry for human input each command tick; chains slow mode, virtual pads, slew, hold.
 * Safety Notes: Only first available pad is used; no pad connected skips slew but hold logic still runs disengage checks.
 * Usage Notes: Called at fixed loopPeriod from loop() before translate_controls_to_commands().
 */
void translate_response_to_controls() {
  unsigned long nowMicros = micros();
  float dtSeconds = loopPeriod / 1000000.0;
  if (lastSlewMicros != 0) {
    dtSeconds = (nowMicros - lastSlewMicros) / 1000000.0;
  }
  if (dtSeconds > 0.10) dtSeconds = 0.10;
  lastSlewMicros = nowMicros;
  activeGamepadIndex = -1;
  for (int joystick_index = 0; joystick_index < COUNT_JOYSTICKS; joystick_index++) {
    if (!(bool)joysticks[joystick_index] || !joysticks[joystick_index].available()) continue;
    activeGamepadIndex = joystick_index;
      butts = joysticks[joystick_index].getButtons();
   //   Serial.printf("micros %9d  buttons %04X", nextLoopMicros, butts);
      // Primary six hardware axes
      for (uint8_t i = 0; i<6; i++) {
        axes[i] = joysticks[joystick_index].getAxis(i);
  //      Serial.printf(" %6d", axes[i]);
      }
  //    Serial.println();
      // Expand button word into per-button array
      for (int i=0;i<16;i++) {
        buttons[i] = (butts>>i)&0x0001;
      }
  
      // Sticks and triggers → normalized requested analogs
      updateSlowModeToggle();
      requestedAnalogs[LJoyX] = joyScale(axes[0]);
      requestedAnalogs[LJoyY] = joyScale(axes[1]);
      requestedAnalogs[RJoyX] = joyScale(axes[2]);
      requestedAnalogs[RJoyY] = joyScale(axes[5]);
      analogs[LTrig] = trigScale(axes[3]);
      analogs[RTrig] = trigScale(axes[4]);
  
      updateVirtualPadAxes();
      // Y and B buttons map to ROV switch channels
      switches[0] = buttons[YButton];
      switches[1] = buttons[BButton];
      
      if (0) {
        for (int i=0;i<NANALOG;i++) {
          Serial.printf(" %6.3f", analogs[i]);
        }
        Serial.println();
      }
    break;
  }
  if (joystickConnected()) {
    applySlewRate(dtSeconds);
  }
  updateDepthHoldAssist(dtSeconds);
}

// Motor PWM scale factor (analog ±1 → ±128 before lims)
float motScale = 128.;

/*
 * Purpose: Clamp motor/servo command delta to ±127 before hex packing.
 * Inputs:  val — signed command offset from Pwm0 center.
 * Outputs:  val limited to [-127, 127].
 * System Impact: Prevents two-digit hex motor fields from wrapping or saturating incorrectly.
 * Safety Notes: Saturation reduces commanded thrust but avoids protocol overflow.
 * Usage Notes: Applied to every motor and servo after gain and direction multiply.
 */
int lims(int val) {
  int res = val;
  if (res > 127) res = 127;
  if (res < -127) res = -127;
  return res;
}

/*
 * Purpose: Mix normalized analogs into motor and servo PWM command bytes.
 * Inputs:  analogs[], motDirs[], gain trims, gripperCmd latch state, triggers.
 * Outputs:  Fills motors[] and servos[] centered on Pwm0; updates latched gripperCmd.
 * System Impact: Final motion mapping sent to ROV each command frame.
 * Safety Notes: Gripper uses latched position — triggers step open/close while held, position retained on release (no auto-center).
 * Usage Notes: LJoyX strafe, LJoyY drive, RJoyX steer, RJoyY dive (or PID hold output); DPadX → LED;
 *             DPadY → camera when CAMERA_TILT_ENABLED; pitch trim on vertical mix from forward stick.
 */
void translate_controls_to_commands() {
  float LRForeAft = analogs[LJoyY]*DriveGainL + analogs[RJoyX]*SteerGain;
  float RRForeAft = analogs[LJoyY]*DriveGainR - analogs[RJoyX]*SteerGain;
  float Strafe    = analogs[LJoyX]*StrafeGain;
  float pitchTrim = analogs[LJoyY] * DRIVE_PITCH_TRIM;
  float LUpDown   = analogs[RJoyY]*DiveGainL + pitchTrim;
  float RUpDown   = analogs[RJoyY]*DiveGainR + pitchTrim;
  // Gripper latch: RT closes, LT opens, while above GRIPPER_TRIGGER_ON; cmd holds last position when triggers released
  if (analogs[RTrig] > GRIPPER_TRIGGER_ON) {
    gripperCmd += GRIPPER_STEP * analogs[RTrig];
  } else if (analogs[LTrig] > GRIPPER_TRIGGER_ON) {
    gripperCmd -= GRIPPER_STEP * analogs[LTrig];
  }
  if (gripperCmd > 1.0f) gripperCmd = 1.0f;
  if (gripperCmd < -1.0f) gripperCmd = -1.0f;
  motors[GripperMot] = Pwm0 + lims((int)(gripperCmd * motDirs[GripperMot] * 127.0f));
  motors[LUpDownMot] = Pwm0 + lims((int)(LUpDown  *motDirs[LUpDownMot]*motScale));
  motors[RUpDownMot] = Pwm0 + lims((int)(RUpDown  *motDirs[RUpDownMot]*motScale));
  motors[LForAftMot] = Pwm0 + lims((int)(LRForeAft*motDirs[LForAftMot]*motScale));
  motors[RForAftMot] = Pwm0 + lims((int)(RRForeAft*motDirs[RForAftMot]*motScale));
  motors[StrafeMot ] = Pwm0 + lims((int)(Strafe   *motDirs[StrafeMot ]*motScale));

  // servos[0]=LED dim; servos[1]=camera tilt when enabled
  servos[0] = Pwm0 + lims((int)((analogs[DPadX]) * motScale));  // LED dimming
#if CAMERA_TILT_ENABLED
  servos[1] = Pwm0 + lims((int)(-(analogs[DPadY]) * motScale));  // camera tilt
#else
  servos[1] = Pwm0;
#endif
}


/*
 * Purpose: Serialize motors[], servos[], and switches[] into hex-ASCII command telegram.
 * Inputs:  msg — buffer with room for full 'M' frame; global command arrays.
 * Outputs:  Null-terminated string: M + motors + P + servos + S + switches + C + checksum + newline.
 * System Impact: Transmitted on Serial1 each control tick to drive ROV actuators.
 * Safety Notes: Checksum is low byte of sum of all motor/servo/switch bytes — ROV should reject bad frames.
 * Usage Notes: Caller passes command_msg[]; function advances pointer internally via sprintf.
 */
void build_command_msg(char *msg) {
  int i, checksum = 0;

  *msg++ = 'M';
  for (i=0;i<NMOTORS;i++) {
    checksum += motors[i];
    msg += sprintf(msg, "%02X", motors[i]);
  }

  *msg++ = 'P';
  for (i=0;i<NSERVOS;i++) {
    checksum += servos[i];
    msg += sprintf(msg, "%02X", servos[i]);
  }

  *msg++ = 'S';
  for (i=0;i<NSWITCHES;i++) {
    checksum += switches[i];
    msg += sprintf(msg, "%01X", switches[i]);
  }

  *msg++ = 'C';
  checksum = checksum%0x100;
  msg += sprintf(msg, "%02X", checksum);
  *msg++ = '\n'; 
  *msg = '\0';
}

/*
 * Purpose: Detect USB connect/disconnect events and log or display device identity.
 * Inputs:  drivers[] and hiddrivers[] vs their *_active[] shadow flags.
 * Outputs:  Serial diagnostics; LCD vendor:product on line 1 when a device attaches; updates active flags.
 * System Impact: Maintains driver_active/hid_driver_active used by joystickConnected().
 * Safety Notes: Disconnect clears active flags — depth hold disengages on next control pass.
 * Usage Notes: Call myusb.Task() before this each loop iteration.
 */
void PrintDeviceListChanges() {
  for (uint8_t i = 0; i < CNT_DEVICES; i++) {
    if (*drivers[i] != driver_active[i]) {
      if (driver_active[i]) {
        Serial.printf("*** Device %s - disconnected ***\n", driver_names[i]);
        driver_active[i] = false;
      } else {
        Serial.printf("*** Device %s %x:%x - connected ***\n", driver_names[i], drivers[i]->idVendor(), drivers[i]->idProduct());
        sprintf(namebuf,"Dev ID %04X:%04X", drivers[i]->idVendor(), drivers[i]->idProduct());
        // Show USB VID:PID on LCD for field identification
        lcd.setCursor(0, 1);
        lcd.print(namebuf);   
        driver_active[i] = true;

        const uint8_t *psz = drivers[i]->manufacturer();
        if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
        psz = drivers[i]->product();
        if (psz && *psz) Serial.printf("  product: %s\n", psz);
        psz = drivers[i]->serialNumber();
        if (psz && *psz) Serial.printf("  Serial: %s\n", psz);
      }
    }
  }

  for (uint8_t i = 0; i < CNT_HIDDEVICES; i++) {
    if (*hiddrivers[i] != hid_driver_active[i]) {
      if (hid_driver_active[i]) {
        Serial.printf("*** HID Device %s - disconnected ***\n", hid_driver_names[i]);
        hid_driver_active[i] = false;
      } else {
        Serial.printf("*** HID Device %s %x:%x - connected ***\n", hid_driver_names[i], hiddrivers[i]->idVendor(), hiddrivers[i]->idProduct());
        sprintf(namebuf,"Dev ID %04X:%04X", hiddrivers[i]->idVendor(), hiddrivers[i]->idProduct());
        // Show USB VID:PID on LCD for field identification
        lcd.setCursor(0, 1);
        lcd.print(namebuf);   
        hid_driver_active[i] = true;

        const uint8_t *psz = hiddrivers[i]->manufacturer();
        if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
        psz = hiddrivers[i]->product();
        if (psz && *psz) Serial.printf("  product: %s\n", psz);
        psz = hiddrivers[i]->serialNumber();
        if (psz && *psz) Serial.printf("  Serial: %s\n", psz);
      }
    }
  }
}

/*
 * Purpose: One-time hardware and state initialization before main loop runs.
 * Inputs:  Compile-time pin/baud/motor direction defines; none at runtime.
 * Outputs:  Configured Serial, Serial1, LCD, USB host; neutral motor/servo commands; timers primed.
 * System Impact: Establishes safe centered outputs and 20 Hz loop schedule.
 * Safety Notes: All motors/servos set to Pwm0 neutral; CPU clock lowered if >100 MHz for LCD timing.
 * Usage Notes: Shows splash then "Seeking Gamepad"; USB host starts after LCD message.
 */
void setup() {
  // Reduce CPU clock when above 100 MHz so LCD timing is reliable
  if ( F_CPU_ACTUAL >= 100'000'000 )
    set_arm_clock(100'000'000);

// Load per-motor direction signs from compile-time constants
motDirs[GripperMot] = GripperMotDir;  
motDirs[LUpDownMot] = LUpDownMotDir; 
motDirs[RUpDownMot] = RUpDownMotDir;
motDirs[LForAftMot] = LForAftMotDir;
motDirs[RForAftMot] = RForAftMotDir;
motDirs[StrafeMot]  = StrafeMotDir;

  // USB debug console
  Serial.begin(115200);
  delay(100);
  Serial.println("Starting Teensy");

  // RS-485 link to ROV
  Serial1.begin(BAUD_RATE); 
  Serial1.transmitterEnable(SER_TXEN);
  Serial.println("Starting LCD");
  
  lcd.begin(20,4);    // LCD size
  lcd.clear();  
  lcd.setCursor(0, 0);
  lcd.print("   ROVotron Cadet  ");    // display splash screen
  lcd.setCursor(0, 1);
  lcd.print(" ROV Control System");
  lcd.setCursor(0, 3);
  lcd.print("     Rev. 1.21");
//  Serial.println("LCD should be alive");
  delay(1500);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Seeking Gamepad");   // the USB ID of gamepad will display below this
  lcd.setCursor(0, 2);
  lcd.print("Slow Mode:OFF     ");
// USB host stack for gamepad
  myusb.begin();
// Neutral safe command defaults
  for (int i=0;i<NMOTORS;i++) 
    motors[i] = Pwm0;
  for (int i=0;i<NSERVOS;i++) 
    servos[i] = Pwm0;
  for (int i = 0; i < NANALOG; i++) {
    analogs[i] = 0.0f;
    requestedAnalogs[i] = 0.0f;
  }
  inPtr = reply_msg;
  gotInString = false;
  nextLoopMicros = micros();
  Serial.println("TOP ready");
}

/*
 * Purpose: Main recurring task — USB service, telemetry RX, display refresh, and timed command TX.
 * Inputs:  Serial1 bytes, USB gamepad state, micros() schedule.
 * Outputs:  RS-485 command frames at loopPeriod; LCD updates on each complete reply.
 * System Impact: Ties together parse → display → control → build → transmit pipeline.
 * Safety Notes: Telemetry parsed before control tick so depth hold sees fresh data; hold disengage
 *              conditions evaluated inside updateDepthHoldAssist() each command frame.
 * Usage Notes: Non-blocking RX accumulates reply_msg until newline; control block runs on timer only.
 */
void loop() {    
 // unsigned long mics = micros();
 // Serial.printf("Mics %d\n", mics);
  myusb.Task();
  PrintDeviceListChanges();  // show USB connect/disconnect activity
  
  // Non-blocking assembly of ROV reply string
  if (Serial1.available()) {
    int inChr = Serial1.read();
    if (inChr) {
      if (inPtr < reply_msg + sizeof(reply_msg) - 2) {
        *inPtr++ = inChr;
      } else {
        inPtr = reply_msg;
      }
    }
    if (inChr == '\n') {
      *inPtr++ = 0x00;
      gotInString = true;
      inPtr = reply_msg;
    }
  }
  // Parse and display before PID so depth hold uses latest filtered sample
  if (gotInString) {
#if TOP_DEBUG_SERIAL
    Serial.print(reply_msg);
#endif
    gotInString = false;
    parse_reply_msg(reply_msg);
    display_all_telems();
  }
  // Fixed-rate command generation and transmit
  if (micros() > nextLoopMicros) {
    nextLoopMicros += loopPeriod;
    translate_response_to_controls();
    translate_controls_to_commands();
    build_command_msg(command_msg);
    Serial1.print(command_msg);
#if TOP_DEBUG_SERIAL
    Serial.println(command_msg);
#endif
  }
}

	     

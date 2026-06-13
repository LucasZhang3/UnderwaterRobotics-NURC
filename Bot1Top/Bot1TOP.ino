/* RVTOP-Ccode
 * ROVotron Cadet topside controller — primary firmware image

(C) 2010, 2023, 2024 David Forbes

System summary

The topside box drives an ROV over RS-485 and shows live vehicle data on a 4×20 LCD.
An Xbox One gamepad connected via USB host provides all operator inputs.

Each 20 Hz cycle reads the gamepad, mixes stick inputs into six thruster channels,
formats a hex-ASCII command frame, and transmits it on Serial1 (RS-485).
Inbound reply frames carry ADC readings that are scaled and rendered on the LCD.

Motor routing, gains, deadband, and telemetry scaling are fixed in compile-time tables.

Revision history

2023-03-26 DF  forked from rtxa.c code, converting to Teensy and Xbox one
2023-09-09 DF  Removing all the config code
2023-09-10 DF  Corrected directions and trim, it works!
2023-09-11 DF  Made vector drive optional with a define
2023-11-25 DF  Removed vector drive, reduced number of motors and servos
2024-02-11 DF  Added motor index, matched motors to instructions
2024-02-22 DF  Added telems[5] for pressure
2024-02-25 DF  Updated depth scale factor and units
2024-05-31 DF  Fixed USB declarations to compile with TeensyDuino 1.59
2024-06-02 DF  Ver 1.20 - Added USB ID display at startup
2024-06-09 DF  Ver 1.21 - added temp readout enable to Y button

Known bugs:

Things to do:

Calibrate temp sensors

Operational role

Bridges an Xbox One controller to an ROV over a half-duplex RS-485 link.
Command packets use hex-ASCII motor, servo, and switch fields; replies carry
telemetry ADC values. The LCD presents battery, depth, and temperature readouts.

Wire format details: see RVdataDescr.txt

Glossary

Buttons — fourteen discrete gamepad face, shoulder, and D-pad inputs
Analogs — ten normalized axes (−1.0 … +1.0), including sticks, triggers, and synthesized pads
Motors — ROV thruster channels 0..5 (channel 0 is the gripper)
Servos — PWM servo outputs 0..2 (channel 0 controls LED dimming)
volts — raw 12-bit ADC samples from the vehicle (index 0 is battery)
telems — volts converted to engineering units via zero offset and scale factors

Motor channel parameters (vehicle-side; referenced here for gain context)
Gain — analog-to-motor multiplier, range −99 … +99
mode — speed behavior: 0 OFF, 1 SPEED (direct pass-through), 2 RATE (integrated position/speed)
In RATE mode, Gain sets the rate at which a full stick deflection changes output

*/

// Teensy peripheral libraries for LCD and USB host
#include <LiquidCrystalFast.h>
#include "USBHost_t36.h"

// ------------------ Teensy pin definitions ----------------------- //

// LCD data bus — four bits, each with a 1 kΩ series resistor for 3.3 V compatibility
#define LCD_RS 5
#define LCD_RW 6
#define LCD_E  7
#define LCD_D4 24
#define LCD_D5 25
#define LCD_D6 26
#define LCD_D7 27

// Vehicle link uses hardware UART Serial1
#define BAUD_RATE 115200

// GPIO that enables the RS-485 transceiver transmit driver
#define SER_TXEN 2

// External ARM clock setter (Teensy core)
extern "C" uint32_t set_arm_clock(uint32_t frequency); // required prototype

// ------------ Sizes of things --------------------------- //

// Thruster command slots in the outbound frame
#define NMOTORS 6
// Servo command slots in the outbound frame
#define NSERVOS 2
// Binary switch slots in the outbound frame
#define NSWITCHES 2
// ADC channels in the inbound telemetry frame
#define NVOLTS 6
// Hex digit count per motor byte in the command string
#define MOTOR_DIGITS 2
// Hex digit count per servo byte in the command string
#define SERVO_DIGITS 2
// Hex digit count per ADC sample in the reply string
#define VOLTS_DIGITS 3

// Joystick center dead zone — suppresses Xbox One stick drift (counts, ±32768 scale)
#define DEADBAND 3000

// Neutral PWM command value written to each motor/servo channel at rest
#define Pwm0  128


// ----------------- Motor configuration ------------------ //

// Thruster indices aligned with vehicle wiring documentation
#define GripperMot 0
#define LUpDownMot 1
#define RUpDownMot 2
#define LForAftMot 3
#define RForAftMot 4
#define StrafeMot  5

// Per-motor sign multiplier — flip a define to reverse a thruster without rewiring
float motDirs[NMOTORS];

// Direction signs applied during thruster mixing (+1 or −1 per channel)
#define GripperMotDir  1. 
#define LUpDownMotDir  1.
#define RUpDownMotDir -1.
#define LForAftMotDir -1.
#define RForAftMotDir  1.
#define StrafeMotDir   1.

// Thruster mixing gains — tune for straight travel, balanced turn, dive, and strafe
// Left/right fore-aft trims compensate mechanical asymmetry; steer/dive/strafe/gripper
// gains scale stick authority before the 20 Hz command frame is built
float DriveGainL  = 0.50;
float DriveGainR  = 0.60;
float SteerGain   = 0.60;
float DiveGainL   = 0.60;    // reducing total current draw
float DiveGainR   = 0.60;
float StrafeGain  = 0.50;
float GripperGain = 0.50;   // to keep from breaking the plastic!

// Fast parallel LCD driver instance (RW pin enables read-back of display RAM)
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

// Scratch buffer for USB vendor/product ID strings shown on the LCD
char namebuf[100] = { 0 };


// USB host stack supports up to four gamepads on a hub (only one needed for ROVotron)
USBDriver *drivers[] = {&hub1, &joysticks[0], &joysticks[1], &joysticks[2], &joysticks[3], &hid1};
#define CNT_DEVICES (sizeof(drivers)/sizeof(drivers[0]))
const char * driver_names[CNT_DEVICES] = {"Hub1", "joystick[0D]", "joystick[1D]", "joystick[2D]", "joystick[3D]",  "HID1"};
bool driver_active[CNT_DEVICES] = {false, false, false, false};


// Parallel HID-input driver list for connect/disconnect logging
USBHIDInput *hiddrivers[] = {&joysticks[0], &joysticks[1], &joysticks[2], &joysticks[3]};
#define CNT_HIDDEVICES (sizeof(hiddrivers)/sizeof(hiddrivers[0]))
const char * hid_driver_names[CNT_DEVICES] = {"joystick[0H]", "joystick[1H]", "joystick[2H]", "joystick[3H]"};
bool hid_driver_active[CNT_DEVICES] = {false};

//------------ Global Variables ------------------------------- //

// Main-loop pacing: loopPeriod µs → 20 Hz command/reply cadence
unsigned long loopPeriod = 50000;  // in microseconds: 20 loops per second
unsigned long nextLoopMicros = 0;

char inString[100];          // Legacy buffer (unused by current RX path)
char inChr = 0;              // Legacy single-char RX scratch
char * inPtr;
bool gotInString = false;    // Set when a newline-terminated RS-485 reply has arrived

// ------------ XBox gamepad control mapping ----------------- //

// Raw USB gamepad state (butts, axes[]) is normalized into buttons[] and analogs[].
// analogs[] then feeds thruster mixing in translate_controls_to_commands().

// Count of logical analog channels, including button-synthesized axes
#define NANALOG 10
#define NBUTTONS 16

// Raw button bitmask from the active JoystickController
uint32_t butts;    // the single word that contains all button bits 
unsigned char buttons[NBUTTONS];  // button control values: 1 = pressed, 0 = not
int axes[6];     // the value of each real analog control axis from gamepad

// buttons[] indices — bit order in butts matches these slot numbers
// the var butts is in order of these bits
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

// analogs[] indices — Xbox One axis layout with triggers and synthesized D-pad/button pads
// This isn't quite in Xbox one order, trigs are weirdly placed
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

// Normalized stick/trigger/pad values in −1.0 … +1.0 (0 = neutral)
float analogs[NANALOG];    // analog control values extracted from gamepad

// Button pairs that increment/decrement synthesized axes (DPadX/Y, ButsX/Y) each 20 Hz tick
int up_button[NANALOG] = {0,0,0,0,0,0,DpadRight,DpadUp,  BButton,YButton};
int dn_button[NANALOG] = {0,0,0,0,0,0,DpadLeft, DpadDown,XButton,AButton};

// Step size for button-held synthesized axes per main-loop iteration (at 20 Hz)
float button_inc = 0.03;

// Outbound PWM command bytes assembled each cycle and packed into the RS-485 frame
int motors[NMOTORS];    // motor command values 0=full reverse, FF=full fwd
int servos[NSERVOS];    // servo command values 0=full CCW, FF=full CW
int switches[NSWITCHES];  // on/off switches based on buttons held down

// RS-485 message buffers — command_msg outbound, reply_msg inbound
char command_msg[100];  // command message, newline, 0
char reply_msg[100];    // reply message, newline, 0
char buf[100];      // temporary string storage

// Latest vehicle ADC samples and derived engineering values
float volts[NVOLTS];      // telemetry voltages 0.0 to 3.3V (we receive 0..0xFFF)
float telems[NVOLTS];     // telemetry in its units

// --------------- Telemetry calculation ----------------- //

// LCD scaling: subtract telZeros[], multiply by telScale[] to get user units
// Depth: ~0.5 V per atm (58 PSI/V); H2O temp on volts[1] uses 1/128 °C per count
// Channel map: Battery | H2O temp | unused | LED temp | spare | Depth (feet)

// Temp scale needs caclualtion...
//    signal              tether   ana1   ana2    ana3    ana4    press
//    measurement         Battery  H2Otemp not  LEDtemp   ---    Depth
//    units                Volts    degC   used   degC    --      Feet
float telZeros[NVOLTS] = {   0.00,  0.00,  0.10,   1.40,   1.40,  0.00};
float telScale[NVOLTS] = {  11.00,  256./3.3,  8.00,  70.00,  71.00, 90};

/*
 * Purpose: Convert latest volts[] ADC readings to engineering units and paint four LCD rows.
 * Inputs:  Global volts[], telZeros[], telScale[] (populated by parse_reply_msg).
 * Outputs: Updates LCD lines 0–3 with battery voltage, depth, LED temp, and water temp.
 * System Impact: Sole routine that refreshes operator-facing telemetry on the 4×20 display.
 * Safety Notes: Stale volts[] if no valid reply received — display may show last good sample.
 * Usage Notes: Called from loop() immediately after a complete RS-485 reply is parsed.
 */
void display_all_telems(void) {
  float telems[NVOLTS];
  for (int i=0;i<NVOLTS;i++) {
    telems[i] = (volts[i] - telZeros[i]) * telScale[i];
  }
  sprintf(buf, "Battery  %5.1f V", telems[0]);
  lcd.setCursor(0, 0);
  lcd.print(buf);
  sprintf(buf, "Depth    %5.2f Feet", telems[5]);
  lcd.setCursor(0, 1);
  lcd.print(buf);
  sprintf(buf, "LED temp %5.1f C", telems[3]);
  lcd.setCursor(0, 2);
  lcd.print(buf);
  sprintf(buf, "H2O temp %5.1f C", telems[1]);
  lcd.setCursor(0, 3);
  lcd.print(buf);
}

// ------------------- ROV message I/O ------------------ //

// Blocking RS-485 reader — not used by the current loop(); retained for reference.
// Active RX path accumulates bytes in loop() until newline.

/*
 * Purpose: Block-read one RS-485 message from Serial1, synchronizing on a start character.
 * Inputs:  msg — destination buffer; first_char — expected leading byte (e.g. 'V').
 * Outputs: Returns 0 on success (NUL-terminated msg); 1 on timeout.
 * System Impact: Legacy blocking I/O; current firmware uses non-blocking assembly in loop().
 * Safety Notes: Spin-waits with microsecond delays — can stall the 20 Hz cadence if called.
 * Usage Notes: Discards bytes until first_char, then reads through NUL with shorter timeout.
 */
int receive_msg(char *msg, char first_char) {
  int timer;

  // Hunt for the frame sync byte, dropping any leading garbage
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

  // Collect payload bytes until NUL with a tight inter-character timeout
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
 * Purpose: Decode one ASCII hex digit to its numeric value.
 * Inputs:  chr — character '0'–'9', 'A'–'F', or 'a'–'f'.
 * Outputs: Returns 0–15 for valid hex; −1 for any other character.
 * System Impact: Building block for atox() and parse_reply_msg() RS-485 reply decoding.
 * Safety Notes: Caller must treat −1 as a parse failure and abort frame processing.
 * Usage Notes: Case-insensitive; used for every nibble in inbound telemetry hex fields.
 */
char atoxdigit(char chr) {
  if (('a' <= chr) && (chr <= 'f')) return (chr - 'a' + 10);
  if (('A' <= chr) && (chr <= 'F')) return (chr - 'A' + 10);
  if (('0' <= chr) && (chr <= '9')) return (chr - '0');
  return -1;
}

/*
 * Purpose: Parse exactly n consecutive hex digits from a string into an integer.
 * Inputs:  str — pointer to first digit; n — digit count to consume.
 * Outputs: Returns parsed value, or −4 if any digit is invalid.
 * System Impact: Converts 3-digit hex ADC fields in RS-485 reply messages to raw counts.
 * Safety Notes: Does not bounds-check the numeric result against ADC full scale.
 * Usage Notes: Advances caller's msg pointer externally after each field read.
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
 * Purpose: Validate and decode an inbound 'V' telemetry reply into global volts[].
 * Inputs:  msg — NUL-terminated reply string beginning with 'V'.
 * Outputs: Returns 0 on success; 1 on format error. Fills volts[0..NVOLTS−1] in volts.
 * System Impact: Feeds display_all_telems() via volts[]; invoked on every complete reply.
 * Safety Notes: Rejects malformed frames — prior volts[] values remain if parse fails.
 * Usage Notes: Expects NVOLTS × VOLTS_DIGITS hex chars, then newline and trailing NUL.
 */
int parse_reply_msg(char *msg) {
  int i;
  int val;

  if (*msg++ != 'V') return 1;    // ADC voltages

  for (i=0;i<NVOLTS;i++) {
    if ((val = atox(msg, VOLTS_DIGITS)) == -1) return 1;
    msg += VOLTS_DIGITS;
    volts[i] = (float)(val) * 3.3/4096.;  // save it
  }

  if (*msg++ != '\n') return 1; 
  if (*msg++ != '\0') return 1; 
  return 0;
}

// ----------------- translation code -------------------- //

/*
 * Purpose: Apply DEADBAND to a raw 16-bit stick sample and normalize to ±1.0 float.
 * Inputs:  stick — raw axis value from JoystickController (±32768 range).
 * Outputs: Float in approximately −1.0 … +1.0; values inside deadband return 0.
 * System Impact: First stage of Xbox mapping before thruster mixing gains are applied.
 * Safety Notes: Deadband prevents drift-induced thruster commands at neutral stick.
 * Usage Notes: Used for all four joystick axes; triggers use trigScale() instead.
 */
float joyScale(int stick) {
  int val = 0;
  if (stick >  DEADBAND) val = (stick - DEADBAND)/256;
  if (stick < -DEADBAND) val = (stick + DEADBAND)/256;
  return (float)(val) * 32768. / (32768.-DEADBAND)/128.; // scale out the deadband
}

/*
 * Purpose: Normalize a 10-bit trigger sample to a 0.0 … +1.0 float.
 * Inputs:  stick — raw trigger axis (0 … 1023 typical).
 * Outputs: Float proportional to trigger pull depth.
 * System Impact: Drives analogs[LTrig] and analogs[RTrig] for gripper differential mixing.
 * Safety Notes: Triggers have no deadband — slight USB noise may affect gripper at rest.
 * Usage Notes: Xbox One reports triggers on axes[3] and axes[4], distinct from stick layout.
 */
float trigScale(int stick) {
  return (float)(stick)/1024.;
}

/*
 * Purpose: Poll USB gamepads and populate buttons[], analogs[], and switches[].
 * Inputs:  Global joysticks[] USB host objects (via myusb.Task() in loop).
 * Outputs: Updates butts, axes[], buttons[], analogs[], switches[] when a pad is available.
 * System Impact: Core Xbox mapping stage; runs each 20 Hz tick before thruster mixing.
 * Safety Notes: Only the first available joystick in the scan order controls the ROV.
 * Usage Notes: Synthesized DPadX/Y and ButsX/Y axes ramp via button_inc while held.
 */
void translate_response_to_controls() {
  for (int joystick_index = 0; joystick_index < COUNT_JOYSTICKS; joystick_index++) {
  // Poll each USB joystick slot for fresh HID reports
    if (joysticks[joystick_index].available()) {
      // Pack all digital inputs into a single 16-bit word
      butts = joysticks[joystick_index].getButtons();
   //   Serial.printf("micros %9d  buttons %04X", nextLoopMicros, butts);
      // Fetch the six hardware analog channels used by this mapping
      for (uint8_t i = 0; i<6; i++) {
        axes[i] = joysticks[joystick_index].getAxis(i);
  //      Serial.printf(" %6d", axes[i]);
      }
  //    Serial.println();
      // Expand the button bitmask into per-button array elements
      for (int i=0;i<16;i++) {
        buttons[i] = (butts>>i)&0x0001;
      }
  
      // Map physical sticks and triggers into normalized analogs[] (−1 … +1)
      analogs[LJoyX] = joyScale(axes[0]);
      analogs[LJoyY] = joyScale(axes[1]);
      analogs[RJoyX] = joyScale(axes[2]);
      analogs[RJoyY] = joyScale(axes[5]);
      analogs[LTrig] = trigScale(axes[3]);
      analogs[RTrig] = trigScale(axes[4]);
  
        // Integrate button-pair inputs into virtual axes 6–9 with clamping
      for (int i=6; i<NANALOG; i++) {
        float val = analogs[i];  // get current position
        if (buttons[up_button[i]]) val += button_inc;
        if (buttons[dn_button[i]]) val -= button_inc;
        if (val >  1.) val =  1.;        // obey range limit 
        if (val < -1.) val = -1.;
        // if (buttons[zero_button[i]]) val = 0.;  // reset to midpoint
        analogs[i] = val;   // update it as calculated
      }
      // Y and B buttons map to outbound switch fields (e.g. temp readout enable)
      switches[0] = buttons[YButton];
      switches[1] = buttons[BButton];
      
      if (0) {
        for (int i=0;i<NANALOG;i++) {
          Serial.printf(" %6.3f", analogs[i]);
        }
        Serial.println();
      }
    }
  }
}

// Half-scale factor: full-stick analog ±1.0 maps to ±128 around Pwm0 neutral
float motScale = 128.;

/*
 * Purpose: Clamp a mixed motor command to the signed 7-bit range before PWM packing.
 * Inputs:  val — signed command prior to offset by Pwm0.
 * Outputs: Returns val limited to [−127, +127].
 * System Impact: Prevents hex field overflow in build_command_msg() motor bytes.
 * Safety Notes: Saturation at full stick — operator may not perceive further deflection.
 * Usage Notes: Applied to every thruster and servo channel after gain mixing.
 */
int lims(int val) {
  int res = val;
  if (res > 127) res = 127;
  if (res < -127) res = -127;
  return res;
}

/*
 * Purpose: Mix normalized analogs[] into motors[] and servos[] using thruster gains and directions.
 * Inputs:  analogs[], motDirs[], DriveGainL/R, SteerGain, DiveGainL/R, StrafeGain, GripperGain.
 * Outputs: Fills motors[0..5] and servos[0..1] with Pwm0-centered PWM command bytes.
 * System Impact: Defines vehicle motion — left stick drive/strafe, right stick steer/dive,
 *                 triggers gripper; D-pad servos LED dim and camera tilt.
 * Safety Notes: GripperGain is intentionally low to protect the mechanism; verify motDirs
 *               if a thruster runs backward after maintenance.
 * Usage Notes: Xbox layout — LJoyY+L/R fore-aft mix, LJoyX strafe, RJoyX steer, RJoyY dive,
 *              RTrig−LTrig gripper; servos driven by synthesized DPadX/DPadY axes.
 */
void translate_controls_to_commands() {
  float LRForeAft = analogs[LJoyY]*DriveGainL + analogs[RJoyX]*SteerGain;
  float RRForeAft = analogs[LJoyY]*DriveGainR - analogs[RJoyX]*SteerGain;
  float Strafe    = analogs[LJoyX]*StrafeGain;
  float LUpDown   = analogs[RJoyY]*DiveGainL;
  float RUpDown   = analogs[RJoyY]*DiveGainR;
  // gripper uses both of the triggers
  float Gripper   = (analogs[RTrig] - analogs[LTrig])*GripperGain;

  // Apply per-motor direction, scale, clamp, and offset to neutral PWM
  motors[GripperMot] = Pwm0 + lims((int)(Gripper  *motDirs[GripperMot]*motScale));
  motors[LUpDownMot] = Pwm0 + lims((int)(LUpDown  *motDirs[LUpDownMot]*motScale));
  motors[RUpDownMot] = Pwm0 + lims((int)(RUpDown  *motDirs[RUpDownMot]*motScale));
  motors[LForAftMot] = Pwm0 + lims((int)(LRForeAft*motDirs[LForAftMot]*motScale));
  motors[RForAftMot] = Pwm0 + lims((int)(RRForeAft*motDirs[RForAftMot]*motScale));
  motors[StrafeMot ] = Pwm0 + lims((int)(Strafe   *motDirs[StrafeMot ]*motScale));

  // the servos use the D pad buttons for camera tilt and brightness
  servos[0] = Pwm0 + lims((int)(analogs[DPadX] * motScale));  // LED dimming
  servos[1] = Pwm0 + lims((int)(-analogs[DPadY] * motScale));  // camera tilt servo
}


/*
 * Purpose: Serialize motors[], servos[], and switches[] into an RS-485 hex-ASCII command frame.
 * Inputs:  Global motors[], servos[], switches[] populated by translate_controls_to_commands().
 * Outputs: Writes NUL-terminated string at msg: M<motors>P<servos>S<switches>C<checksum>\n.
 * System Impact: Outbound payload transmitted on Serial1 every 20 Hz from loop().
 * Safety Notes: Checksum is low-byte sum of all motor, servo, and switch values — vehicle
 *               should reject frames with mismatch to avoid runaway thrusters.
 * Usage Notes: Motor/servo fields are two hex digits each; switch fields one digit each.
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

// ----------- Show when USB devices are added or removed ------------------

/*
 * Purpose: Detect USB connect/disconnect events and log them to Serial and the LCD.
 * Inputs:  drivers[], hiddrivers[] state compared against driver_active[] snapshots.
 * Outputs: Serial diagnostics; LCD line 1 shows USB vendor:product ID on connect.
 * System Impact: Operator feedback during gamepad pairing — aids Xbox controller identification.
 * Safety Notes: ROV commands continue at Pwm0 neutral if no gamepad is connected.
 * Usage Notes: Invoked every loop() iteration after myusb.Task() services the host stack.
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
        // Mirror USB VID:PID on LCD row 1 for field identification
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
        // Mirror USB VID:PID on LCD row 1 for field identification
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
/* ----------------- init code -------------------- */

/*
 * Purpose: One-time hardware and software initialization before the 20 Hz main loop runs.
 * Inputs:  Compile-time pin defines, motDirs[] signs, and peripheral configuration tables.
 * Outputs: Configured Serial, Serial1 (RS-485), LCD, USB host; motors/servos at Pwm0 neutral.
 * System Impact: Establishes RS-485 link, LCD splash, USB host, and safe idle thruster commands.
 * Safety Notes: All motor and servo commands initialize to Pwm0 (neutral) before USB is ready.
 * Usage Notes: Lowers CPU clock if needed for LCD timing; shows splash then "Seeking Gamepad".
 */
void setup() {
  // Limit CPU speed when running above 100 MHz so the parallel LCD bus remains reliable
  if ( F_CPU_ACTUAL >= 100'000'000 )
    set_arm_clock(100'000'000);

// Load motDirs[] from compile-time direction defines before any commands are sent
motDirs[GripperMot] = GripperMotDir;  
motDirs[LUpDownMot] = LUpDownMotDir; 
motDirs[RUpDownMot] = RUpDownMotDir;
motDirs[LForAftMot] = LForAftMotDir;
motDirs[RForAftMot] = RForAftMotDir;
motDirs[StrafeMot]  = StrafeMotDir;

  // USB debug console on the Teensy programming port
  Serial.begin(115200);
  delay(100);
  Serial.println("Starting Teensy");

  // RS-485 vehicle link — 115200 baud with hardware TX-enable on SER_TXEN
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
// Start USB host stack for Xbox One controller enumeration
  myusb.begin();
// initialize all motors and servos to safe position
  for (int i=0;i<NMOTORS;i++) 
    motors[i] = Pwm0;
  for (int i=0;i<NSERVOS;i++) 
    servos[i] = Pwm0;
  inPtr = reply_msg;       // initialize the reply message receiver
  gotInString = false;
}

/* --------------------- main loop --------------------------- */

/*
 * Purpose: Run the continuous 20 Hz control cycle — USB, RS-485 TX/RX, and LCD telemetry.
 * Inputs:  USB gamepad (via myusb), inbound Serial1 bytes, micros() for loop pacing.
 * Outputs: Transmits command_msg on Serial1; updates LCD when a reply is complete.
 * System Impact: Heart of topside operation — ties Xbox mapping, thruster mixing, and telemetry.
 * Safety Notes: Commands transmit on schedule even if telemetry is missing; neutral Pwm0 at boot.
 * Usage Notes: Command path fires every loopPeriod µs; reply parsing and LCD refresh are event-driven.
 */
void loop() {    
 // unsigned long mics = micros();
 // Serial.printf("Mics %d\n", mics);
  myusb.Task();
  PrintDeviceListChanges();  // show USB connect/disconnect activity
  
  // Non-blocking RS-485 RX — accumulate bytes until newline completes the reply frame
  if (Serial1.available()) {
    int inChr = Serial1.read();
    if (inChr) *inPtr++ = inChr;   // store it if it's not null
// A newline marks end-of-frame; NUL-terminate and flag gotInString for parsing
    if (inChr == '\n') {
      *inPtr++ = 0x00;           // null terminate the input string
      gotInString = true;
      inPtr = reply_msg;       // tell the parser to do its work on this string
    }
  }
  // 20 Hz command cadence — read gamepad, mix thrusters, build and send RS-485 frame
  if (micros() > nextLoopMicros) {
    nextLoopMicros += loopPeriod;
    translate_response_to_controls();  // Read the gamepad into analogs, buttons
    translate_controls_to_commands();  // make ROV motion data from that
    build_command_msg(command_msg);
  //  Serial.print(command_msg);        // show commands to monitor
    Serial1.print(command_msg);        // give commands to ROV
  }
  // Event-driven telemetry path — parse reply, then refresh LCD readouts
  if (gotInString) {
    Serial.print(reply_msg);        // give commands to ROV
    gotInString = false;
    parse_reply_msg(reply_msg);    // update telemetry if it's valid
    display_all_telems();          // display telemetry on the LCD
  }
}


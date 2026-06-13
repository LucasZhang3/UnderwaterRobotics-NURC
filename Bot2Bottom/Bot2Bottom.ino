
/* 
Rovotron Cadet RVCBOT-D — bottom-side actuator and telemetry controller

Receives hex-ASCII command frames over RS-485 from the surface board,
drives thruster ESCs and auxiliary PWM outputs, and returns raw ADC
telemetry in a matching reply frame.

Target hardware: Teensy 4.0

Revision history

2023-04-04 DF  Teensy version, making it simpler to use Servo library
2023-04-09 DF  ready to try running, needs I2C code
2023-04-16 DF  Reversing motors, commenting out I2C code for now
2023-04-30 DF  Adding exponential control to motors
2023-05-07 DF  Not doing exponnential, instead adding 7 seconds delay at startup per Bluerobotics
2023-06-04 DF  Changing servo 4 to analogWrite becasue we need faster PWM to reduce LED flicker
2023-06-09 DF  Adding the MS5837 I2C pressure sensor code
2023-06-10 DF  Disabled depth sensor, as it's not behaving in top end

2023-07-31 DF  Added pin defines for RVCBOT-A pinout, Teensy 4.0 MPU, 6 motors
2023-08-01 DF  Swapped motor5 and servo2 for strafing motor
2023-08-16 DF  Checking camera servo and LED 
2023-08-26 DF  RVCBOT-B version has different pinout
2023-08-29 DF  Added ADC reads
2023-09-09 DF  Changing to talk to RVTOP-A board, different motor count etc.
2023-11-19 DF  RVCBOT-C version has only 6 motors, tilt, dim
2024-02-22 DF  Adding pressure to telem message

Actuator channels:
 5 thruster ESC outputs (motors 1–5)
 1 claw RC servo (motor index 0)
 1 LED dimmer PWM (servo index 0)
 Camera tilt output compile-time disabled (CAMERA_TILT_ENABLED = 0)

Telemetry ADC channels (raw counts in reply V-section):

 volts[0]  Battery voltage (VBATT)
 volts[1]  Depth analog
 volts[2]  (unused)
 volts[3]  LED temperature
 volts[4]  Water temperature
 volts[5]  Pressure sensor

ADC scale: 0x000 = 0.00 V, 0xFFF = 3.30 V (12-bit resolution).

Open items:
  Improve comm error recovery and add a bench test for bad frames.
*/

//------------ INCLUDES AND LIBRARIES ------------------------------- //

// Standard Arduino Servo library for ESC and RC-servo pulse outputs
#include <Servo.h> 

float depthMeters;  // reserved for computed depth in meters (not currently populated)

// ------------ Sizes of things --------------- //

// Array indices and field counts are zero-based to match protocol sections
#define NMOTORS 6		// motor command slots (index 0 = claw, 1–5 = thrusters)
#define NSERVOS 2   	// P-section fields from top-side (dim + camera tilt)
#define NSWITCHES 2		// S-section switch bits parsed into checksum only
#define NVOLTS 6		// telemetry ADC channels in the V-section reply


// Teensy pin definitions ------------------ //

// Serial1 on pins 0 (RX) and 1 (TX); TXE drives RS-485 transceiver direction
#define TXE_PIN      2

// Thruster ESC and auxiliary PWM outputs
#define MOTOR1_PIN   3
#define MOTOR2_PIN   4
#define MOTOR3_PIN   5
#define MOTOR4_PIN   6
#define MOTOR5_PIN   7
#define GRIP1_PIN    9
#define GRIP2_PIN   10
#define CAMERA_TILT_PIN 17  // camera tilt pin (inactive while CAMERA_TILT_ENABLED = 0)
#define CLAW_SERVO_PIN  21  // 5 V claw RC servo on dedicated pin
#define DIM_PIN     22

// Claw servo pulse limits and slew-rate governor (reduces twitch and brownout risk)
#define CLAW_SERVO_US_MIN 1000
#define CLAW_SERVO_US_MAX 2000
#define CLAW_UPDATE_MS 20
#define CLAW_MAX_US_STEP 80

// Camera tilt: set CAMERA_TILT_ENABLED to 0 while hardware is unplugged
#define CAMERA_TILT_ENABLED 0
#define CAMERA_TILT_ANALOG 1
#define CAMERA_TILT_PWM_HZ 50

// I2C bus (reserved for future sensors)
#define SDA0_PIN    18
#define SCL0_PIN    19

// SPI bus pins (reserved)
#define MOSI_PIN    11
#define MISO_PIN    12
#define SCK_PIN     13

// Analog telemetry inputs
#define VBATT_PIN   A9 
#define ANALOG1_PIN A3
#define ANALOG2_PIN A2
#define ANALOG3_PIN A1
#define ANALOG4_PIN A0
#define PRESSURE_PIN  A6

#define Pwm0 128  // protocol neutral value (maps to 1500 µs / mid PWM)
#define BOTTOM_DEBUG_SERIAL 0

//------------ GLOBAL VARIABLES AND RUNTIME STATE ------------------------------- //

// Latest command values decoded from the surface packet
int motors[NMOTORS];		// M-section: motors[0]=claw, motors[1..5]=thrusters
int servos[NSERVOS];		// P-section: servos[0]=dim, servos[1]=camera tilt
int volts[NVOLTS];					// smoothed raw ADC readings for telemetry

// RS-485 message buffers
char command_msg[300];	// inbound command frame, null-terminated after newline
char reply_msg[300];		// outbound telemetry frame
char buf[300];			// scratch string buffer

// Servo objects — pins are assigned in setup()
Servo motor1;  
Servo motor2;  
Servo motor3;  
Servo motor4;  
Servo motor5;  

Servo clawServo;

// Serial1 receive accumulator
char inString[100];          // legacy receive buffer (unused by current loop)
char inChr = 0;              // last received character placeholder
char * inPtr;
bool gotInString = false;    // true when a complete newline-delimited frame arrived

int msg_count = 0;   // count of successfully parsed command frames
bool escStartupDone = false;
unsigned long lastGoodPacketMs = 0;
int lastParseStatus = -1;

// ---------------------- SENSOR READINGS --------------------------- //

const int smoothRange  = 100;      // denominator for exponential smoothing blend
const int smooth    = 80;      // weight of previous sample (higher = smoother, slower)

/*
Purpose:
  Refresh all bottom-side analog telemetry channels with exponentially smoothed ADC counts.
Inputs:
  Six physical analog pins: battery, depth, spare, LED temp, water temp, pressure.
Outputs:
  Updates global volts[0..NVOLTS-1] with filtered 12-bit raw counts.
System Impact:
  Runs every loop iteration; increases loop time slightly via six analogRead() calls.
Safety Notes:
  Read-only — does not drive any actuator.
Usage Notes:
  Requires analogReadResolution(12) in setup(); values are sent upstream as-is in build_reply_msg().
*/
void read_all_adcs(void) {
  volts[0] = (volts[0]*smooth+analogRead(VBATT_PIN)*(smoothRange-smooth))/smoothRange;  
  volts[1] = (volts[1]*smooth+analogRead(ANALOG1_PIN)*(smoothRange-smooth))/smoothRange;  
  volts[2] = (volts[2]*smooth+analogRead(ANALOG2_PIN)*(smoothRange-smooth))/smoothRange;  
  volts[3] = (volts[3]*smooth+analogRead(ANALOG3_PIN)*(smoothRange-smooth))/smoothRange;  
  volts[4] = (volts[4]*smooth+analogRead(ANALOG4_PIN)*(smoothRange-smooth))/smoothRange;  
  volts[5] = (volts[5]*smooth+analogRead(PRESSURE_PIN)*(smoothRange-smooth))/smoothRange;  
  return;
}

// -------------------- COMMUNICATION PARSING HELPERS ---------------------- //

/*
Purpose:
  Decode a single ASCII hex digit into its 4-bit numeric value.
Inputs:
  chr — one character from an inbound packet field.
Outputs:
  Returns 0–15 for valid hex digits; returns -1 if the character is not hex.
System Impact:
  None — pure function with no side effects.
Safety Notes:
  Callers must treat -1 as a hard parse failure.
Usage Notes:
  Called repeatedly by atox() while walking a fixed-width hex field.
*/
char atoxdigit(char chr) {
	if (('a' <= chr) && (chr <= 'f')) return (chr - 'a' + 10);
	if (('A' <= chr) && (chr <= 'F')) return (chr - 'A' + 10);
	if (('0' <= chr) && (chr <= '9')) return (chr - '0');
	return -1;
}

/*
Purpose:
  Read a fixed-length unsigned hexadecimal integer from a character buffer.
Inputs:
  str — pointer to the first hex digit; n — number of digits to consume.
Outputs:
  Parsed integer value, or -4 if any digit fails atoxdigit() validation.
System Impact:
  None — advances are the caller's responsibility after return.
Safety Notes:
  No bounds checking on str length; caller must guarantee n valid characters exist.
Usage Notes:
  Typical field widths are 2 digits (motors, servos, checksum) or 1 digit (S-section switches).
*/
int atox(char *str, char n) {
	char digit;
	int i, val;
	val = 0;
	for (i = 0; i < n; i++) {
		if ((digit = atoxdigit(*str++)) == -1) return -4;
		val = (val << 4) + digit;
	}
	return val;
}

// -------------------- COMMAND PACKET PARSING ------------------------ //

/*
Purpose:
  Validate and decode one surface-to-bottom command frame into motor and servo arrays.
Inputs:
  msg — null-terminated buffer starting with 'M'; mots/sers — output arrays sized NMOTORS/NSERVOS.
Outputs:
  Returns 0 on success (arrays populated); returns 1 on any format, checksum, or terminator error.
System Impact:
  On success, downstream applyActuatorCommands() and applyClawOutput() consume the decoded values.
  On failure, actuator arrays are left unchanged and outputs hold their previous state.
Safety Notes:
  Checksum validation is active: the running sum of all M, P, and S field bytes (mod 256) must
  match the 2-digit hex value after the 'C' marker, or the entire frame is rejected.
  S-section parsing reads NSWITCHES single-hex-digit switch states, adds each to the checksum,
  but does not store them locally — they affect validation only.
Usage Notes:
  Expected frame layout: M(2*NMOTORS) P(2*NSERVOS) S(NSWITCHES) C(2) newline null.
  Reply is always sent before parse result is checked; a bad frame still triggers telemetry TX.
*/
int parse_command_msg(char *msg, int *mots, int *sers) {
	int i, checksum = 0;
	int val;

	if (*msg++ != 'M') return 1;

	for (i=0;i<NMOTORS;i++) {
		if ((val = atox(msg, 2)) < 0) return 1;
		msg += 2;
		checksum += mots[i] = val;
	}

	if (*msg++ != 'P') return 1;
	for (i=0;i<NSERVOS;i++) {
		if ((val = atox(msg, 2)) < 0) return 1;
		msg += 2;
		checksum += sers[i] =  val;
	}

	if (*msg++ != 'S') return 1;
	for (i=0;i<NSWITCHES;i++) {
		if ((val = atox(msg, 1)) < 0) return 1;
		msg += 1;
		checksum += val;
	}

	if (*msg++ != 'C') return 1;
	if ((val = atox(msg, 2)) < 0) return 1;
	msg += 2;
	if ((checksum % 0x100) != (unsigned char)val) return 1;
	if (*msg++ != '\n') return 1; 
	if (*msg++ != '\0') return 1; 
	return 0;
}

// -------------------- TELEMETRY PACKAGING ------------------------ //

/*
Purpose:
  Assemble the bottom-to-surface telemetry reply string from the latest ADC readings.
Inputs:
  Global volts[] filled by read_all_adcs().
Outputs:
  Writes a 'V'-prefixed, newline-terminated hex string into msg.
System Impact:
  Transmitted on Serial1 immediately after each received command newline.
Safety Notes:
  Outbound telemetry carries no checksum; integrity relies on the half-duplex request-reply cadence.
Usage Notes:
  Each channel is encoded as three uppercase hex digits (12-bit raw count); NVOLTS fields total.
*/
void build_reply_msg(char *msg) {
	int i;

	*msg++ = 'V';
	for (i=0;i<NVOLTS;i++) {
		msg += sprintf(msg, "%03X", volts[i]);
	}

  *msg++ = '\n'; 
	*msg = '\0';
}

/*
Purpose:
  Convert an 8-bit thruster command byte into ESC-compatible servo pulse width.
Inputs:
  cmd — protocol value where 128 is neutral stop.
Outputs:
  Pulse width in microseconds for Servo.writeMicroseconds() (1000–2000 µs span).
System Impact:
  Applied to motors[1..5] inside applyActuatorCommands() after a valid parse.
Safety Notes:
  Values outside the mapped range still produce bounded µs output via linear scaling.
Usage Notes:
  Mapping: cmd 0 → 1900 µs, cmd 128 → 1500 µs, cmd 255 → 1100 µs (reverse of forward convention).
*/
int motor_scale(int cmd) {
  return (((cmd - 128) * 1000) / 256) + 1500;
} 

/*
Purpose:
  Convert an 8-bit servo command byte into standard RC pulse width.
Inputs:
  cmd — protocol value where 128 is center position.
Outputs:
  Pulse width in microseconds (700–2300 µs span).
System Impact:
  Used by camera tilt path (when enabled) and as the base scale for claw_servo_scale().
Safety Notes:
  Wider span than thrusters; ensure attached servo hardware tolerates 700–2300 µs.
Usage Notes:
  cmd 128 → 1500 µs center; endpoints map linearly across the full 0–255 command range.
*/
int servo_scale(int cmd) {
  return (((cmd - 128) * 1600) / 256) + 1500;
}

/*
Purpose:
  Map a claw command byte to a clamped RC pulse within safe mechanical and electrical limits.
Inputs:
  cmd — 8-bit claw position request (same encoding as servo_scale).
Outputs:
  Microsecond pulse clamped to CLAW_SERVO_US_MIN..CLAW_SERVO_US_MAX.
System Impact:
  Feeds initClawOutput(), applyClawOutput(), and applySafeActuatorNeutral().
Safety Notes:
  Clamping prevents commanding the claw servo beyond 1000–2000 µs regardless of upstream value.
Usage Notes:
  Delegates initial scaling to servo_scale(), then applies hard min/max bounds.
*/
int claw_servo_scale(int cmd) {
  int us = servo_scale(cmd);
  if (us < CLAW_SERVO_US_MIN) us = CLAW_SERVO_US_MIN;
  if (us > CLAW_SERVO_US_MAX) us = CLAW_SERVO_US_MAX;
  return us;
}

#if CAMERA_TILT_ENABLED && !CAMERA_TILT_ANALOG
Servo cameraServo;
#endif

static int clawCurrentUs = 1500;

/*
Purpose:
  Write the claw servo output and mirror the commanded pulse in local state.
Inputs:
  pulseUs — target pulse width in microseconds.
Outputs:
  None (void); updates clawCurrentUs and drives clawServo hardware.
System Impact:
  Immediate hardware write — no slew limiting; used by init and slew steps.
Safety Notes:
  Caller is responsible for supplying a value already within CLAW_SERVO_US_MIN/MAX if required.
Usage Notes:
  Prefer applyClawOutput() during normal operation; use this for init and single-step slew updates.
*/
void writeClawServoUs(int pulseUs) {
  clawCurrentUs = pulseUs;
  clawServo.writeMicroseconds(pulseUs);
}

/*
Purpose:
  Attach the claw servo pin and drive it to the neutral protocol position at boot.
Inputs:
  None.
Outputs:
  None (void); clawServo attached and set to claw_servo_scale(Pwm0).
System Impact:
  Called once from setup() after the ESC startup sequence completes.
Safety Notes:
  Ensures claw starts at a known center pulse before the main loop runs.
Usage Notes:
  Runs before applySafeActuatorNeutral(); both converge on the same neutral scale value.
*/
void initClawOutput(void) {
  clawServo.attach(CLAW_SERVO_PIN);
  writeClawServoUs(claw_servo_scale(Pwm0));
}

/*
Purpose:
  Rate-limit claw motion toward the latest motors[0] command using timed microsecond steps.
Inputs:
  cmd — current claw command byte (typically motors[0] from the last valid or stale parse).
Outputs:
  None (void); may increment claw position by up to CLAW_MAX_US_STEP µs per eligible pass.
System Impact:
  Invoked unconditionally at the end of every loop() iteration — independent of parse success.
  Claw slew continues toward the last motors[0] value even when no new packet arrived.
Safety Notes:
  Slew cap (CLAW_MAX_US_STEP every CLAW_UPDATE_MS) limits inrush current and mechanical jerk.
  Does not auto-return to neutral on comm loss; last commanded claw position is retained.
Usage Notes:
  Early-return if less than CLAW_UPDATE_MS elapsed since last step; no write when delta is zero.
*/
void applyClawOutput(int cmd) {
  static unsigned long lastMs = 0;
  unsigned long now = millis();
  if (now - lastMs < CLAW_UPDATE_MS) return;
  lastMs = now;

  int targetUs = claw_servo_scale(cmd);
  int delta = targetUs - clawCurrentUs;
  if (delta > CLAW_MAX_US_STEP) delta = CLAW_MAX_US_STEP;
  if (delta < -CLAW_MAX_US_STEP) delta = -CLAW_MAX_US_STEP;
  if (delta != 0) writeClawServoUs(clawCurrentUs + delta);
}

/*
Purpose:
  Configure the camera tilt output pin when compile-time camera support is enabled.
Inputs:
  None.
Outputs:
  None (void); no-op when CAMERA_TILT_ENABLED is 0 (current build — camera unplugged).
System Impact:
  When enabled, selects analogWrite PWM or Servo mode based on CAMERA_TILT_ANALOG.
Safety Notes:
  Disabled configuration prevents accidental PWM on an unconnected pin 17.
Usage Notes:
  Called from setup(); pair with applyCameraTiltOutput() for runtime updates when re-enabled.
*/
void initCameraTiltOutput(void) {
#if CAMERA_TILT_ENABLED
#if CAMERA_TILT_ANALOG
  pinMode(CAMERA_TILT_PIN, OUTPUT);
  analogWriteFrequency(CAMERA_TILT_PIN, CAMERA_TILT_PWM_HZ);
  analogWrite(CAMERA_TILT_PIN, Pwm0);
#else
  cameraServo.attach(CAMERA_TILT_PIN);
  cameraServo.writeMicroseconds(servo_scale(Pwm0));
#endif
#endif
}

/*
Purpose:
  Apply a camera tilt command to the tilt output when camera hardware is enabled.
Inputs:
  cmd — 8-bit tilt value from servos[1] (raw PWM byte or scaled servo pulse depending on mode).
Outputs:
  None (void); no-op while CAMERA_TILT_ENABLED is 0.
System Impact:
  Called from applyActuatorCommands() only on servos[1] change when camera is enabled.
Safety Notes:
  Compile-time guard ensures no output is driven while camera is physically disconnected.
Usage Notes:
  In analog mode writes cmd directly; in servo mode applies servo_scale() first.
*/
void applyCameraTiltOutput(int cmd) {
#if CAMERA_TILT_ENABLED
#if CAMERA_TILT_ANALOG
  analogWrite(CAMERA_TILT_PIN, cmd);
#else
  cameraServo.writeMicroseconds(servo_scale(cmd));
#endif
#endif
}

/*
Purpose:
  Force claw, optional camera tilt, and LED dim outputs to their neutral boot values.
Inputs:
  None.
Outputs:
  None (void); claw at scaled neutral, dim PWM at Pwm0, camera neutral if enabled.
System Impact:
  Called once at end of setup() to establish a safe post-ESC-startup actuator baseline.
Safety Notes:
  Does not reset thruster ESCs — those are handled by stop_all_motors() in the startup sequence.
Usage Notes:
  Useful reference for expected neutral state before the first valid command frame arrives.
*/
void applySafeActuatorNeutral(void) {
  writeClawServoUs(claw_servo_scale(Pwm0));
#if CAMERA_TILT_ENABLED
  applyCameraTiltOutput(Pwm0);
#endif
  analogWrite(DIM_PIN, Pwm0);
}


/*
Purpose:
  Briefly command all thruster ESCs to a forward arming pulse during the startup ritual.
Inputs:
  None.
Outputs:
  Writes 1600 µs to motors 1–5.
System Impact:
  Part of runEscStartupSequence(); thrusters spin briefly if props are installed.
Safety Notes:
  Only invoked during the one-time ESC initialization window in setup(), not during normal loop.
Usage Notes:
  Follows a 7 s neutral hold and precedes a 1 s forward pulse before returning to neutral.
*/
void start_all_motors(void) {
  motor1.writeMicroseconds(1600);  
  motor2.writeMicroseconds(1600);  
  motor3.writeMicroseconds(1600);  
  motor4.writeMicroseconds(1600);  
  motor5.writeMicroseconds(1600);  
}

/*
Purpose:
  Command all thruster ESC outputs to the standard 1500 µs neutral/stop pulse.
Inputs:
  None.
Outputs:
  Writes 1500 µs to motors 1–5.
System Impact:
  Used at ESC startup boundaries and as the resting state between arming pulses.
Safety Notes:
  Immediate stop command — safe to call whenever thrusters must be disarmed.
Usage Notes:
  Does not affect claw, dim, or camera outputs.
*/
void stop_all_motors(void) {
  motor1.writeMicroseconds(1500);  
  motor2.writeMicroseconds(1500);  
  motor3.writeMicroseconds(1500);  
  motor4.writeMicroseconds(1500);  
  motor5.writeMicroseconds(1500);  
}

/*
Purpose:
  Execute the Blue Robotics–style ESC initialization timing sequence exactly once.
Inputs:
  None.
Outputs:
  None (void); sets escStartupDone true after completion.
System Impact:
  Blocks setup() for ~8 s total (7 s neutral + 1 s forward pulse) on first call.
Safety Notes:
  Props should be clear during this window — forward pulse can spin thrusters briefly.
Usage Notes:
  Subsequent calls return immediately; sequence order is stop → 7 s delay → start → 1 s delay → stop.
*/
void runEscStartupSequence(void) {
  if (escStartupDone) return;
  stop_all_motors();
  delay(7000);
  start_all_motors();
  delay(1000);
  stop_all_motors();
  escStartupDone = true;
#if BOTTOM_DEBUG_SERIAL
  Serial.println("ESC startup done");
#endif
}

/*
Purpose:
  Push decoded thruster, dim, and optional camera commands to hardware after a valid parse.
Inputs:
  Implicit globals motors[] and servos[] populated by parse_command_msg().
Outputs:
  None (void); updates ESC pulses, DIM_PIN PWM, and camera tilt when enabled.
System Impact:
  Called only when parse_command_msg() returns 0; claw is handled separately in loop().
Safety Notes:
  motors[0] (claw) is intentionally excluded here — applyClawOutput() owns claw writes every loop.
Usage Notes:
  Camera tilt update is change-detected on servos[1]; dim uses servos[0] as raw analogWrite value.
*/
void applyActuatorCommands(void) {
  motor1.writeMicroseconds(motor_scale(motors[1]));
  motor2.writeMicroseconds(motor_scale(motors[2]));
  motor3.writeMicroseconds(motor_scale(motors[3]));
  motor4.writeMicroseconds(motor_scale(motors[4]));
  motor5.writeMicroseconds(motor_scale(motors[5]));
#if CAMERA_TILT_ENABLED
  static int lastCameraCmd = -1;
  if (servos[1] != lastCameraCmd) {
    applyCameraTiltOutput(servos[1]);
    lastCameraCmd = servos[1];
  }
#endif
  analogWrite(DIM_PIN, servos[0]);
}

/* ----------------- init code -------------------- */

/*
Purpose:
  One-time hardware and protocol initialization before entering the main control loop.
Inputs:
  None.
Outputs:
  None (void); configures serial, ADC, servos, ESC startup, and neutral actuator state.
System Impact:
  Blocks ~8 s during runEscStartupSequence(); afterward loop() handles all runtime I/O.
Safety Notes:
  ESC startup must complete before thrusters accept normal commands; claw init follows the sequence.
Usage Notes:
  Serial @ 115200 for debug; Serial1 @ 115200 with TXE for RS-485 half-duplex to surface board.
*/
void setup() {
  Serial.begin(115200);            // USB debug console
  Serial1.begin(115200);             // RS-485 tether link to surface
  Serial1.transmitterEnable(TXE_PIN);   // assert TX driver during Serial1 transmit
  pinMode(DIM_PIN, OUTPUT);
  analogReadResolution(12);

  motor1.attach(MOTOR1_PIN);  
  motor2.attach(MOTOR2_PIN);  
  motor3.attach(MOTOR3_PIN);  
  motor4.attach(MOTOR4_PIN);  
  motor5.attach(MOTOR5_PIN);  
  initCameraTiltOutput();

  for (int i = 0; i < NMOTORS; i++) motors[i] = Pwm0;
  for (int i = 0; i < NSERVOS; i++) servos[i] = Pwm0;

  runEscStartupSequence();
  initClawOutput();
  applySafeActuatorNeutral();
  inPtr = command_msg;
  gotInString = false;
#if BOTTOM_DEBUG_SERIAL
  Serial.println("TRASHBACK ready");
#endif
}

/*
Purpose:
  Continuous bottom-side control cycle: sample sensors, receive commands, reply, actuate.
Inputs:
  Incoming Serial1 bytes; analog pin voltages.
Outputs:
  Telemetry reply on Serial1; thruster/dim/camera updates on valid parse; claw slew every pass.
System Impact:
  read_all_adcs() runs first each iteration for rolling telemetry averages.
  applyClawOutput(motors[0]) runs unconditionally after packet handling — claw slew is loop-rate,
  not packet-rate, and proceeds toward the current motors[0] even between valid frames.
Safety Notes:
  Failed parse rejects actuator updates for thrusters/dim/camera but does not reset claw slew target.
  Buffer overflow on receive resets the accumulator pointer without flagging an error upstream.
Usage Notes:
  Pipeline per complete newline: build_reply_msg → Serial1.print → parse → applyActuatorCommands (if OK).
*/
void loop() {		
	char st = 0;
  
  read_all_adcs();      // keep telemetry filters warm with frequent samples

  // accumulate Serial1 bytes until newline completes a command frame
  if (Serial1.available()) {
    int inChr = Serial1.read();
    if (inChr) {
      if (inPtr < command_msg + sizeof(command_msg) - 2) {
        *inPtr++ = inChr;
      } else {
        inPtr = command_msg;
      }
    }
    if (inChr == '\n') {
      *inPtr++ = 0x00;
      gotInString = true;
      inPtr = command_msg;
    }
  }
	if (gotInString) {
    gotInString = false;
    build_reply_msg(reply_msg);
    Serial1.print(reply_msg);

    st = parse_command_msg(command_msg, motors, servos);
    lastParseStatus = st;
    if (!st) {
      applyActuatorCommands();
      lastGoodPacketMs = millis();
      msg_count++;
#if BOTTOM_DEBUG_SERIAL
      if ((msg_count % 20) == 0) {
        Serial.printf("OK #%d M1=%02X claw=%02X\n", msg_count, motors[1], motors[0]);
      }
#endif
    } else {
#if BOTTOM_DEBUG_SERIAL
      Serial.print("PARSE FAIL: ");
      Serial.println(command_msg);
#endif
    }
	}
  applyClawOutput(motors[0]);
}

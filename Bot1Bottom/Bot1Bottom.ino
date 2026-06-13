/*
 * Rovotron Cadet RVCBOT-D — bottom-board receiver firmware
 *
 * Drives thrusters, gripper PWM, camera servo, and LED dimming from hex-ASCII
 * commands over RS-485; replies with smoothed ADC telemetry.
 * Target: Teensy 4.0 (6 motor channels, 2 servo channels).
 *
 * Revision history
 * 2023-04-04 DF  Teensy port; Servo library for PWM outputs
 * 2023-04-09 DF  First run build; I2C hooks pending
 * 2023-04-16 DF  Motor direction fixes; I2C disabled temporarily
 * 2023-04-30 DF  Exponential motor curve removed
 * 2023-05-07 DF  Seven-second ESC startup delay (Blue Robotics practice)
 * 2023-06-04 DF  Servo 4 moved to analogWrite for faster LED PWM
 * 2023-06-09 DF  MS5837 I2C pressure sensor support added
 * 2023-06-10 DF  Depth sensor disabled (top-end instability)
 * 2023-07-31 DF  RVCBOT-A pin map; Teensy 4.0 MPU; six motors
 * 2023-08-01 DF  motor5 and servo2 swapped for strafe motor
 * 2023-08-16 DF  Camera servo and LED checks
 * 2023-08-26 DF  RVCBOT-B alternate pinout
 * 2023-08-29 DF  ADC read path added
 * 2023-09-09 DF  RVTOP-A link; revised motor count
 * 2023-11-19 DF  RVCBOT-C: six motors, tilt, dim
 * 2024-02-22 DF  Pressure channel in telemetry frame
 *
 * Actuators: 6 PWM thrusters, 2 PWM servos (gripper uses dual complementary PWM).
 * Telemetry: raw 12-bit ADC counts in volts[] (0x000 = 0.00 V, 0xFFF = 3.30 V).
 */

//------------ INCLUDES ------------------------------- //

#include <Servo.h>   // Hardware PWM for ESC and servo outputs

float depthMeters;  // Reserved for depth in meters (sensor path currently unused)

// ------------ Sizes of things --------------- //

#define NMOTORS 6   // Motor command slots in the inbound frame (index 0 = gripper)
#define NSERVOS 2   // Servo command slots (index 0 = dim, index 1 = camera tilt)
#define NVOLTS 6    // ADC channels packed into the outbound telemetry frame


// Teensy pin definitions ------------------ //

#define TXE_PIN      2   // RS-485 driver transmit-enable

#define MOTOR1_PIN   3
#define MOTOR2_PIN   4
#define MOTOR3_PIN   5
#define MOTOR4_PIN   6
#define MOTOR5_PIN   7
#define GRIP1_PIN    9   // Gripper coil A (outward drive)
#define GRIP2_PIN   10   // Gripper coil B (inward drive)
#define SERVO1_PIN  21
#define DIM_PIN     22   // LED brightness PWM

#define SDA0_PIN    18   // I2C data (reserved)
#define SCL0_PIN    19   // I2C clock (reserved)

#define MOSI_PIN    11   // SPI MOSI (reserved)
#define MISO_PIN    12
#define SCK_PIN     13

#define VBATT_PIN   A9   // volts[0] — battery
#define ANALOG1_PIN A3   // volts[1] — depth analog
#define ANALOG2_PIN A2   // volts[2] — spare
#define ANALOG3_PIN A1   // volts[3] — LED temperature
#define ANALOG4_PIN A0   // volts[4] — water temperature
#define PRESSURE_PIN  A6 // volts[5] — pressure transducer


//------------ Global Variables ------------------------------- //

int motors[NMOTORS];   // Parsed motor bytes from surface (motors[0] = gripper command)
int servos[NSERVOS];   // Parsed servo bytes (servos[0] = dim, servos[1] = tilt)
int volts[NVOLTS];     // Smoothed ADC samples for telemetry reply

int msg_count = 0;     // Debug counter of successfully handled command frames


char command_msg[300]; // Inbound RS-485 line buffer (newline + null terminated)
char reply_msg[300];   // Outbound telemetry line buffer
char buf[300];         // Scratch string (unused in current build)

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;
Servo motor5;

Servo servo1;

char inString[100];          // Legacy receive buffer (superseded by command_msg)
char inChr = 0;
char * inPtr;
bool gotInString = false;    // Set when a full newline-terminated frame arrives

int grip, grip1, grip2 = 0;  // Gripper signed offset and complementary PWM duties

// ---------------------- ADC read --------------------------- //

const int smoothRange  = 100;   // Denominator for exponential moving average
const int smooth    = 80;       // Weight of prior sample (higher = smoother, slower)

/*
 * Purpose
 *   Sample all telemetry ADC channels and apply exponential smoothing into volts[].
 *
 * Inputs
 *   None (reads hardware pins VBATT_PIN through PRESSURE_PIN; uses prior volts[]).
 *
 * Outputs
 *   Updates global volts[0..NVOLTS-1]; no return value.
 *
 * System Impact
 *   Runs every loop iteration; sets telemetry content for the next build_reply_msg().
 *
 * Safety Notes
 *   Smoothed values lag step changes; do not use volts[] for fast fault detection.
 *
 * Usage Notes
 *   Called from loop() before framing replies so telemetry reflects recent averages.
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

// -------------------- Parsing routines ---------------------- //

/*
 * Purpose
 *   Decode one ASCII hex digit to a 4-bit numeric value.
 *
 * Inputs
 *   chr — single character from the command string.
 *
 * Outputs
 *   0–15 for valid hex digits; -1 if the character is not hexadecimal.
 *
 * System Impact
 *   Used by atox(); invalid input propagates as a parse failure upstream.
 *
 * Safety Notes
 *   Returns -1 rather than guessing; callers must treat -1 as a hard reject.
 *
 * Usage Notes
 *   Accepts both upper- and lower-case A–F.
 */
char atoxdigit(char chr) {
	if (('a' <= chr) && (chr <= 'f')) return (chr - 'a' + 10);
	if (('A' <= chr) && (chr <= 'F')) return (chr - 'A' + 10);
	if (('0' <= chr) && (chr <= '9')) return (chr - '0');
	return -1;
}

/*
 * Purpose
 *   Parse exactly n consecutive hex digits from a string into an integer.
 *
 * Inputs
 *   str — pointer to the first digit (advanced by caller after success).
 *   n   — digit count to consume (typically 2 per motor/servo byte).
 *
 * Outputs
 *   Non-negative parsed value, or -4 if any digit fails atoxdigit().
 *
 * System Impact
 *   Feeds parse_command_msg(); a -4 (or legacy -1 check) aborts command application.
 *
 * Safety Notes
 *   Does not bounds-check the parsed value against actuator limits.
 *
 * Usage Notes
 *   Caller must advance str by n bytes only after a non-negative return.
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

// -------------------- Command parser ------------------------ //

/*
 * Purpose
 *   Validate and decode an inbound ROV command frame into motor and servo arrays.
 *
 * Inputs
 *   msg  — null-terminated string: 'M' + NMOTORS×2 hex + 'P' + NSERVOS×2 hex + 'C' + 2 hex + '\n' + '\0'.
 *   mots — destination for NMOTORS byte values (written on success).
 *   sers — destination for NSERVOS byte values (written on success).
 *
 * Outputs
 *   0 if framing and hex fields are valid; 1 on any format or digit error.
 *
 * System Impact
 *   On success, mots/sers drive thrusters, gripper, dim, and tilt in loop().
 *   Checksum byte is read but not verified (see inline note in function body).
 *
 * Safety Notes
 *   Checksum validation is intentionally disabled; corrupted frames may still apply.
 *   Return 1 must prevent actuator updates (enforced in loop()).
 *
 * Usage Notes
 *   Frame layout mirrors the surface transmitter; motors[0] is gripper, not a thruster.
 */
int parse_command_msg(char *msg, int *mots, int *sers) {
	int i, checksum = 0;
	int val;

	if (*msg++ != 'M') return 1;

	for (i=0;i<NMOTORS;i++) {
		if ((val = atox(msg, 2)) == -1) return 1;
		msg += 2;
		checksum += mots[i] = val;
	}

	if (*msg++ != 'P') return 1;
	for (i=0;i<NSERVOS;i++) {
		if ((val = atox(msg, 2)) == -1) return 1;
		msg += 2;
		checksum += sers[i] =  val;
	}

	if (*msg++ != 'C') return 1;
	if ((val = atox(msg, 2)) == -1) return 1;
	msg += 2;
	// Checksum compare disabled — surface and bottom must stay in sync if re-enabled
//	if (checksum != (unsigned char) val) return 1;
	if (*msg++ != '\n') return 1; 
	if (*msg++ != '\0') return 1; 
	return 0;
}

// -------------------- Build a reply ------------------------ //

/*
 * Purpose
 *   Assemble the telemetry reply string from smoothed ADC samples in volts[].
 *
 * Inputs
 *   msg — writable buffer large enough for 'V' + NVOLTS×3 hex digits + '\n' + '\0'.
 *
 * Outputs
 *   Null-terminated reply written to msg; no return value.
 *
 * System Impact
 *   Reply is transmitted on Serial1 before the next command is applied.
 *
 * Safety Notes
 *   No reply checksum; host cannot detect telemetry corruption on the wire.
 *
 * Usage Notes
 *   Each channel is three uppercase hex digits (12-bit ADC range 0x000–0xFFF).
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
 * Purpose
 *   Map a legacy 0x00–0xFF motor command byte to ESC pulse width in microseconds.
 *
 * Inputs
 *   cmd — motor command byte (0x80 ≈ neutral, 0x00 max reverse, 0xFF max forward).
 *
 * Outputs
 *   PWM pulse width in microseconds (approximately 1000–2000, neutral at 1500).
 *
 * System Impact
 *   Applied to motor1–motor5 via writeMicroseconds() after a valid parse.
 *
 * Safety Notes
 *   Values outside 0–255 are not clamped; invalid bytes can command extreme throttle.
 *
 * Usage Notes
 *   Linear map: cmd 0 → 1900 µs, 128 → 1500 µs, 255 → 1100 µs (see legacy curve).
 */
int motor_scale(int cmd) {
  return (((cmd - 128) * 1000) / 256) + 1500;
} 

/*
 * Purpose
 *   Map a legacy 0x00–0xFF servo command byte to standard servo pulse width.
 *
 * Inputs
 *   cmd — servo command byte (0x80 ≈ center).
 *
 * Outputs
 *   PWM pulse width in microseconds (approximately 700–2300, center at 1500).
 *
 * System Impact
 *   Applied to servo1 (camera tilt) via writeMicroseconds() after a valid parse.
 *
 * Safety Notes
 *   Wide pulse range can stress servos beyond mechanical stops if mis-calibrated.
 *
 * Usage Notes
 *   Wider span than motor_scale (1600 µs total travel vs 1000 µs).
 */
int servo_scale(int cmd) {
  return (((cmd - 128) * 1600) / 256) + 1500;
} 


/*
 * Purpose
 *   Briefly command all ESC outputs above neutral to satisfy arming sequence requirements.
 *
 * Inputs
 *   None (uses attached motor1–motor5 Servo objects).
 *
 * Outputs
 *   None; writes 1600 µs to each thruster channel.
 *
 * System Impact
 *   Part of setup() ESC wake-up; must be followed by stop_all_motors().
 *
 * Safety Notes
 *   Thrusters spin briefly during setup; ensure ROV is secured and props clear.
 *
 * Usage Notes
 *   Paired with delay(1000) then stop_all_motors() after the 7 s power-on wait.
 */
void start_all_motors(void) {
  motor1.writeMicroseconds(1600);  
  motor2.writeMicroseconds(1600);  
  motor3.writeMicroseconds(1600);  
  motor4.writeMicroseconds(1600);  
  motor5.writeMicroseconds(1600);  
}

/*
 * Purpose
 *   Set all thruster ESC outputs to neutral (stopped) pulse width.
 *
 * Inputs
 *   None (uses attached motor1–motor5 Servo objects).
 *
 * Outputs
 *   None; writes 1500 µs to each thruster channel.
 *
 * System Impact
 *   Safe idle state after setup and on parse/timeout failure paths in loop().
 *
 * Safety Notes
 *   Primary software stop; loss of serial link does not auto-invoke this (see loop st flag).
 *
 * Usage Notes
 *   Called at end of setup() and when st is non-zero in loop().
 */
void stop_all_motors(void) {
  motor1.writeMicroseconds(1500);  
  motor2.writeMicroseconds(1500);  
  motor3.writeMicroseconds(1500);  
  motor4.writeMicroseconds(1500);  
  motor5.writeMicroseconds(1500);  
}

/* ----------------- init code -------------------- */

/*
 * Purpose
 *   Initialize serial links, GPIO, ADC resolution, PWM attachments, and ESC startup.
 *
 * Inputs
 *   None (Arduino setup entry point).
 *
 * Outputs
 *   None; hardware left in neutral throttle with RS-485 receive armed.
 *
 * System Impact
 *   One-time configuration; 7 s delay blocks all command handling until complete.
 *
 * Safety Notes
 *   ESC arming sequence moves props; verify mechanical isolation before power-on.
 *   Gripper and dim pins are driven as outputs immediately.
 *
 * Usage Notes
 *   Serial @ 115200 for USB debug; Serial1 @ 115200 with TXE on RS-485 tether.
 */
void setup() {
  Serial.begin(115200);            // USB debug console
  Serial1.begin(115200);             // RS-485 tether to surface
  Serial1.transmitterEnable(TXE_PIN);   // Assert RS-485 driver only when transmitting
  pinMode(GRIP1_PIN, OUTPUT);        // Gripper half-bridge outputs
  pinMode(GRIP2_PIN, OUTPUT);
  pinMode(DIM_PIN, OUTPUT);        // LED dim PWM output
  analogReadResolution(12);        // Full 12-bit ADC for telemetry

  // Thruster and servo objects bound to physical pins (indices 1–5 are thrusters on the wire)
  motor1.attach(MOTOR1_PIN);  
  motor2.attach(MOTOR2_PIN);  
  motor3.attach(MOTOR3_PIN);  
  motor4.attach(MOTOR4_PIN);  
  motor5.attach(MOTOR5_PIN);  
  servo1.attach(SERVO1_PIN);  

  stop_all_motors();         // Hold neutral before ESC power-on timing
  delay(7000);               // Blue Robotics ESC initialization window
  start_all_motors();        // Momentary above-neutral pulse to arm ESCs
  delay(1000);               // Hold arming pulse
  stop_all_motors();         // Return to neutral after arming
  inPtr = command_msg;       // RS-485 accumulator starts at command buffer base
  gotInString = false;
}

/*
 * Purpose
 *   Main cycle: smooth ADCs, receive RS-485 commands, reply with telemetry, apply actuators.
 *
 * Inputs
 *   None (Arduino loop entry point; data arrives asynchronously on Serial1).
 *
 * Outputs
 *   None; continuous side effects on motors, servos, gripper PWM, and dim output.
 *
 * System Impact
 *   Telemetry is sent before parse/apply so the surface sees fresh ADC data each frame.
 *
 * Safety Notes
 *   st is hard-coded to 0 — timeout shutdown via stop_all_motors() is currently unreachable.
 *   Invalid parse results skip actuator updates; last good command persists.
 *
 * Usage Notes
 *   motors[1..5] map to motor1..motor5; motors[0] is gripper; servos[0] is dim, servos[1] is tilt.
 */
void loop() {		
	char st = 0;
  
  read_all_adcs();      // Refresh smoothed telemetry every iteration

  // Accumulate RS-485 bytes until newline completes a command frame
  if (Serial1.available()) {
    int inChr = Serial1.read();
    if (inChr) *inPtr++ = inChr;   // Append non-null characters
    if (inChr == '\n') { // Frame delimiter from surface controller
      *inPtr++ = 0x00;           // C-string terminate for parse_command_msg()
      gotInString = true;
      inPtr = command_msg;       // Reset write pointer for next frame
    }
  }
	if (gotInString) {
    Serial.println(command_msg);
    gotInString = false;
    
		build_reply_msg(reply_msg);	// Pack volts[] into outbound telemetry line
    Serial.println(reply_msg);
		st = 0;   // Timeout path disabled — always treat frame as received
		if (st) {
			stop_all_motors();		 // Would stop thrusters on comm timeout (inactive)
		}
		else {		// Valid receive path: reply first, then parse and apply
  //    Serial.print(reply_msg);   // USB echo of telemetry (disabled)
      Serial1.print(reply_msg);   // Telemetry to surface over RS-485
      Serial.println(msg_count);
      msg_count++;
      
			st = parse_command_msg(command_msg, motors, servos);
  //    Serial.printf("st=%d  M=%02X S=%02X\n", st, motors[3], servos[0]);   // Parse debug (disabled)
      
      
			if (!st) {			// Apply actuators only when frame structure checks pass
        // Wire indices 1–5 drive thrusters; index 0 is gripper-only
        motor1.writeMicroseconds(motor_scale(motors[1]));  
        motor2.writeMicroseconds(motor_scale(motors[2]));  
        motor3.writeMicroseconds(motor_scale(motors[3]));  
        motor4.writeMicroseconds(motor_scale(motors[4]));  
        motor5.writeMicroseconds(motor_scale(motors[5]));  
        servo1.writeMicroseconds(servo_scale(servos[1]));  
        grip = motors[0] - 128;
        if (grip < 0) {
          // Negative grip: energize GRIP2 (inward), hold GRIP1 at minimum duty
          grip2 = 1+grip*2;
          grip1 = 1;
        }
        else {
          // Positive grip: energize GRIP1 (outward), hold GRIP2 at minimum duty
          grip1 = 1-grip*2;
          grip2 = 1;
        }
        analogWrite(GRIP1_PIN, grip1);
        analogWrite(GRIP2_PIN, grip2);   // Complementary PWM — only one side slews with command
        analogWrite(DIM_PIN, servos[0]);
			}
		}
	}	
}

// Balance bot wheel controller for an ATmega328P board (Arduino Uno / Nano, 16 MHz, Arduino IDE board "Arduino Uno"
// or "Arduino Nano") driving two hoverboard hub motors through ZS-X11H BLDC drivers (36 V) with PWM throttle +
// DIR only. The drivers' BRAKE and STOP inputs are not used: tie them (or leave them) at the level that lets the
// motor run.
//
// Same serial protocol as the ESP32 version (arduino/esp32_legacy/balance_bot_esp32), so the ROS bridge, wheel_check and the
// policy work unchanged:
//
//   PC -> Arduino   "V <left> <right>\n"   wheel speed targets [rad/s], positive = drive the robot forward
//                   "S\n"                  stop: throttle 0 (wheels coast) until the next V / D
//                   "D <left> <right>\n"   open-loop test: raw duty in [-1, 1], no speed loop (wheels lifted!)
//                   "G <kp> <ki>\n"        set the velocity loop gains (kp in duty per rad/s, ki in duty per rad)
//                   "?\n"                  print the configuration, hall counts and hall errors
//   Arduino -> PC   "W <left> <right>\n"   measured wheel speeds [rad/s] at REPORT_HZ, positive = forward
//                   "# ...\n"              information lines (ignored by the ROS bridge, shown in its log)
//
// "left" and "right" are seen from behind the robot (front = where it drives with positive speed). "V" and "D"
// commands must keep coming (the ROS bridge sends 50 per second): after CMD_TIMEOUT_MS without one, the outputs
// switch off.
//
// Wiring (same pins on Uno / Nano and Arduino Mega 2560)
//
//   ZS-X11H signal header: PWM throttle, speed pulse out (SC), GND, GND, 5V out
//
//   Right driver   PWM throttle <- D9     SC -> D2     GND - Arduino GND     DIR <- D6     5V out: not connected
//   Left driver    PWM throttle <- D10    SC -> D3     GND - Arduino GND     DIR <- D7     5V out: not connected
//
//   * Throttle: D9 / D10 are the two outputs of Timer1, the only timer that gives a fast, 10-bit PWM. Connect them
//     straight to the drivers' PWM throttle inputs (duty 0-100 % = stop to full speed). PWM_SLOW selects 15.6 kHz or
//     2 kHz, see below.
//   * Leave the drivers' 5V outputs unconnected: the Arduino is powered over USB, and two 5 V supplies must not be
//     tied together. One GND per driver to the Arduino's GND is needed (the signals are measured against it).
//   * If the driver has an onboard speed potentiometer, turn it to minimum.
//   * Speed feedback, FEEDBACK_SPEED_PULSE (default): each driver's speed pulse output ("SC"), one wire per motor:
//         right SC -> D2, left SC -> D3       (the two external interrupt pins; 5 V signals connect directly)
//     It has no direction, so the sign comes from the commanded direction (worse for balancing near zero speed).
//     COUNTS_PER_REV must be the number of SC edges per wheel turn: measure it (first-time setup, step 1).
//     FEEDBACK_HALLS (better, if you can tap them): each motor's three hall wires in parallel with the driver:
//         left  hall A -> A0, B -> A1, C -> A2        right hall A -> A3, B -> A4, C -> A5
//
// First-time setup, wheels off the ground, Arduino Serial Monitor at 115200 baud, line ending "Newline":
//   1. Send "?", turn each wheel exactly one turn forward by hand, send "?" again. FEEDBACK_SPEED_PULSE: the count
//      change is the number of SC edges per turn: put it into COUNTS_PER_REV (the sign does not matter here, it
//      follows the commanded direction). FEEDBACK_HALLS: the count must change by about +90; negative: flip that
//      wheel's FEEDBACK_INVERT; hall errors that keep growing: a hall wire is swapped, missing, or noisy.
//   2. Send "D 0.1 0.1" a few times a second (or paste it repeatedly): both wheels must turn forward and "W" must be
//      positive. A wheel turning backward: flip its MOTOR_INVERT. Send "D 0.5 0.5": "W" x 2 is roughly
//      MOTOR_MAX_RAD_S. (The ROS tool `ros2 run balance_bot_policy wheel_check` measures all of this for you.)
//   3. Send "V 5 5" repeatedly, then "V -5 -5", then "V 20 20". The wheels should reach the speed within ~0.1 s,
//      without buzzing. If you change KP / KI, change HoverboardDriveCfg in the Isaac Lab package and retrain.

#include <Arduino.h>
#include <stdint.h>
#include <stdlib.h>

// ------------------------------------------------------------------------------------------------------------
// Configuration
// ------------------------------------------------------------------------------------------------------------

// Which motor is the robot's left one. The two wiring channels are fixed:
//   channel 1: throttle D9,  speed pulse D2, DIR D6        channel 2: throttle D10, speed pulse D3, DIR D7
// SWAP_LEFT_RIGHT 0: channel 1 = right wheel, channel 2 = left wheel (the wiring table above)
// SWAP_LEFT_RIGHT 1: channel 1 = left wheel,  channel 2 = right wheel
// Check (robot on a stand, wheel_test, turn left / "j" in teleop_twist_keyboard): seen from behind, the RIGHT wheel
// must turn forward and the LEFT one backward. The other way round: flip this. Forward driving looks the same
// either way, but a balancing policy with swapped wheels spins on the spot at full speed.
#define SWAP_LEFT_RIGHT 1

// Pins. DIR can be any free digital pin; the throttles must stay on D9 / D10 (see below).
#if SWAP_LEFT_RIGHT
const uint8_t RIGHT_DIR_PIN = 7;
const uint8_t LEFT_DIR_PIN = 6;
#else
const uint8_t RIGHT_DIR_PIN = 6;
const uint8_t LEFT_DIR_PIN = 7;
#endif
// Throttle: right = D9, left = D10 on both supported boards (fixed, see setupPwm()). The chip is detected when
// compiling (pick the board in the IDE / --fqbn):
//   ATmega328P (Uno / Nano):       D9 / D10 = Timer1 OC1A / OC1B, 10-bit PWM
//   ATmega2560 (Arduino Mega 2560): D9 / D10 = Timer2 OC2B / OC2A, 8-bit PWM, dithered to 10 bits (TIMER2_OVF)
#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
#define BOARD_MEGA 1
#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega168__)
#define BOARD_MEGA 0
#else
#error "unsupported board: pick Arduino Uno / Nano (ATmega328P) or Arduino Mega 2560"
#endif
#if SWAP_LEFT_RIGHT
const uint8_t RIGHT_PWM_PIN = 10;
const uint8_t LEFT_PWM_PIN = 9;
#else
const uint8_t RIGHT_PWM_PIN = 9;
const uint8_t LEFT_PWM_PIN = 10;
#endif

// Flip these so that a positive target drives the robot forward and is measured as positive (see setup above).
// The motors are mirrored on a two-wheeled robot, so usually exactly one of the two MOTOR_INVERTs is true.
// FEEDBACK_INVERT only applies to FEEDBACK_HALLS (speed pulses take their sign from the commanded direction).
// These belong to the wheel (left / right after SWAP_LEFT_RIGHT): the motor on channel 1 (D9) needs inverting.
const bool LEFT_MOTOR_INVERT = SWAP_LEFT_RIGHT ? true : false;
const bool LEFT_FEEDBACK_INVERT = SWAP_LEFT_RIGHT ? true : false;
const bool RIGHT_MOTOR_INVERT = SWAP_LEFT_RIGHT ? false : true;
const bool RIGHT_FEEDBACK_INVERT = SWAP_LEFT_RIGHT ? false : true;

// Speed feedback:
//   FEEDBACK_SPEED_PULSE : the drivers' speed pulse outputs (SC) on D2 (right) / D3 (left), sign = commanded direction
//                          (the default: only one wire per motor)
//   FEEDBACK_HALLS       : the motors' three hall sensors tapped onto A0-A5, decoded here (direction-aware, better)
#define FEEDBACK_HALLS 1
#define FEEDBACK_SPEED_PULSE 2
#define FEEDBACK FEEDBACK_SPEED_PULSE

const bool HALL_PULLUPS = false;  // true: enable the ATmega's pull-ups (only if nothing else pulls the lines up)

// Motor pole pairs (hoverboard hub motors: 15, i.e. 30 magnets). Three halls give 6 states per electrical revolution,
// so 90 counts per wheel turn.
const int POLE_PAIRS = 15;
// FEEDBACK_SPEED_PULSE: SC edges per wheel turn, MEASURE IT (step 1); 90 is only a guess (one edge per hall change)
const float COUNTS_PER_REV = 6.0f * POLE_PAIRS;

// Wheel speed at full duty (100 % throttle PWM) with your battery [rad/s]. Used as the feed-forward (step 2).
// Hoverboard hub motors turn about 16 RPM per volt with no load: roughly 1.7 x battery volts in rad/s
// (36 V -> 60 rad/s).
const float MOTOR_MAX_RAD_S = 68.0f;  // measured with motor_test.py speed: left 69.8, right 66.7

// The policy was trained for wheel speeds up to 30 rad/s (WHEEL_MAX_SPEED in the Isaac Lab package).
const float MAX_TARGET_RAD_S = 30.0f;

// Velocity loop: duty = target / MOTOR_MAX_RAD_S + KP * error + KI * integral(error), duty in [-1, 1].
// The feed-forward does most of the work: the driver drives current both ways, so the motor's back-EMF already pulls
// the wheel to about target speed. Keep KP / KI low: below ~2 rad/s the hall speed estimate is coarse and delayed
// (one edge per 35+ ms), and a stiff loop reacting to it makes the wheels twitch. Must match HoverboardDriveCfg.
float KP = 0.01f;  // duty per rad/s of error
float KI = 0.2f;   // duty per rad of accumulated error

// Kick added in the target's direction to get past the driver's dead zone at low duty. 0 = let the integral do it
// (what the policy is trained with); wheel_check measures the dead zone.
const float MIN_DUTY = 0.0f;

// Output protection (the ZS-X11H trips its over-current protection, or the battery's BMS, on hard reversals and
// throttle steps: violent vibration, then everything cuts out):
//   * the throttle rises by at most DUTY_SLEW_PER_S per second (falling is not limited: less throttle is safe)
//   * DIR only changes with the throttle at zero, the wheel slower than REVERSE_MAX_RAD_S, and at least
//     DIR_MIN_HOLD_MS after the last change (no chattering when the speed loop hovers around zero)
// The wheel coasts (throttle 0) while a reversal waits.
const float DUTY_SLEW_PER_S = 4.0f;     // 0 -> 100 % in 0.25 s, 0 -> 30 % in 75 ms
const float REVERSE_MAX_RAD_S = 3.0f;   // ~0.25 m/s at the tyre
const uint32_t DIR_MIN_HOLD_MS = 30;

// Throttle PWM frequency: 0 = 15.6 kHz (default), 1 = 1.95 kHz. Both 10-bit. ZS-X11H boards are usually specified
// for ~1-20 kHz; if the motor does not respond smoothly to "D 0.1 0.1" .. "D 0.5 0.5", try the other one.
#define PWM_SLOW 0

const uint32_t SERIAL_BAUD = 115200;
const float CONTROL_HZ = 500.0f;          // velocity loop rate
const float REPORT_HZ = 200.0f;           // "W" message rate: fresher wheel speeds for the 50 Hz policy (~4 kB/s)
const uint32_t STANDSTILL_US = 100000;    // no hall edge for this long -> speed 0
const float MAX_MEASURED_RAD_S = 100.0f;  // clip speed estimates (two edges microseconds apart give absurd values)
const uint32_t CMD_TIMEOUT_MS = 200;      // no "V" / "D" command for this long -> outputs off

// ------------------------------------------------------------------------------------------------------------

const uint16_t PWM_TOP = 1023;  // throttle resolution: 10 bits (Timer1 TOP = ICR1; Mega: 8 bits + 2 dithered)

#if BOARD_MEGA
typedef volatile uint8_t PwmReg;
#define PWM_TCCRA TCCR2A
#else
typedef volatile uint16_t PwmReg;
#define PWM_TCCRA TCCR1A
#endif

struct Wheel {
  const char* name;
  uint8_t dirPin;
  PwmReg* ocr;             // timer compare register of this wheel's throttle output
  uint8_t comBit;          // TCCRxA bit that connects the output (COM1A1 / COM1B1, Mega: COM2B1 / COM2A1)
  uint8_t pwmPin;
  bool motorInvert, feedbackInvert;

  // written by the feedback interrupt (read with interrupts off)
  volatile int32_t count;
  volatile uint32_t lastEdgeUs;
  volatile uint8_t hallState;
  volatile uint32_t hallErrors;
  volatile int8_t pulseSign;  // FEEDBACK_SPEED_PULSE: +1 / -1 = direction the driver is pushing (forward = +1)

  volatile uint16_t pwm10;  // Mega: 10-bit throttle, dithered onto the 8-bit timer by TIMER2_OVF
  uint8_t ditherAcc;

  int32_t prevCount;
  uint32_t prevEdgeUs;
  int8_t lastDir;  // direction of the last counted edge(s): +1, -1, 0 = none yet
  float target;    // rad/s, forward positive
  float duty;      // open-loop duty ("D" command)
  float velocity;  // rad/s, forward positive
  float integral;  // rad

  float appliedDuty;    // what actually goes to the driver, after the output protection, [-1, 1]
  int8_t dirSign;       // direction DIR currently selects: +1 forward, -1 backward
  uint32_t dirChangeMs; // when DIR last changed
};

// timer compare channel of each throttle pin: D9 = OC2B (Mega) / OC1A (Uno), D10 = OC2A / OC1B
#if BOARD_MEGA
#define D9_OCR OCR2B
#define D9_COM COM2B1
#define D10_OCR OCR2A
#define D10_COM COM2A1
#else
#define D9_OCR OCR1A
#define D9_COM COM1A1
#define D10_OCR OCR1B
#define D10_COM COM1B1
#endif
#if SWAP_LEFT_RIGHT
Wheel leftWheel = {"left", LEFT_DIR_PIN, &D9_OCR, D9_COM, LEFT_PWM_PIN, LEFT_MOTOR_INVERT, LEFT_FEEDBACK_INVERT};
Wheel rightWheel = {"right", RIGHT_DIR_PIN, &D10_OCR, D10_COM, RIGHT_PWM_PIN, RIGHT_MOTOR_INVERT, RIGHT_FEEDBACK_INVERT};
#else
Wheel leftWheel = {"left", LEFT_DIR_PIN, &D10_OCR, D10_COM, LEFT_PWM_PIN, LEFT_MOTOR_INVERT, LEFT_FEEDBACK_INVERT};
Wheel rightWheel = {"right", RIGHT_DIR_PIN, &D9_OCR, D9_COM, RIGHT_PWM_PIN, RIGHT_MOTOR_INVERT, RIGHT_FEEDBACK_INVERT};
#endif
Wheel* const wheels[2] = {&leftWheel, &rightWheel};

enum Mode { MODE_OFF, MODE_SPEED, MODE_OPEN_LOOP };
Mode mode = MODE_OFF;
uint32_t lastCmdMs = 0;

// ---- speed feedback ----

// hall state (A << 2 | B << 1 | C) -> position in the 6-step sequence 1, 3, 2, 6, 4, 5; -1 = invalid (000, 111)
const int8_t HALL_INDEX[8] = {-1, 0, 2, 1, 4, 5, 3, -1};

// port C bits PC0..PC2 = A0..A2 (left A, B, C), PC3..PC5 = A3..A5 (right A, B, C)
inline uint8_t hallStateFromPort(uint8_t port, uint8_t shift) {
  uint8_t bits = (port >> shift) & 0x07;  // bit 0 = A, bit 1 = B, bit 2 = C
  return ((bits & 0x01) << 2) | (bits & 0x02) | ((bits & 0x04) >> 2);
}

inline void hallEdge(Wheel& w, uint8_t state, uint32_t nowUs) {
  if (state == w.hallState) return;  // the other motor's lines changed
  int8_t index = HALL_INDEX[state];
  int8_t prevIndex = HALL_INDEX[w.hallState];
  w.hallState = state;
  if (index < 0 || prevIndex < 0) {
    w.hallErrors++;
    return;
  }
  int8_t step = (index - prevIndex + 6) % 6;  // 1 = one step one way, 5 = one step the other way
  if (step != 1 && step != 5) {
    w.hallErrors++;  // skipped a state: noise or a swapped wire
    return;
  }
  w.count += (step == 1) ? 1 : -1;
  w.lastEdgeUs = nowUs;
}

#if FEEDBACK == FEEDBACK_HALLS && SWAP_LEFT_RIGHT
#error "SWAP_LEFT_RIGHT does not swap the hall inputs: swap the hall wires (A0-A2 <-> A3-A5) instead"
#endif
#if FEEDBACK == FEEDBACK_HALLS && defined(__AVR_ATmega2560__)
#error "FEEDBACK_HALLS reads A0-A5 through port C: ATmega328P only. Use FEEDBACK_SPEED_PULSE on the Mega."
#endif

#if FEEDBACK == FEEDBACK_HALLS
ISR(PCINT1_vect) {
  uint8_t port = PINC;
  uint32_t now = micros();
  hallEdge(leftWheel, hallStateFromPort(port, 0), now);
  hallEdge(rightWheel, hallStateFromPort(port, 3), now);
}
#else
void onRightPulse() {
  rightWheel.count += rightWheel.pulseSign;
  rightWheel.lastEdgeUs = micros();
}
void onLeftPulse() {
  leftWheel.count += leftWheel.pulseSign;
  leftWheel.lastEdgeUs = micros();
}
#endif

// Speed from the time between hall edges: with only ~90 counts per turn, counting edges per control period would
// give a resolution of several rad/s. Between edges the speed can be at most one count over the time since the last
// edge, which lets a stopping wheel's speed fall smoothly to zero.
void updateVelocity(Wheel& w, uint32_t nowUs) {
  noInterrupts();
  int32_t count = w.count;
  uint32_t edgeUs = w.lastEdgeUs;
  interrupts();
  if (FEEDBACK == FEEDBACK_HALLS && w.feedbackInvert) count = -count;  // speed pulses are already signed forward
  const float radPerCount = 2.0f * PI / COUNTS_PER_REV;

  int32_t newCounts = count - w.prevCount;
  if (newCounts != 0) {
    uint32_t dt = edgeUs - w.prevEdgeUs;  // spans exactly newCounts counts
    if (dt > STANDSTILL_US) dt = STANDSTILL_US;  // first edge after standing still
    if (dt == 0) dt = 1;
    int8_t dir = newCounts > 0 ? 1 : -1;
    if (w.lastDir != 0 && dir != w.lastDir) {
      // the wheel rocked back over the same hall boundary: dt is not the time for one count (it can be a few ms,
      // which would read as 10+ rad/s and kick the speed loop). The wheel has just passed through zero speed.
      w.velocity = 0.0f;
    } else {
      w.velocity = constrain(newCounts * radPerCount / (dt * 1e-6f), -MAX_MEASURED_RAD_S, MAX_MEASURED_RAD_S);
    }
    w.lastDir = dir;
    w.prevCount = count;
    w.prevEdgeUs = edgeUs;
  } else {
    uint32_t since = nowUs - edgeUs;
    if (since >= STANDSTILL_US) {
      w.velocity = 0.0f;
    } else {
      float bound = radPerCount / (since * 1e-6f + 1e-6f);
      if (fabs(w.velocity) > bound) w.velocity = w.velocity > 0.0f ? bound : -bound;
    }
  }
}

// ---- motor output ----

#if BOARD_MEGA
// Spreads the 2 low bits of each wheel's 10-bit throttle over 4 PWM periods (the driver sees the average).
ISR(TIMER2_OVF_vect) {
  for (Wheel* w : wheels) {
    uint16_t v = w->pwm10;
    uint8_t high = v >> 2;
    w->ditherAcc += v & 3;
    if (w->ditherAcc >= 4) {
      w->ditherAcc -= 4;
      if (high < 255) high++;
    }
    *w->ocr = high;  // double-buffered: takes effect at the next period
  }
}
#endif

void setupPwm() {
#if BOARD_MEGA
  // Timer2 fast PWM, mode 3 (TOP = 255) on OC2B (D9) and OC2A (D10): 16 MHz / 8 / 256 = 7.8 kHz, / 32 = 1.95 kHz
  TCCR2A = (1 << WGM21) | (1 << WGM20);
  TCCR2B = PWM_SLOW ? ((1 << CS21) | (1 << CS20)) : (1 << CS21);
  OCR2A = OCR2B = 0;
  TIMSK2 = (1 << TOIE2);
#else
  // Timer1 fast PWM, mode 14 (TOP = ICR1) on OC1A (D9) and OC1B (D10): 16 MHz / 1024 = 15.6 kHz without
  // prescaler, / 8 = 1.95 kHz
  TCCR1A = (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (PWM_SLOW ? (1 << CS11) : (1 << CS10));
  ICR1 = PWM_TOP;
  OCR1A = OCR1B = 0;
#endif
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(LEFT_PWM_PIN, OUTPUT);
  digitalWrite(RIGHT_PWM_PIN, LOW);
  digitalWrite(LEFT_PWM_PIN, LOW);
}

// 0 .. PWM_TOP. Zero disconnects the timer from the pin and holds it low (fast PWM with OCR = 0 still gives a short
// spike every period).
void writePwm(Wheel& w, uint16_t value) {
  if (value > PWM_TOP) value = PWM_TOP;
#if BOARD_MEGA
  noInterrupts();  // 16-bit value read by TIMER2_OVF
  w.pwm10 = value;
  interrupts();
#endif
  if (value == 0) {
    PWM_TCCRA &= ~(1 << w.comBit);
    digitalWrite(w.pwmPin, LOW);
  } else {
#if !BOARD_MEGA
    *w.ocr = value;
#endif
    PWM_TCCRA |= (1 << w.comBit);
  }
}

void setDir(Wheel& w, int8_t sign) {
  digitalWrite(w.dirPin, ((sign > 0) != w.motorInvert) ? HIGH : LOW);
  w.dirSign = sign;
  w.pulseSign = sign;
  w.dirChangeMs = millis();
}

void stopMotor(Wheel& w) {
  w.appliedDuty = 0.0f;
  writePwm(w, 0);
}

// duty in [-1, 1], positive = forward, through the output protection (see DUTY_SLEW_PER_S). Returns true when the
// output had to be held back (the speed loop then stops integrating).
bool driveMotor(Wheel& w, float duty, float dt) {
  int8_t want = duty > 0.0f ? 1 : (duty < 0.0f ? -1 : w.dirSign);
  float magnitude = fabs(duty);
  bool limited = false;
  if (want != w.dirSign) {
    // reversal: coast until the wheel has slowed down, then flip DIR (throttle is already 0 here)
    bool slow = fabs(w.velocity) < REVERSE_MAX_RAD_S;
    bool held = millis() - w.dirChangeMs >= DIR_MIN_HOLD_MS;
    if (slow && held) {
      writePwm(w, 0);
      setDir(w, want);
      w.appliedDuty = 0.0f;
    } else {
      stopMotor(w);
      return true;
    }
  }
  float current = fabs(w.appliedDuty);
  float maxUp = current + DUTY_SLEW_PER_S * dt;
  if (magnitude > maxUp) {
    magnitude = maxUp;
    limited = true;
  }
  w.appliedDuty = w.dirSign * magnitude;
  writePwm(w, (uint16_t)(magnitude * PWM_TOP + 0.5f));
  return limited;
}

// ---- velocity loop ----

void controlWheel(Wheel& w, float dt) {
  if (mode == MODE_OFF) {
    w.integral = 0.0f;
    stopMotor(w);
    return;
  }
  if (mode == MODE_OPEN_LOOP) {
    w.integral = 0.0f;
    driveMotor(w, w.duty, dt);
    return;
  }
  float error = w.target - w.velocity;
  float duty = w.target / MOTOR_MAX_RAD_S + KP * error + KI * w.integral;
  if (MIN_DUTY > 0.0f && fabs(w.target) > 0.2f) duty += (w.target > 0.0f ? MIN_DUTY : -MIN_DUTY);
  duty = constrain(duty, -1.0f, 1.0f);
  // anti-windup: hold the integral while saturated in the direction of the error
  bool saturated = (duty >= 1.0f && error > 0.0f) || (duty <= -1.0f && error < 0.0f);
  bool limited = driveMotor(w, duty, dt);
  if (!saturated && !limited) w.integral += error * dt;
}

// ---- serial protocol ----

void printConfig() {
  Serial.print(F("# balance bot ATmega BLDC wheel controller (ZS-X11H, throttle right D"));
  Serial.print(RIGHT_PWM_PIN);
  Serial.print(F(" / left D"));
  Serial.print(LEFT_PWM_PIN);
  Serial.println(F(" + DIR D6/D7)"));
  Serial.print(F("# feedback "));
  Serial.print(FEEDBACK == FEEDBACK_HALLS ? F("halls A0-A5") : F("speed pulse D2/D3"));
  Serial.print(F(", "));
  Serial.print(COUNTS_PER_REV, 0);
  Serial.print(F(" counts/rev, max "));
  Serial.print(MOTOR_MAX_RAD_S, 1);
  Serial.print(F(" rad/s, target limit "));
  Serial.print(MAX_TARGET_RAD_S, 1);
  Serial.println(F(" rad/s"));
  Serial.print(F("# kp "));
  Serial.print(KP, 4);
  Serial.print(F(" ki "));
  Serial.print(KI, 4);
  Serial.print(F(", control "));
  Serial.print(CONTROL_HZ, 0);
  Serial.print(F(" Hz, report "));
  Serial.print(REPORT_HZ, 0);
  Serial.print(F(" Hz, timeout "));
  Serial.print(CMD_TIMEOUT_MS);
  Serial.println(F(" ms"));
  for (Wheel* w : wheels) {
    noInterrupts();
    int32_t count = w->count;
    uint8_t state = w->hallState;
    uint32_t errors = w->hallErrors;
    interrupts();
    Serial.print(F("# "));
    Serial.print(w->name);
    Serial.print(F(": count "));
    Serial.print(FEEDBACK == FEEDBACK_HALLS && w->feedbackInvert ? -count : count);
    Serial.print(F(", hall state "));
    Serial.print(state);
    Serial.print(F(", hall errors "));
    Serial.println(errors);
  }
}

bool parseTwo(char* text, float& a, float& b) {
  char* end;
  a = strtod(text, &end);
  if (end == text) return false;
  char* end2;
  b = strtod(end, &end2);
  return end2 != end;
}

void handleLine(char* line) {
  float a, b;
  switch (line[0]) {
    case 'V':
      if (!parseTwo(line + 1, a, b)) break;
      leftWheel.target = constrain(a, -MAX_TARGET_RAD_S, MAX_TARGET_RAD_S);
      rightWheel.target = constrain(b, -MAX_TARGET_RAD_S, MAX_TARGET_RAD_S);
      mode = MODE_SPEED;
      lastCmdMs = millis();
      break;
    case 'D':
      if (!parseTwo(line + 1, a, b)) break;
      leftWheel.duty = constrain(a, -1.0f, 1.0f);
      rightWheel.duty = constrain(b, -1.0f, 1.0f);
      mode = MODE_OPEN_LOOP;
      lastCmdMs = millis();
      break;
    case 'S':
      mode = MODE_OFF;
      leftWheel.target = rightWheel.target = 0.0f;
      break;
    case 'G':
      if (parseTwo(line + 1, a, b)) {
        KP = a;
        KI = b;
      }
      Serial.print(F("# kp "));
      Serial.print(KP, 4);
      Serial.print(F(" ki "));
      Serial.println(KI, 4);
      break;
    case '?':
      printConfig();
      break;
    default:
      break;
  }
}

void readSerial() {
  static char buf[48];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      buf[len] = '\0';
      if (len > 0) handleLine(buf);
      len = 0;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    } else {
      len = 0;  // overlong line: drop it
    }
  }
}

// ---- main ----

void setupFeedback() {
  const uint8_t inputMode = HALL_PULLUPS ? INPUT_PULLUP : INPUT;
#if FEEDBACK == FEEDBACK_HALLS
  for (uint8_t pin = A0; pin <= A5; pin++) pinMode(pin, inputMode);
  uint8_t port = PINC;
  leftWheel.hallState = hallStateFromPort(port, 0);
  rightWheel.hallState = hallStateFromPort(port, 3);
  PCMSK1 = 0x3F;           // PCINT8..13 = A0..A5
  PCIFR = (1 << PCIF1);    // clear a pending flag
  PCICR |= (1 << PCIE1);   // enable the port C pin-change interrupt
#else
  pinMode(2, inputMode);
  pinMode(3, inputMode);
  rightWheel.pulseSign = leftWheel.pulseSign = 1;
  // D2 = channel 1's speed pulse, D3 = channel 2's
  attachInterrupt(digitalPinToInterrupt(2), SWAP_LEFT_RIGHT ? onLeftPulse : onRightPulse, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), SWAP_LEFT_RIGHT ? onRightPulse : onLeftPulse, CHANGE);
#endif
  uint32_t now = micros();
  for (Wheel* w : wheels) {
    w->lastEdgeUs = w->prevEdgeUs = now;
    w->lastDir = 0;
  }
}

void setup() {
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  setupPwm();
  for (Wheel* w : wheels) setDir(*w, 1);
  Serial.begin(SERIAL_BAUD);
  setupFeedback();
  printConfig();
}

void loop() {
  static uint32_t lastControlUs = micros();
  static uint32_t lastReportUs = micros();
  const uint32_t controlPeriodUs = (uint32_t)(1e6f / CONTROL_HZ);
  const uint32_t reportPeriodUs = (uint32_t)(1e6f / REPORT_HZ);

  readSerial();
  if (mode != MODE_OFF && millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    mode = MODE_OFF;  // PC or cable gone: switch the motors off
    leftWheel.target = rightWheel.target = 0.0f;
    Serial.print(F("# no command for "));
    Serial.print(CMD_TIMEOUT_MS);
    Serial.println(F(" ms, motors off"));
  }

  uint32_t now = micros();
  if (now - lastControlUs >= controlPeriodUs) {
    lastControlUs += controlPeriodUs;
    if (now - lastControlUs > 10 * controlPeriodUs) lastControlUs = now;  // fell far behind: resync
    for (Wheel* w : wheels) {
      updateVelocity(*w, now);
      controlWheel(*w, 1.0f / CONTROL_HZ);
    }
  }
  if (now - lastReportUs >= reportPeriodUs) {
    lastReportUs += reportPeriodUs;
    if (now - lastReportUs > 10 * reportPeriodUs) lastReportUs = now;
    Serial.print(F("W "));
    Serial.print(leftWheel.velocity, 3);
    Serial.print(' ');
    Serial.println(rightWheel.velocity, 3);
  }
}

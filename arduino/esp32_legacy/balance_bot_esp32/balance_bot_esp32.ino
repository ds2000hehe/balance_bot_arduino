// Balance bot wheel controller for an ESP32 (Arduino IDE, "esp32" boards package 2.x or 3.x) driving two sensored
// BLDC motors through BLDC driver boards controlled with PWM + DIR only (hoverboard motors). The drivers' BRAKE and
// STOP inputs are not used: tie them (or leave them) at the level that lets the motor run.
//
// Runs a velocity loop on each wheel from the motor's hall sensors and talks to the PC over USB serial:
//
//   PC -> ESP32   "V <left> <right>\n"   wheel speed targets [rad/s], positive = drive the robot forward
//                 "S\n"                  stop: motor outputs off (PWM 0, wheels coast) until the next V / D
//                 "D <left> <right>\n"   open-loop test: raw duty in [-1, 1], no speed loop (wheels lifted!)
//                 "G <kp> <ki>\n"        set the velocity loop gains (kp in duty per rad/s, ki in duty per rad)
//                 "?\n"                  print the configuration, hall counts and hall errors
//   ESP32 -> PC   "W <left> <right>\n"   measured wheel speeds [rad/s] at REPORT_HZ, positive = forward
//                 "# ...\n"              information lines (ignored by the ROS bridge, shown in its log)
//
// "left" and "right" are seen from behind the robot (front = where it drives with positive speed). This matches
// /wheel_cmd and /wheel_vel with wheel_signs [1, 1] in balance_bot_policy's config/policy.yaml. "V" and "D" commands
// must keep coming (the ROS bridge sends 50 per second): after CMD_TIMEOUT_MS without one, the outputs switch off.
//
// Wiring
//   * Hall sensors: tap the motor's three hall wires (A, B, C) in parallel with the driver, which still needs them.
//     Hall outputs are usually pulled up to 5 V by the driver and the ESP32 is NOT 5 V tolerant: put a divider
//     (e.g. 10k from the hall line, 20k to GND) or a level shifter on each line.
//     If you can only use the driver's speed-pulse output ("SC"/"SPEED"), set FEEDBACK = FEEDBACK_SPEED_PULSE: it
//     has no direction information, so the sign is taken from the commanded direction (worse for balancing).
//   * Driver inputs: most accept 3.3 V logic; if yours needs 5 V, use a level shifter. Connect the grounds.
//
// First-time setup, wheels off the ground, Arduino Serial Monitor at 115200 baud, line ending "Newline":
//   1. Set the pins and POLE_PAIRS below. Turn each wheel one turn forward by hand and send
//      "?": the count must change by about +COUNTS_PER_REV. A wheel that counts negative: flip its FEEDBACK_INVERT.
//      Hall errors that keep growing: a hall wire is swapped, missing, or noisy.
//   2. Send "D 0.1 0.1" a few times a second (or paste it repeatedly): both wheels must turn forward and "W" must be
//      positive. A wheel turning backward: flip its MOTOR_INVERT. Send "D 0.5 0.5": "W" x 2 is roughly
//      MOTOR_MAX_RAD_S. (The ROS tool `ros2 run balance_bot_policy wheel_check` measures all of this for you.)
//   3. Send "V 5 5" repeatedly, then "V -5 -5", then "V 20 20". The wheels should reach the speed within ~0.1 s,
//      without buzzing. If you change KP / KI, change HoverboardDriveCfg in the Isaac Lab package and retrain.

// ------------------------------------------------------------------------------------------------------------
// Configuration
// ------------------------------------------------------------------------------------------------------------

// Pins (ESP32 DevKit). Avoid GPIO 0, 2, 12, 15 (boot strapping) and 6-11 (flash). GPIO 34-39 are input only and
// have no internal pull-ups (fine for hall lines through a divider). On WROVER modules GPIO 16/17 are taken by PSRAM.
const int LEFT_PWM_PIN = 25;
const int LEFT_DIR_PIN = 26;
const int LEFT_HALL_A_PIN = 18;
const int LEFT_HALL_B_PIN = 19;  // unused with FEEDBACK_SPEED_PULSE (-1)
const int LEFT_HALL_C_PIN = 4;   // unused with FEEDBACK_SPEED_PULSE (-1)

const int RIGHT_PWM_PIN = 21;
const int RIGHT_DIR_PIN = 22;
const int RIGHT_HALL_A_PIN = 34;  // with FEEDBACK_SPEED_PULSE: the driver's speed pulse output
const int RIGHT_HALL_B_PIN = 35;
const int RIGHT_HALL_C_PIN = 32;

const bool HALL_PULLUPS = false;  // true: enable the ESP32's 3.3 V pull-ups (only if nothing pulls them to 5 V)

// Flip these so that a positive target drives the robot forward and is measured as positive (see setup above).
// The motors are mirrored on a two-wheeled robot, so usually exactly one of the two MOTOR_INVERTs is true.
const bool LEFT_MOTOR_INVERT = false;
const bool LEFT_FEEDBACK_INVERT = false;
const bool RIGHT_MOTOR_INVERT = true;
const bool RIGHT_FEEDBACK_INVERT = true;

// Speed feedback:
//   FEEDBACK_HALLS       : the motor's three hall sensors, decoded here (direction-aware, recommended)
//   FEEDBACK_SPEED_PULSE : the driver's speed pulse output on HALL_A, one input, sign = commanded direction
#define FEEDBACK_HALLS 1
#define FEEDBACK_SPEED_PULSE 2
#define FEEDBACK FEEDBACK_HALLS

// Motor pole pairs (hoverboard hub motors: 15, i.e. 30 magnets). Three halls give 6 states per electrical revolution,
// so 90 counts per wheel turn.
const int POLE_PAIRS = 15;
const float COUNTS_PER_REV = 6.0f * POLE_PAIRS;  // FEEDBACK_SPEED_PULSE: pulses per wheel turn, measure it (step 1)

// Wheel speed at full PWM with your battery [rad/s]. Used as the feed-forward (step 2).
// Hoverboard hub motors turn about 16 RPM per volt with no load: roughly 1.7 x battery volts in rad/s
// (36 V -> 60 rad/s, 24 V -> 40 rad/s).
const float MOTOR_MAX_RAD_S = 60.0f;

// The policy was trained for wheel speeds up to 30 rad/s (WHEEL_MAX_SPEED in the Isaac Lab package).
const float MAX_TARGET_RAD_S = 30.0f;

// Velocity loop: duty = target / MOTOR_MAX_RAD_S + KP * error + KI * integral(error), duty in [-1, 1].
// The feed-forward does most of the work: the driver drives current both ways, so the motor's back-EMF already pulls
// the wheel to about target speed. Keep KP / KI low: below ~2 rad/s the hall speed estimate is coarse and delayed
// (one edge per 35+ ms), and a stiff loop reacting to it makes the wheels twitch. Must match HoverboardDriveCfg.
float KP = 0.01f;  // duty per rad/s of error
float KI = 0.2f;   // duty per rad of accumulated error

// Kick added in the target's direction to get past the driver's dead zone at low PWM. 0 = let the integral do it
// (what the policy is trained with); wheel_check measures the dead zone.
const float MIN_DUTY = 0.0f;

const uint32_t PWM_FREQ_HZ = 16000;  // check the range your driver accepts (often 1-20 kHz)
const uint8_t PWM_BITS = 10;

const uint32_t SERIAL_BAUD = 115200;  // ignored by boards with native USB (ESP32-S2/S3/C3 "USB CDC on boot")
const float CONTROL_HZ = 500.0f;      // velocity loop rate
const float REPORT_HZ = 100.0f;       // "W" message rate (the policy runs at 50 Hz)
const uint32_t STANDSTILL_US = 100000;  // no hall edge for this long -> speed 0
const float MAX_MEASURED_RAD_S = 100.0f;  // clip speed estimates (two edges microseconds apart give absurd values)
const uint32_t CMD_TIMEOUT_MS = 200;    // no "V" / "D" command for this long -> outputs off

// ------------------------------------------------------------------------------------------------------------

#include <Arduino.h>
#include <driver/gpio.h>

struct Wheel {
  const char* name;
  int pwmPin, dirPin, hallA, hallB, hallC;
  bool motorInvert, feedbackInvert;

  // written by the hall interrupt (guarded by hallMux)
  volatile int32_t count;
  volatile uint32_t lastEdgeUs;
  volatile uint8_t hallState;
  volatile uint32_t hallErrors;
  volatile int8_t pulseSign;  // FEEDBACK_SPEED_PULSE: +1 / -1 = direction the driver is pushing (forward = +1)

  int32_t prevCount;
  uint32_t prevEdgeUs;
  int8_t lastDir;  // direction of the last counted edge(s): +1, -1, 0 = none yet
  float target;    // rad/s, forward positive
  float duty;      // open-loop duty ("D" command)
  float velocity;  // rad/s, forward positive
  float integral;  // rad
};

Wheel leftWheel = {"left", LEFT_PWM_PIN, LEFT_DIR_PIN, LEFT_HALL_A_PIN, LEFT_HALL_B_PIN, LEFT_HALL_C_PIN,
                   LEFT_MOTOR_INVERT, LEFT_FEEDBACK_INVERT};
Wheel rightWheel = {"right", RIGHT_PWM_PIN, RIGHT_DIR_PIN, RIGHT_HALL_A_PIN, RIGHT_HALL_B_PIN, RIGHT_HALL_C_PIN,
                    RIGHT_MOTOR_INVERT, RIGHT_FEEDBACK_INVERT};
Wheel* wheels[2] = {&leftWheel, &rightWheel};

enum Mode { MODE_OFF, MODE_SPEED, MODE_OPEN_LOOP };
Mode mode = MODE_OFF;
uint32_t lastCmdMs = 0;
const int PWM_MAX = (1 << PWM_BITS) - 1;
portMUX_TYPE hallMux = portMUX_INITIALIZER_UNLOCKED;

// ---- speed feedback ----

// hall state (A << 2 | B << 1 | C) -> position in the 6-step sequence 1, 3, 2, 6, 4, 5; -1 = invalid (000, 111)
const int8_t HALL_INDEX[8] = {-1, 0, 2, 1, 4, 5, 3, -1};

uint8_t IRAM_ATTR readHalls(const Wheel* w) {
  return (gpio_get_level((gpio_num_t)w->hallA) << 2) | (gpio_get_level((gpio_num_t)w->hallB) << 1) |
         gpio_get_level((gpio_num_t)w->hallC);
}

void IRAM_ATTR onHallEdge(void* arg) {
  Wheel* w = (Wheel*)arg;
  uint8_t state = readHalls(w);
  int8_t index = HALL_INDEX[state];
  int8_t prevIndex = HALL_INDEX[w->hallState];
  w->hallState = state;
  if (index < 0 || prevIndex < 0) {
    w->hallErrors++;
    return;
  }
  int8_t step = (index - prevIndex + 6) % 6;  // 1 = one step one way, 5 = one step the other way
  if (step == 0) return;                      // bounce on one line
  if (step != 1 && step != 5) {
    w->hallErrors++;  // skipped a state: noise or a swapped wire
    return;
  }
  portENTER_CRITICAL_ISR(&hallMux);
  w->count += (step == 1) ? 1 : -1;
  w->lastEdgeUs = micros();
  portEXIT_CRITICAL_ISR(&hallMux);
}

void IRAM_ATTR onSpeedPulse(void* arg) {
  Wheel* w = (Wheel*)arg;
  portENTER_CRITICAL_ISR(&hallMux);
  w->count += w->pulseSign;
  w->lastEdgeUs = micros();
  portEXIT_CRITICAL_ISR(&hallMux);
}

// Speed from the time between hall edges: with only ~90 counts per turn, counting edges per control period would
// give a resolution of several rad/s. Between edges the speed can be at most one count over the time since the last
// edge, which lets a stopping wheel's speed fall smoothly to zero.
void updateVelocity(Wheel& w, uint32_t nowUs) {
  portENTER_CRITICAL(&hallMux);
  int32_t count = w.count;
  uint32_t edgeUs = w.lastEdgeUs;
  portEXIT_CRITICAL(&hallMux);
  if (w.feedbackInvert) count = -count;
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
      if (fabsf(w.velocity) > bound) w.velocity = copysignf(bound, w.velocity);
    }
  }
}

// ---- motor output ----

#if ESP_ARDUINO_VERSION_MAJOR < 3
int pwmChannelOfPin[40];  // core 2.x addresses PWM by channel, core 3.x by pin
#endif

void setupPwm(int pin) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, PWM_FREQ_HZ, PWM_BITS);
#else
  static int nextChannel = 0;
  ledcSetup(nextChannel, PWM_FREQ_HZ, PWM_BITS);
  ledcAttachPin(pin, nextChannel);
  pwmChannelOfPin[pin] = nextChannel++;
#endif
}

void writePwm(int pin, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(pwmChannelOfPin[pin], duty);
#endif
}

// duty in [-1, 1], positive = forward; the driver handles direction changes at speed by itself
void driveMotor(Wheel& w, float duty) {
  bool forward = duty >= 0.0f;
  digitalWrite(w.dirPin, (forward != w.motorInvert) ? HIGH : LOW);
  w.pulseSign = forward ? 1 : -1;
  int pwm = (int)(fabsf(duty) * PWM_MAX + 0.5f);
  writePwm(w.pwmPin, pwm > PWM_MAX ? PWM_MAX : pwm);
}

// ---- velocity loop ----

void controlWheel(Wheel& w, float dt) {
  if (mode == MODE_OFF) {
    w.integral = 0.0f;
    writePwm(w.pwmPin, 0);
    return;
  }
  if (mode == MODE_OPEN_LOOP) {
    w.integral = 0.0f;
    driveMotor(w, w.duty);
    return;
  }
  float error = w.target - w.velocity;
  float duty = w.target / MOTOR_MAX_RAD_S + KP * error + KI * w.integral;
  if (MIN_DUTY > 0.0f && fabsf(w.target) > 0.2f) duty += (w.target > 0.0f ? MIN_DUTY : -MIN_DUTY);
  duty = constrain(duty, -1.0f, 1.0f);
  // anti-windup: hold the integral while saturated in the direction of the error
  bool saturated = (duty >= 1.0f && error > 0.0f) || (duty <= -1.0f && error < 0.0f);
  if (!saturated) w.integral += error * dt;
  driveMotor(w, duty);
}

// ---- serial protocol ----

void printConfig() {
  Serial.printf("# balance bot ESP32 BLDC wheel controller\n");
  Serial.printf("# feedback %s, %.0f counts/rev, max %.1f rad/s, target limit %.1f rad/s\n",
                FEEDBACK == FEEDBACK_HALLS ? "halls" : "speed pulse", COUNTS_PER_REV, MOTOR_MAX_RAD_S,
                MAX_TARGET_RAD_S);
  Serial.printf("# kp %.4f ki %.4f, control %.0f Hz, report %.0f Hz, timeout %u ms\n", KP, KI, CONTROL_HZ,
                REPORT_HZ, (unsigned)CMD_TIMEOUT_MS);
  for (Wheel* w : wheels) {
    Serial.printf("# %s: count %ld, hall state %u, hall errors %lu\n", w->name,
                  (long)(w->feedbackInvert ? -w->count : w->count), (unsigned)w->hallState,
                  (unsigned long)w->hallErrors);
  }
}

bool parseTwo(char* text, float& a, float& b) {
  char* end;
  a = strtof(text, &end);
  if (end == text) return false;
  char* end2;
  b = strtof(end, &end2);
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
      Serial.printf("# kp %.4f ki %.4f\n", KP, KI);
      break;
    case '?':
      printConfig();
      break;
    default:
      break;
  }
}

void readSerial() {
  static char buf[64];
  static int len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      buf[len] = '\0';
      if (len > 0) handleLine(buf);
      len = 0;
    } else if (len < (int)sizeof(buf) - 1) {
      buf[len++] = c;
    } else {
      len = 0;  // overlong line: drop it
    }
  }
}

// ---- main ----

void setupWheel(Wheel& w) {
  pinMode(w.dirPin, OUTPUT);
  setupPwm(w.pwmPin);
  writePwm(w.pwmPin, 0);

  const int hallMode = HALL_PULLUPS ? INPUT_PULLUP : INPUT;
#if FEEDBACK == FEEDBACK_HALLS
  int pins[3] = {w.hallA, w.hallB, w.hallC};
  for (int pin : pins) pinMode(pin, hallMode);
  w.hallState = readHalls(&w);
  for (int pin : pins) attachInterruptArg(digitalPinToInterrupt(pin), onHallEdge, &w, CHANGE);
#else
  pinMode(w.hallA, hallMode);
  w.pulseSign = 1;
  attachInterruptArg(digitalPinToInterrupt(w.hallA), onSpeedPulse, &w, CHANGE);
#endif
  w.lastEdgeUs = w.prevEdgeUs = micros();
  w.lastDir = 0;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  setupWheel(leftWheel);
  setupWheel(rightWheel);
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
    Serial.printf("# no command for %u ms, motors off\n", (unsigned)CMD_TIMEOUT_MS);
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
    Serial.printf("W %.3f %.3f\n", leftWheel.velocity, rightWheel.velocity);
  }
}

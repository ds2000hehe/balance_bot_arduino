// Reversal braking test: can the ZS-X11H brake a spinning wheel by reversing it? WHEELS OFF THE GROUND.
//
// Separate test sketch (the balance firmware is not changed). Same pins as balance_bot_atmega on the Mega with
// SWAP_LEFT_RIGHT 1:
//   left  wheel: throttle D9,  DIR D6, speed pulse (SC) D2      right wheel: throttle D10, DIR D7, SC D3
//
// For each wheel: spin forward at SPIN_DUTY, then
//   coast:          throttle 0, the wheel freewheels (what happens now on the robot above 3 rad/s)
//   reverse x %:    throttle 0, DIR flipped after DIR_HOLD_MS, throttle x % backwards until the wheel has stopped
// and print how long the wheel takes to reach half speed and to stop. The balance firmware currently refuses to
// reverse above 3 rad/s; if reversing stops the wheel much faster than coasting (and the driver does not cut out,
// beep or get hot), that limit can be raised and the robot gets real braking.
//
// Serial Monitor 115200 baud (or: python3 -m serial.tools.miniterm /dev/ttyUSB0 115200), send "g" to start, "s" stops.
// Reflash balance_bot_atmega afterwards.

const uint8_t LEFT_PWM = 9, LEFT_DIR = 6, LEFT_SC = 2;
const uint8_t RIGHT_PWM = 10, RIGHT_DIR = 7, RIGHT_SC = 3;
const bool LEFT_FORWARD_LEVEL = LOW;    // balance_bot_atmega: LEFT_MOTOR_INVERT true  -> forward = DIR low
const bool RIGHT_FORWARD_LEVEL = HIGH;  // RIGHT_MOTOR_INVERT false -> forward = DIR high

const float COUNTS_PER_REV = 90.0f;     // SC edges per wheel turn (both edges counted)
const float SPIN_DUTY = 0.30f;          // ~20 rad/s in the air
const float REVERSE_DUTIES[] = {0.05f, 0.10f, 0.15f, 0.20f};
const uint16_t DIR_HOLD_MS = 5;         // throttle off before DIR flips
const uint16_t STOP_RAD_S = 1;          // "stopped" below this
const uint16_t MAX_BRAKE_MS = 800;      // give up (throttle off) after this

volatile uint32_t lastEdgeUs[2], prevEdgeUs[2], edges[2];
void onLeft() { prevEdgeUs[0] = lastEdgeUs[0]; lastEdgeUs[0] = micros(); edges[0]++; }
void onRight() { prevEdgeUs[1] = lastEdgeUs[1]; lastEdgeUs[1] = micros(); edges[1]++; }

bool stopRequested() {
  while (Serial.available()) {
    if (Serial.read() == 's') return true;
  }
  return false;
}

// wheel speed [rad/s, no sign] from the time between the last two speed pulses; 0 when none for 50 ms
float speed(uint8_t w) {
  noInterrupts();
  uint32_t last = lastEdgeUs[w], prev = prevEdgeUs[w];
  interrupts();
  uint32_t now = micros();
  if (now - last > 50000UL || last == prev) return 0.0f;
  uint32_t period = last - prev;
  if (now - last > period) period = now - last;  // slowing down: the next edge is already late
  return 2.0f * PI / (COUNTS_PER_REV * period * 1e-6f);
}

void drive(uint8_t w, float duty) {  // duty > 0 forward, < 0 backward, |duty| <= 1
  uint8_t pwm = w == 0 ? LEFT_PWM : RIGHT_PWM, dir = w == 0 ? LEFT_DIR : RIGHT_DIR;
  bool fwd = w == 0 ? LEFT_FORWARD_LEVEL : RIGHT_FORWARD_LEVEL;
  digitalWrite(dir, duty >= 0.0f ? fwd : !fwd);
  analogWrite(pwm, (uint8_t)(fabs(duty) * 255.0f + 0.5f));
}

void allOff() {
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
}

// spin up gently, hold, return the steady speed (0 = aborted / did not spin)
float spinUp(uint8_t w) {
  for (int i = 1; i <= 30; i++) {  // 0 -> SPIN_DUTY in 0.3 s
    drive(w, SPIN_DUTY * i / 30.0f);
    delay(10);
    if (stopRequested()) return 0.0f;
  }
  float sum = 0.0f;
  for (int i = 0; i < 150; i++) {  // 1.5 s, average the last 0.5 s
    delay(10);
    if (i >= 100) sum += speed(w);
    if (stopRequested()) return 0.0f;
  }
  return sum / 50.0f;
}

// one trial: reverse duty 0 = coast. Prints speeds every 50 ms, time to half speed and to stop.
bool trial(uint8_t w, float reverseDuty) {
  float v0 = spinUp(w);
  if (v0 < 3.0f) {
    allOff();
    Serial.println(v0 == 0.0f ? F("  stopped") : F("  wheel did not spin up (no speed pulses?)"));
    return false;
  }
  analogWrite(w == 0 ? LEFT_PWM : RIGHT_PWM, 0);
  uint32_t t0 = millis();
  if (reverseDuty > 0.0f) {
    delay(DIR_HOLD_MS);
    drive(w, -reverseDuty);
  }
  int32_t halfMs = -1, stopMs = -1;
  char line[160];
  uint8_t len = 0;
  uint32_t nextPrint = 50;
  while (millis() - t0 < MAX_BRAKE_MS) {
    uint32_t t = millis() - t0;
    float v = speed(w);
    if (halfMs < 0 && v <= 0.5f * v0) halfMs = t;
    if (v < STOP_RAD_S && t > DIR_HOLD_MS + 5) {  // stopped: throttle off before it spins up backwards
      stopMs = t;
      break;
    }
    if (t >= nextPrint && len < sizeof(line) - 8) {
      len += snprintf(line + len, sizeof(line) - len, " %4d", (int)(v + 0.5f));
      nextPrint += 50;
    }
    if (stopRequested()) break;
  }
  allOff();
  Serial.print(reverseDuty > 0.0f ? F("  reverse ") : F("  coast      "));
  if (reverseDuty > 0.0f) {
    Serial.print((int)(reverseDuty * 100 + 0.5f));
    Serial.print(F(" %: "));
  } else {
    Serial.print(F(": "));
  }
  Serial.print(F("from "));
  Serial.print(v0, 1);
  Serial.print(F(" rad/s, half speed after "));
  if (halfMs >= 0) Serial.print(halfMs); else Serial.print(F(">800"));
  Serial.print(F(" ms, stopped after "));
  if (stopMs >= 0) Serial.print(stopMs); else Serial.print(F(">800"));
  Serial.print(F(" ms | rad/s every 50 ms:"));
  Serial.println(line);
  delay(1500);  // let it come to rest
  return true;
}

void runTest() {
  const char* names[2] = {"left", "right"};
  for (uint8_t w = 0; w < 2; w++) {
    Serial.print(F("\n"));
    Serial.print(names[w]);
    Serial.println(F(" wheel:"));
    if (!trial(w, 0.0f)) return;
    for (float d : REVERSE_DUTIES) {
      if (!trial(w, d)) return;
    }
  }
  Serial.println(F("\ndone. Paste this to Claude. Did a driver cut out, beep, or get hot during the reverse runs?"));
}

void setup() {
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
  allOff();
  pinMode(LEFT_SC, INPUT);
  pinMode(RIGHT_SC, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_SC), onLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_SC), onRight, CHANGE);
  Serial.begin(115200);
  Serial.println(F("# reverse_test: reversal braking. WHEELS OFF THE GROUND. Send g to start, s to stop."));
}

void loop() {
  if (Serial.available() && Serial.read() == 'g') {
    Serial.println(F("running (send s to stop)"));
    runTest();
    allOff();
    Serial.println(F("send g to run again"));
  }
}

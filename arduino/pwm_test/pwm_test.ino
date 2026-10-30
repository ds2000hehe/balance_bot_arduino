// Wiring test, no speed loop: is it the wiring / driver or the firmware? WHEELS OFF THE GROUND.
//
// Uses plain analogWrite() on the same pins as balance_bot_atmega (Uno / Nano / Mega 2560):
//   right driver: PWM throttle D9, SC D2, DIR D6      left driver: PWM throttle D10, SC D3, DIR D7
//
// Repeats: 3 s forward at THROTTLE, 2 s off, 3 s backward, 2 s off. Prints the PWM value, DIR level and the SC pulses
// counted per second on each wheel (Serial Monitor 115200 baud).
//   Motors turn here but not with balance_bot_atmega -> firmware problem, tell Claude.
//   Motors don't turn here either -> wiring / driver: see the checks in arduino/README.md ("Motors don't move").

const uint8_t RIGHT_PWM = 9, LEFT_PWM = 10, RIGHT_DIR = 6, LEFT_DIR = 7, RIGHT_SC = 2, LEFT_SC = 3;
const uint8_t THROTTLE = 60;  // of 255 (~25 %)

volatile uint32_t rightPulses = 0, leftPulses = 0;
void onRight() { rightPulses++; }
void onLeft() { leftPulses++; }

void setThrottle(uint8_t pwm, bool forward) {
  digitalWrite(RIGHT_DIR, forward ? HIGH : LOW);
  digitalWrite(LEFT_DIR, forward ? HIGH : LOW);
  analogWrite(RIGHT_PWM, pwm);
  analogWrite(LEFT_PWM, pwm);
}

void runFor(const char* label, uint8_t pwm, bool forward, uint8_t seconds) {
  setThrottle(pwm, forward);
  for (uint8_t s = 0; s < seconds; s++) {
    noInterrupts();
    rightPulses = leftPulses = 0;
    interrupts();
    delay(1000);
    noInterrupts();
    uint32_t r = rightPulses, l = leftPulses;
    interrupts();
    Serial.print(label);
    Serial.print(F("  pwm "));
    Serial.print(pwm);
    Serial.print(F("/255  DIR "));
    Serial.print(forward ? F("HIGH") : F("LOW "));
    Serial.print(F("  SC pulses/s: right "));
    Serial.print(r);
    Serial.print(F("  left "));
    Serial.println(l);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(RIGHT_DIR, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_SC, INPUT);
  pinMode(LEFT_SC, INPUT);
  attachInterrupt(digitalPinToInterrupt(RIGHT_SC), onRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_SC), onLeft, CHANGE);
  Serial.println(F("# pwm_test: D9/D10 throttle, D6/D7 DIR, SC on D2/D3. Wheels off the ground!"));
}

void loop() {
  runFor("forward ", THROTTLE, true, 3);
  runFor("off     ", 0, true, 2);
  runFor("backward", THROTTLE, false, 3);
  runFor("off     ", 0, false, 2);
}

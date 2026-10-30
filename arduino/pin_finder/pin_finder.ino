// Pin finder: drive single pins from the PC to find out which pin moves which motor. Use with arduino/pin_finder.py.
// WHEELS OFF THE GROUND. Works on Uno / Nano / Mega 2560 (any AVR board).
//
// Serial 115200 baud, one command per line:
//   "?"              list the PWM-capable pins:            "P <pin> <pin> ..."
//   "A <pin> <0-255>" analogWrite (PWM) on a PWM pin
//   "H <pin>" / "L <pin>"  digital HIGH / LOW
//   "O"              everything off (all touched pins back to inputs)
//   "C"              speed pulses (SC) counted on D2 / D3 since the last "C":  "C <d2> <d3>"
// Safety: without any command for 500 ms, everything switches off (the Python script keeps sending).

const uint32_t TIMEOUT_MS = 500;

volatile uint32_t pulsesD2 = 0, pulsesD3 = 0;
void onD2() { pulsesD2++; }
void onD3() { pulsesD3++; }

bool touched[NUM_DIGITAL_PINS];
uint32_t lastCmdMs = 0;
char line[32];
uint8_t lineLen = 0;

bool usable(int pin) { return pin > 1 && pin < NUM_DIGITAL_PINS; }  // 0 / 1 are the USB serial

void allOff() {
  for (int pin = 2; pin < NUM_DIGITAL_PINS; pin++) {
    if (!touched[pin]) continue;
    digitalWrite(pin, LOW);  // also stops analogWrite's PWM
    pinMode(pin, INPUT);
    touched[pin] = false;
  }
}

void handle(char* cmd) {
  char c = cmd[0];
  char* end;
  long pin = strtol(cmd + 1, &end, 10);
  long value = strtol(end, nullptr, 10);
  if (c == '?') {
    Serial.print(F("P"));
    for (int p = 2; p < NUM_DIGITAL_PINS; p++) {
      if (digitalPinHasPWM(p)) {
        Serial.print(' ');
        Serial.print(p);
      }
    }
    Serial.println();
  } else if (c == 'O') {
    allOff();
    Serial.println(F("ok"));
  } else if (c == 'C') {
    noInterrupts();
    uint32_t a = pulsesD2, b = pulsesD3;
    pulsesD2 = pulsesD3 = 0;
    interrupts();
    Serial.print(F("C "));
    Serial.print(a);
    Serial.print(' ');
    Serial.println(b);
  } else if ((c == 'A' || c == 'H' || c == 'L') && usable(pin)) {
    if (c == 'A' && !digitalPinHasPWM(pin)) {
      Serial.println(F("err not a PWM pin"));
      return;
    }
    pinMode(pin, OUTPUT);
    touched[pin] = true;
    if (c == 'A') analogWrite(pin, constrain(value, 0, 255));
    else digitalWrite(pin, c == 'H' ? HIGH : LOW);
    Serial.println(F("ok"));
  } else if (c) {
    Serial.println(F("err"));
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), onD2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), onD3, CHANGE);
  Serial.println(F("# pin_finder ready"));
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      line[lineLen] = 0;
      if (lineLen) {
        handle(line);
        lastCmdMs = millis();
      }
      lineLen = 0;
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = ch;
    }
  }
  if (millis() - lastCmdMs > TIMEOUT_MS) allOff();
}

// MicroStart test for JSumo XMotion V3. Signal pin = A0 (found by pin scan on this board;
// published XMotion table says D10, but on V3 D10 is the left motor PWM).
// MicroStart: rests LOW, Start -> HIGH, Stop -> LOW (then power must be reapplied).
const uint8_t PIN_START = A0;
const uint8_t MOTOR[] = {10, 11, 12, 13};   // held LOW so motors cannot spin
uint8_t last;

void setup() {
  for (uint8_t m : MOTOR) { pinMode(m, OUTPUT); digitalWrite(m, LOW); }
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  pinMode(PIN_START, INPUT_PULLUP);
  last = digitalRead(PIN_START);
}

void loop() {
  static uint32_t t;                  // heartbeat so the monitor shows it is alive
  if (millis() - t >= 1000) { t = millis(); Serial.print(F("A0=")); Serial.println(last ? F("HIGH") : F("LOW")); }
  uint8_t v = digitalRead(PIN_START);
  if (v == last) return;
  delay(5);                           // debounce
  if (digitalRead(PIN_START) != v) return;
  last = v;
  Serial.println(v ? F(">>> TRIGGERED: START (A0 HIGH)") : F(">>> TRIGGERED: EMERGENCY STOP (A0 LOW)"));
}

/*
 * JSumo XPulse - 16-Sensor Multiplexer Test
 * Board: Tools > Board > "Arduino Leonardo"
 * Serial Monitor: 115200 baud
 */

const uint8_t PIN_MUX_S0  = A2;  // Mux select bit 0 (LSB)
const uint8_t PIN_MUX_S1  = A4;  // Mux select bit 1
const uint8_t PIN_MUX_S2  = A5;  // Mux select bit 2
const uint8_t PIN_MUX_S3  = 4;   // Mux select bit 3 (MSB)
const uint8_t PIN_MUX_SIG = A1;  // Multiplexer signal output

const uint8_t PIN_LED_L   = 8;
const uint8_t PIN_LED_R   = 9;

// Motor pins (kept LOW so robot stays stationary)
const uint8_t PIN_L_PWM   = 10;
const uint8_t PIN_L_DIR   = 12;
const uint8_t PIN_R_PWM   = 11;
const uint8_t PIN_R_DIR   = 13;

uint8_t sensorVal[16];

void selectMuxChannel(uint8_t ch) {
  digitalWrite(PIN_MUX_S0, (ch & 0x01) ? HIGH : LOW);
  digitalWrite(PIN_MUX_S1, (ch & 0x02) ? HIGH : LOW);
  digitalWrite(PIN_MUX_S2, (ch & 0x04) ? HIGH : LOW);
  digitalWrite(PIN_MUX_S3, (ch & 0x08) ? HIGH : LOW);
  delayMicroseconds(25); // Allow multiplexer output to settle
}

void setup() {
  pinMode(PIN_MUX_S0, OUTPUT);
  pinMode(PIN_MUX_S1, OUTPUT);
  pinMode(PIN_MUX_S2, OUTPUT);
  pinMode(PIN_MUX_S3, OUTPUT);
  pinMode(PIN_MUX_SIG, INPUT);

  pinMode(PIN_LED_L, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);

  // Keep motors completely off
  pinMode(PIN_L_PWM, OUTPUT); pinMode(PIN_L_DIR, OUTPUT);
  pinMode(PIN_R_PWM, OUTPUT); pinMode(PIN_R_DIR, OUTPUT);
  digitalWrite(PIN_L_DIR, LOW); analogWrite(PIN_L_PWM, 0);
  digitalWrite(PIN_R_DIR, LOW); analogWrite(PIN_R_PWM, 0);

  Serial.begin(115200);
}

void loop() {
  bool leftTrigger = false;
  bool rightTrigger = false;

  // Read all 16 channels (0..14 = front V array, 15 = bottom centre)
  for (uint8_t ch = 0; ch < 16; ch++) {
    selectMuxChannel(ch);
    sensorVal[ch] = digitalRead(PIN_MUX_SIG);
  }

  // Front V array printed ch 14 (left) .. ch 0 (right), ch 7 = top centre
  Serial.print(F("FRONT [L->R]: "));
  for (int8_t i = 14; i >= 0; i--) {
    Serial.print(sensorVal[i]);
    Serial.print(' ');
    if (sensorVal[i] == LOW) {
      if (i >= 7) leftTrigger = true;
      if (i <= 7) rightTrigger = true;
    }
  }

  // Bottom centre (ch 15), straight-line sensor: lights both LEDs
  Serial.print(F(" | BOTTOM [15]: "));
  Serial.println(sensorVal[15]);
  if (sensorVal[15] == LOW) { leftTrigger = true; rightTrigger = true; }

  digitalWrite(PIN_LED_L, leftTrigger ? HIGH : LOW);
  digitalWrite(PIN_LED_R, rightTrigger ? HIGH : LOW);

  delay(80);
}

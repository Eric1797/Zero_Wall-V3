/*
 * JSumo XPulse - Simple Line Follower (PD control)
 * Board: Tools > Board > "Arduino Leonardo"
 * Serial Monitor: 115200 baud
 *
 * Built from ir_array_test (16-ch mux, LOW = black line) and the
 * motor pins used there (PWM + DIR per side).
 *
 * Front sensors: ch 14 = far LEFT ... ch 7 = centre ... ch 0 = far RIGHT
 *
 * TUNING (do this with the robot lifted off the ground first):
 *  1. If a wheel spins the wrong way, flip LEFT_FWD / RIGHT_FWD.
 *  2. If it steers away from the line, change STEER_SIGN to -1.
 *  3. Raise BASE_SPEED once it follows reliably; tune KP, then KD.
 */

// ---------- Pins ----------
const uint8_t PIN_MUX_S0  = A2;
const uint8_t PIN_MUX_S1  = A4;
const uint8_t PIN_MUX_S2  = A5;
const uint8_t PIN_MUX_S3  = 4;
const uint8_t PIN_MUX_SIG = A1;

const uint8_t PIN_LED_L   = 8;
const uint8_t PIN_LED_R   = 9;

const uint8_t PIN_L_PWM   = 10;
const uint8_t PIN_L_DIR   = 12;
const uint8_t PIN_R_PWM   = 11;
const uint8_t PIN_R_DIR   = 13;

// ---------- Settings ----------
const uint8_t LEFT_FWD    = HIGH;   // DIR level that drives left motor forward
const uint8_t RIGHT_FWD   = HIGH;   // DIR level that drives right motor forward
const int8_t  STEER_SIGN  = 1;      // flip to -1 if robot turns away from line

const int   BASE_SPEED    = 64;     // 0-255 (~25% duty)
const float KP            = 5.0;    // proportional gain: PWM per sensor of error
const float KD            = 0.15;   // derivative gain: PWM per (error/second), on a filtered rate
const float KI            = 2.0;    // integral gain (per error-second); 0 = PD only
const float INTEGRAL_MAX  = 4.0;    // anti-windup limit on the error-seconds sum
const float D_FILTER_S    = 0.030;  // derivative low-pass time constant (seconds)
const int   SEARCH_SPEED  = 64;     // spin speed when the line is lost

// ---------- State ----------
float lastError = 0;
float dFilt     = 0;
float integral  = 0;
uint32_t lastUs = 0;

void selectMuxChannel(uint8_t ch) {
  digitalWrite(PIN_MUX_S0, (ch & 0x01) ? HIGH : LOW);
  digitalWrite(PIN_MUX_S1, (ch & 0x02) ? HIGH : LOW);
  digitalWrite(PIN_MUX_S2, (ch & 0x04) ? HIGH : LOW);
  digitalWrite(PIN_MUX_S3, (ch & 0x08) ? HIGH : LOW);
  delayMicroseconds(25);
}

// speed: -255..255 (negative = reverse)
void setMotors(int left, int right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);

  digitalWrite(PIN_L_DIR, (left  >= 0) ? LEFT_FWD  : !LEFT_FWD);
  digitalWrite(PIN_R_DIR, (right >= 0) ? RIGHT_FWD : !RIGHT_FWD);
  analogWrite(PIN_L_PWM, abs(left));
  analogWrite(PIN_R_PWM, abs(right));
}

void setup() {
  pinMode(PIN_MUX_S0, OUTPUT);
  pinMode(PIN_MUX_S1, OUTPUT);
  pinMode(PIN_MUX_S2, OUTPUT);
  pinMode(PIN_MUX_S3, OUTPUT);
  pinMode(PIN_MUX_SIG, INPUT);

  pinMode(PIN_LED_L, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);

  pinMode(PIN_L_PWM, OUTPUT); pinMode(PIN_L_DIR, OUTPUT);
  pinMode(PIN_R_PWM, OUTPUT); pinMode(PIN_R_DIR, OUTPUT);
  setMotors(0, 0);

  Serial.begin(115200);
  delay(1000);  // time to put the robot down
}

void loop() {
  // Read the 15 front sensors and compute the line position.
  // Weight: ch 14 (left) = -7 ... ch 7 (centre) = 0 ... ch 0 (right) = +7
  int sum = 0;
  int count = 0;
  for (uint8_t ch = 0; ch < 15; ch++) {
    selectMuxChannel(ch);
    if (digitalRead(PIN_MUX_SIG) == LOW) {   // LOW = black line
      sum += (7 - ch);
      count++;
    }
  }

  digitalWrite(PIN_LED_L, lastError < 0);
  digitalWrite(PIN_LED_R, lastError > 0);

  uint32_t nowUs = micros();
  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;
  if (dt > 0.05f) dt = 0.05f;               // ignore long gaps (e.g. after startup)

  if (count == 0) {
    integral = 0;                            // reset while the line is lost
    dFilt = 0;
    // Line lost: spin toward the side where it was last seen
    if (lastError < 0) setMotors(-SEARCH_SPEED,  SEARCH_SPEED);
    else               setMotors( SEARCH_SPEED, -SEARCH_SPEED);
    return;
  }

  float error = (float)sum / count;          // -7 (far left) .. +7 (far right)

  // Integral: accumulate error over time, clamp it, and clear it when the
  // line crosses the centre (prevents overshoot from wound-up error)
  if ((error > 0 && lastError < 0) || (error < 0 && lastError > 0)) integral = 0;
  integral = constrain(integral + error * dt, -INTEGRAL_MAX, INTEGRAL_MAX);

  // Derivative: real rate (per second), low-pass filtered so single-sensor
  // steps don't kick the motors (this was the main cause of straight-line wobble)
  if (dt < 0.0005f) dt = 0.0005f;
  float rate = (error - lastError) / dt;
  dFilt += (rate - dFilt) * (dt / (D_FILTER_S + dt));

  float correction = STEER_SIGN * (KP * error + KI * integral + KD * dFilt);
  correction = constrain(correction, -BASE_SPEED, BASE_SPEED);   // never reverse a wheel while following
  lastError = error;

  // Line on the left (error < 0) -> slow left wheel, speed up right wheel
  setMotors(BASE_SPEED + (int)correction, BASE_SPEED - (int)correction);
}

/*
 * JSumo XPulse - Line Follower v4  (v3 with a ~1.2 kHz control loop)
 * v4 = v3 driving logic, faster I/O (measured on this Leonardo with pid_loop_optimization):
 *   sensor mux, sensor read, LEDs, DIR pins and MicroStart use direct port access instead of
 *   digitalWrite/digitalRead (~5 us per call), mux settle 25 -> 15 us (0 read errors at 10 us,
 *   errors at 5 us), one float divide less per loop. Loop: ~700 Hz (v3) -> ~1200 Hz.
 * Board: Tools > Board > "Arduino Leonardo"
 * Serial Monitor: 115200 baud
 *
 * Same pins, mux reading, sensor order and setMotors() as line_follower.ino:
 *   Front sensors: ch 14 = far LEFT ... ch 7 = centre ... ch 0 = far RIGHT, LOW = black tape.
 * Kept from line_follower.ino: PID with filtered derivative, integral with anti-windup that is
 * cleared when the line crosses the centre, spin toward the last side when the line is lost.
 *
 * Added:
 *  1. Tape pieces: follows only the black piece nearest to where the line was, so it does not
 *     jump to the other side of a hairpin or onto a crossing line and turn back.
 *  2. Wider-than-tape piece (crossing, also one met at an angle, or a sharp corner): holds its
 *     course instead of swerving and remembers which side the tape spread to. If the line is
 *     then lost (sharp corner) it pivots that way at once.
 *  3. Speed planner: MAX_SPEED on straights, slower the more the line is off-centre or moving.
 *     A speed estimate from the motor commands lets it BRAKE with reverse power when it is
 *     faster than the planned speed (tyres can only stop so fast before a corner).
 *  4. Inner wheel may reverse (up to PIVOT_MAX) for tight corners.
 *  5. Short gaps in the line near the centre are driven straight through.
 *  6. MicroStart start module on A0 (see microstart/startstop_instructions.md):
 *     remote START (A0 LOW->HIGH) starts the run, remote STOP (A0 HIGH->LOW) is an EMERGENCY
 *     STOP: motors off and latched until the robot is power-cycled. START_WITH_MODULE = 0 starts
 *     1 s after power-on instead (STOP still works once START was pressed).
 *  7. Straight mode: when the line has stayed near the centre (and the bottom sensor 15 sees it)
 *     for STRAIGHT_CONFIRM_S, steering goes soft (KP x KP_STRAIGHT, KD x KD_STRAIGHT) so it does
 *     not wobble, and it may speed up to BOOST_SPEED. Any real error drops back at once to full
 *     corner gains and the normal speed planner.
 *
 * The simulator (sim/xpulse_sim.py) compiles and runs THIS file. Its Code tab and
 * tools/tune.py change the settings below; "Save into .ino" writes them back here.
 *
 * DIP SWITCHES D5 / D6 / D7 (read at power-on; ON = switch to GND = LOW). LEDs blink the mode number:
 *   all OFF            1 blink  COMPETITION  full speed
 *   DIP1 ON            2 blinks TEST         10% speed (gentle first try on a new track)
 *   DIP2 ON            3 blinks SAFE         70% speed (bad grip / dusty track / first real runs)
 *   DIP3 ON            4 blinks SENSOR CHECK motors never run; LEDs show line left/right,
 *                                            Serial prints the 16 sensors (put robot on the line)
 *   DIP2 + DIP3 ON     5 blinks FAST         110% speed (fresh battery, clean tyres, fan on)
 *   DIP1 + DIP3 ON     6 blinks TECHNICAL    for a hard track: 90 degree corners, tight turns.
 *                                            Brakes earlier, lower corner speed, pivots at once
 *                                            when the line disappears.
 *   any other mix      = TEST (safest)
 *
 * TUNING on the real robot (lift it first):
 *  - Wheel spins the wrong way: flip LEFT_FWD / RIGHT_FWD.
 *  - Steers away from the line: set STEER_SIGN to -1.
 *  - First runs: lower MAX_SPEED (e.g. 140), then raise it step by step.
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

const uint8_t PIN_START   = A0;     // MicroStart signal (rests LOW, START = HIGH, STOP = LOW)
const uint8_t PIN_DIP1    = 5;
const uint8_t PIN_DIP2    = 6;
const uint8_t PIN_DIP3    = 7;

const uint8_t MUX_SETTLE_US = 15;   // wait after switching the sensor mux (v3: 25). 5 us gives wrong reads

// Fast pin access. Leonardo: A0 = PF7, A1 = PF6, A2 = PF5, A4 = PF1, A5 = PF0, D4 = PD4,
// D8 = PB4, D9 = PB5, D12 = PD6, D13 = PC7. If you change a PIN_ above, change these too.
#ifdef __AVR__
inline bool muxBlack()   { return !(PINF & _BV(6)); }   // LOW = black tape
inline uint8_t startPin() { return (PINF >> 7) & 1; }
inline void ledL(bool v) { if (v) PORTB |= _BV(4); else PORTB &= ~_BV(4); }
inline void ledR(bool v) { if (v) PORTB |= _BV(5); else PORTB &= ~_BV(5); }
inline void dirL(bool v) { if (v) PORTD |= _BV(6); else PORTD &= ~_BV(6); }
inline void dirR(bool v) { if (v) PORTC |= _BV(7); else PORTC &= ~_BV(7); }
#else  // PC simulator
inline bool muxBlack()   { return digitalRead(PIN_MUX_SIG) == LOW; }
inline uint8_t startPin() { return digitalRead(PIN_START); }
inline void ledL(bool v) { digitalWrite(PIN_LED_L, v); }
inline void ledR(bool v) { digitalWrite(PIN_LED_R, v); }
inline void dirL(bool v) { digitalWrite(PIN_L_DIR, v); }
inline void dirR(bool v) { digitalWrite(PIN_R_DIR, v); }
#endif

// ---------- Settings ----------
const uint8_t LEFT_FWD    = HIGH;   // DIR level that drives left motor forward
const uint8_t RIGHT_FWD   = HIGH;   // DIR level that drives right motor forward
const int8_t  STEER_SIGN  = 1;      // flip to -1 if robot turns away from line
const int     START_WITH_MODULE = 1; // 1 = run starts on remote START; 0 = start 1 s after power-on
const float   TEST_SCALE  = 0.10;   // DIP1: speed x 10%
const float   SAFE_SCALE  = 0.70;   // DIP2: speed x 70%
const float   FAST_SCALE  = 1.10;   // DIP2+DIP3: speed x 110%
const float   TECH_SCALE  = 0.90;   // DIP1+DIP3: straight speed x 90%
const float   TECH_SLOW   = 1.6;    // DIP1+DIP3: slows down this much harder for the same error
const float   TECH_MIN    = 0.75;   // DIP1+DIP3: tightest-curve speed x 75%
const int     MIN_MOVE_PWM = 45;    // never plan less than this (motors stall below it)

const int   MAX_SPEED     = 194;    // 0-255, speed on straights
const int   MIN_SPEED     = 113;     // 0-255, planned speed in the tightest curves
const float SLOW_PER_ERR  = 20.2574;   // PWM taken off MAX_SPEED per sensor of error
const float SLOW_PER_RATE = 0.1934;   // PWM taken off per (sensor/second) of line movement
const float ACCEL_PER_S   = 1215.325;  // how fast the planned speed may rise (PWM per second)
const float BRAKE_GAIN    = 1.4014;    // reverse power per PWM of over-speed
const int   BRAKE_MAX     = 255;    // most reverse power used for braking
const float SPEED_TAU_S   = 0.0974;   // robot speed time constant, for the speed estimate
const float KP            = 37.8207;   // PWM per sensor of error
const float KD            = 2.8225;    // PWM per (error/second), on a filtered rate
const float KI            = 0.7032;    // integral gain (per error-second); 0 = PD only
const float INTEGRAL_MAX  = 3.3837;    // anti-windup limit on the error-seconds sum
const float D_FILTER_S    = 0.0061;  // derivative low-pass time constant (seconds)
const int   PIVOT_MAX     = 231;    // how far the inner wheel may reverse in tight turns
const int   SEARCH_SPEED  = 76;    // pivot speed when the line is lost
const float GAP_S         = 0.0536;  // line gaps shorter than this are driven straight through
const int   WIDE_W        = 5;       // a piece this many sensors wide = crossing / sharp corner
const float CROSS_CURVE   = 0.5678;    // over a crossing: 0 = drive straight, 1 = keep the curve it had
const float HINT_S        = 0.3237;   // how long a "tape spread left/right" hint is trusted (s)
const int   BOOST_SPEED   = 194;    // 0-255, speed on confirmed straights
const float STRAIGHT_ERR  = 1.3987;    // |error| below this (sensors) counts as "on a straight"
const float STRAIGHT_RATE = 7.5186;    // ...and the line moves slower than this (sensors/second)
const float STRAIGHT_CONFIRM_S = 0.0868;  // must hold that long before straight mode
const float KP_STRAIGHT   = 0.4281;    // KP multiplier in straight mode (soft = no wobble)
const float KD_STRAIGHT   = 1.185;    // KD multiplier in straight mode (damps small swings)

// ---------- State ----------
float lastError = 0;
float dFilt     = 0;
float integral  = 0;
float speed     = 0;       // planned base PWM (negative = braking)
float vEst      = 0;       // estimated robot speed in PWM units
float cmdL = 0, cmdR = 0;  // last motor commands
float lostS     = 0;
int8_t hint     = 0;       // side a wide piece spread to: -1 left, +1 right
float hintS     = 99;      // seconds since that hint
float straightS = 0;       // how long the robot has looked like it is on a straight
float turnCorr = 0, turnSp = 1;  // steering and speed before a crossing (= how tight it was turning)
bool running    = false;
bool stopped    = false;
uint32_t runStartUs = 0, loopCount = 0;  // measured control-loop rate, printed after STOP   // emergency stop latched (until power-cycle)
uint8_t startLevel = LOW;  // debounced MicroStart level
enum Mode { M_COMP = 1, M_TEST, M_SAFE, M_SENSOR, M_FAST, M_TECH };
uint8_t mode = M_COMP;
float speedScale = 1.0;
uint32_t lastUs = 0;

void selectMuxChannel(uint8_t ch) {
#ifdef __AVR__
  uint8_t f = PORTF & ~(_BV(5) | _BV(1) | _BV(0));      // S0 = A2, S1 = A4, S2 = A5 in one write
  if (ch & 0x01) f |= _BV(5);
  if (ch & 0x02) f |= _BV(1);
  if (ch & 0x04) f |= _BV(0);
  PORTF = f;
  if (ch & 0x08) PORTD |= _BV(4); else PORTD &= ~_BV(4); // S3 = D4
#else
  digitalWrite(PIN_MUX_S0, (ch & 0x01) ? HIGH : LOW);
  digitalWrite(PIN_MUX_S1, (ch & 0x02) ? HIGH : LOW);
  digitalWrite(PIN_MUX_S2, (ch & 0x04) ? HIGH : LOW);
  digitalWrite(PIN_MUX_S3, (ch & 0x08) ? HIGH : LOW);
#endif
  delayMicroseconds(MUX_SETTLE_US);
}

// speed: -255..255 (negative = reverse)
void setMotors(int left, int right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);
  cmdL = left;
  cmdR = right;

  dirL((left  >= 0) ? LEFT_FWD  : !LEFT_FWD);
  dirR((right >= 0) ? RIGHT_FWD : !RIGHT_FWD);
  analogWrite(PIN_L_PWM, abs(left));
  analogWrite(PIN_R_PWM, abs(right));
}

void setup() {
  pinMode(PIN_MUX_S0, OUTPUT);
  pinMode(PIN_MUX_S1, OUTPUT);
  pinMode(PIN_MUX_S2, OUTPUT);
  pinMode(PIN_MUX_S3, OUTPUT);
  pinMode(PIN_MUX_SIG, INPUT);
  pinMode(PIN_START, INPUT_PULLUP);   // module drives it through 1k, overriding the pull-up
  pinMode(PIN_DIP1, INPUT_PULLUP);
  pinMode(PIN_DIP2, INPUT_PULLUP);
  pinMode(PIN_DIP3, INPUT_PULLUP);

  pinMode(PIN_LED_L, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);

  pinMode(PIN_L_PWM, OUTPUT); pinMode(PIN_L_DIR, OUTPUT);
  pinMode(PIN_R_PWM, OUTPUT); pinMode(PIN_R_DIR, OUTPUT);
  setMotors(0, 0);

  Serial.begin(115200);

  // ---- mode from the DIP switches ----
  delay(5);
  bool d1 = digitalRead(PIN_DIP1) == LOW, d2 = digitalRead(PIN_DIP2) == LOW, d3 = digitalRead(PIN_DIP3) == LOW;
  if (!d1 && !d2 && !d3)     { mode = M_COMP;   speedScale = 1.0; }
  else if (d1 && !d2 && !d3) { mode = M_TEST;   speedScale = TEST_SCALE; }
  else if (!d1 && d2 && !d3) { mode = M_SAFE;   speedScale = SAFE_SCALE; }
  else if (!d1 && !d2 && d3) { mode = M_SENSOR; speedScale = 0; }
  else if (!d1 && d2 && d3)  { mode = M_FAST;   speedScale = FAST_SCALE; }
  else if (d1 && !d2 && d3)  { mode = M_TECH;   speedScale = TECH_SCALE; }
  else                       { mode = M_TEST;   speedScale = TEST_SCALE; }
  for (uint8_t k = 0; k < mode; k++) {       // blink the mode number
    ledL(HIGH); ledR(HIGH); delay(150);
    ledL(LOW);  ledR(LOW);  delay(150);
  }

  startLevel = digitalRead(PIN_START);
  if (!START_WITH_MODULE && mode != M_SENSOR) {
    delay(1000);  // time to put the robot down
    running = true;
  }
  lastUs = micros();
}

// speeds scaled by the DIP mode; never below what the motors need to turn
float scaled(float pwm) {
  if (pwm <= 0) return pwm;
  return max(pwm * speedScale, (float)MIN_MOVE_PWM);
}

// MicroStart: START (LOW->HIGH) runs, STOP (HIGH->LOW) = emergency stop, latched.
void serviceStart() {
  uint8_t v = startPin();
  if (v == startLevel) return;
  delayMicroseconds(300);                    // debounce
  if (startPin() != v) return;
  startLevel = v;
  if (v == HIGH) {
    if (!stopped && mode != M_SENSOR && !running) {
      running = true;
      lastError = dFilt = integral = speed = vEst = 0;
      hintS = 99;
      runStartUs = micros();
      loopCount = 0;                         // no Serial while driving: it can block on USB
    }
  } else {
    stopped = true;                          // only a power-cycle clears this
    running = false;
    setMotors(0, 0);                         // motors off first, then report
    if (Serial) {
      Serial.println(F("EMERGENCY STOP"));
      if (loopCount) {
        Serial.print(F("control loop: "));
        Serial.print(loopCount * 1e6f / (micros() - runStartUs), 0);
        Serial.println(F(" Hz"));
      }
    }
  }
}

void sensorCheck() {
  static uint32_t lastMs = 0;
  bool on[16];
  for (uint8_t ch = 0; ch < 16; ch++) { selectMuxChannel(ch); on[ch] = muxBlack(); }
  bool left = false, right = false;
  for (uint8_t ch = 0; ch < 15; ch++) if (on[ch]) { if (ch >= 7) left = true; if (ch <= 7) right = true; }
  ledL(left);
  ledR(right);
  if (millis() - lastMs > 150) {
    lastMs = millis();
    Serial.print(F("L |"));
    for (int8_t ch = 14; ch >= 0; ch--) Serial.print(on[ch] ? '#' : '.');
    Serial.print(F("| R   back:"));
    Serial.print(on[15] ? '#' : '.');
    Serial.print(F("   start A0:"));
    Serial.println(startLevel ? F("HIGH") : F("LOW"));
  }
}

void loop() {
  // ---- start / emergency stop (checked every loop, before any motor output) ----
  serviceStart();
  if (mode == M_SENSOR) { setMotors(0, 0); sensorCheck(); return; }
  if (!running) {
    setMotors(0, 0);
    if (stopped) {                               // emergency stop: both LEDs blink
      bool ph = (millis() / 200) & 1;
      ledL(ph); ledR(ph);
    }
    lastUs = micros();
    return;
  }

  // on[i]: sensor at position i-7 sees tape (i = 0 far LEFT = ch 14 ... i = 14 far RIGHT = ch 0)
  bool on[15];
  for (uint8_t ch = 0; ch < 15; ch++) {
    selectMuxChannel(ch);
    on[14 - ch] = (muxBlack());   // LOW = black tape
  }
  // a single dark gap inside black (V-bar meeting a crossing at an angle) is part of that black
  for (uint8_t k = 1; k < 14; k++) if (!on[k] && on[k - 1] && on[k + 1]) on[k] = true;
  selectMuxChannel(15);
  bool backOn = (muxBlack());     // bottom centre sensor (ch 15)

  loopCount++;
  uint32_t nowUs = micros();
  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;
  dt = constrain(dt, 0.0005f, 0.05f);

  // speed estimate (PWM units): the robot follows the average motor command with a lag
  vEst += ((cmdL + cmdR) * 0.5f - vEst) * constrain(dt / SPEED_TAU_S, 0.0f, 1.0f);

  // Split the readings into separate tape pieces and keep the one nearest to where the line was
  int8_t bestLo = -1, bestHi = -1;
  float bestDist = 99;
  uint8_t i = 0;
  while (i < 15) {
    if (on[i]) {
      uint8_t j = i;
      while (j + 1 < 15 && on[j + 1]) j++;
      float d = fabs((i + j) / 2.0f - 7 - lastError);
      if (d < bestDist) { bestDist = d; bestLo = i; bestHi = j; }
      i = j + 1;
    } else {
      i++;
    }
  }

  if (bestLo < 0) {
    // ---- line lost ----
    lostS += dt;
    hintS += dt;
    straightS = 0;
    integral = 0;                                          // as in line_follower.ino
    dFilt = 0;
    ledL(HIGH);
    ledR(HIGH);
    bool hinted = hintS < HINT_S;
    if (!hinted && mode != M_TECH && lostS < GAP_S && fabs(lastError) < 3) {  // short gap: keep going straight
      setMotors((int)speed, (int)speed);
      return;
    }
    speed = 0;
    bool left = hinted ? (hint < 0) : (lastError < 0);     // sharp corner: turn the way the tape spread
    int sp = (int)scaled(SEARCH_SPEED);
    if (left) setMotors(-sp,  sp);                         // spin left
    else      setMotors( sp, -sp);
    return;
  }

  lostS = 0;
  hintS += dt;
  float error;
  bool wide = bestHi - bestLo + 1 >= WIDE_W;
  if (wide) {
    // Wider than tape: a crossing (also one met at an angle) or a sharp corner.
    // Hold course instead of chasing it, and remember which side it spread to.
    float lastI = lastError + 7;
    float extL = lastI - bestLo, extR = bestHi - lastI;
    if (fabs(extL - extR) >= 2) {
      hint = (extL > extR) ? -1 : 1;
      hintS = 0;
    }
    error = lastError;
  } else {
    error = (bestLo + bestHi) / 2.0f - 7;   // -7 (far left) .. +7 (far right)
  }

  // Integral (from line_follower.ino): cleared when the line crosses the centre, clamped
  if ((error > 0 && lastError < 0) || (error < 0 && lastError > 0)) integral = 0;
  integral = constrain(integral + error * dt, -INTEGRAL_MAX, INTEGRAL_MAX);

  // Derivative: real rate (per second), low-pass filtered
  float rate = (error - lastError) / dt;
  dFilt += (rate - dFilt) * (dt / (D_FILTER_S + dt));
  lastError = error;

  ledL(error < 0);
  ledR(error > 0);

  // ---- straight or corner? ----
  bool calm = fabs(error) < STRAIGHT_ERR && fabs(dFilt) < STRAIGHT_RATE && backOn;
  straightS = calm ? straightS + dt : 0;
  bool straight = straightS > STRAIGHT_CONFIRM_S;

  // ---- speed planner ----
  float slowK = (mode == M_TECH) ? TECH_SLOW : 1.0f;
  float minSp = (mode == M_TECH) ? MIN_SPEED * TECH_MIN : MIN_SPEED;
  float target = MAX_SPEED - slowK * (SLOW_PER_ERR * fabs(error) + SLOW_PER_RATE * fabs(dFilt));
  target = constrain(target, minSp, (float)MAX_SPEED);
  if (straight) target = BOOST_SPEED;
  target = scaled(target);                                 // DIP mode speed
  if (vEst > target + 10) {
    speed = target - BRAKE_GAIN * (vEst - target);         // too fast: brake (reverse power)
    if (speed < -BRAKE_MAX) speed = -BRAKE_MAX;
  } else if (target > speed) {
    speed = min(target, speed + ACCEL_PER_S * dt);         // speed up gently
  } else {
    speed = target;
  }

  float kp = straight ? KP * KP_STRAIGHT : KP;   // soft on straights, full in corners
  float kd = straight ? KD * KD_STRAIGHT : KD;
  // steering strength follows the DIP speed (gains were tuned at full speed)
  float gainScale = constrain(max(speedScale, (float)MIN_MOVE_PWM / MAX_SPEED), 0.0f, 1.0f);
  float corr = STEER_SIGN * (kp * error + KI * integral + kd * dFilt) * gainScale;
  float lim = fabs(speed) + PIVOT_MAX;
  // Over a crossing keep the same CURVE as before it (not the same steering value: at low speed
  // the same steering would turn much tighter and swing onto the crossing line).
  float sp = max(fabs(speed), 30.0f);
  if (wide) corr = turnCorr / turnSp * sp * CROSS_CURVE;   // divide only over a crossing
  else      { turnCorr = corr; turnSp = sp; }
  corr = constrain(corr, -lim, lim);                       // inner wheel may reverse in tight turns

  // Line on the left (error < 0) -> slow/reverse left wheel, speed up right wheel
  setMotors((int)(speed + corr), (int)(speed - corr));
}

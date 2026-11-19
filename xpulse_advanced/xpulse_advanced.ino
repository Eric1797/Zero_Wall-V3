/*
 * ============================================================================
 *  XPULSE  -  ROBOTEX LINE FOLLOWING  -  AUM track edition
 * ============================================================================
 *  Robot  : JSumo XPulse  (XMotion V3 controller = ATmega32U4 = Arduino Leonardo,
 *           XLine board: 15 front sensors in a V + 1 rear centre sensor,
 *           read through a 16-channel multiplexer, 3S LiPo, suction fan)
 *  IDE    : Arduino IDE  ->  Tools > Board > "Arduino Leonardo"
 *
 *  How it drives
 *   - Reads all 16 sensors about 1000 times per second.
 *   - Finds every black patch under the sensor bar and keeps following the
 *     SAME line it was already on. That is how it ignores the side branch at
 *     the Y-merge, goes straight over the crossing and is not fooled by the
 *     start / finish marks.
 *   - PD steering whose gains change with speed.
 *   - Speed planner: boost on straights, slows down in curves, brakes with
 *     reverse power before sharp corners, pivots on 90 degree / acute corners.
 *   - Start / stop with the MicroStart module (Sony remote) or the board button.
 *   - Optional auto-stop at the finish mark.
 *   - USB console: change speeds and gains without re-uploading (saved in EEPROM).
 *   - Event log of the last run (corners, crossings, marks, lost line ...).
 *
 *  >>> TEST BUILD @ 30% speed, adapted to the proven simple follower:
 *      raw motor pins (L_PWM = D10), ch 14 = LEFT, black = LOW, no start button
 *      (D10 is a motor) -> AUTO START (3 s on the line). Lift the robot to stop it.
 *      Gains scale with speed. Raise DEFAULT_SPEED_PCT (or "set speed N") step by step.
 *
 *  VERSION 3: motors through the JSumo XMotion library (xmotion.MotorControl),
 *  start/stop on D10 (the XMotion "button & start" input - A0 is the battery
 *  sensor), full speed (the 9.5 s simulator setting).
 *  Needs the XMotion library: jsumo.com/xmotion -> Sketch > Include Library > Add .ZIP
 *  Upload -> both LEDs blink 3x -> right LED on = waiting.
 *  AUTO START: put the robot on the line and leave it - LEDs blink faster,
 *  after 3 s it drives. (Lift it after a run to re-arm.) Button / remote also work.
 *  Do not touch the button while switching on (the code learns its rest level).
 *
 *  DIP switches are OFF by default (USE_DIP_SWITCHES 0). When enabled:
 *   DIP1 ON : TEST MODE - motors never run, live sensor view on the Serial Monitor
 *   DIP2 ON : SAFE MODE - every speed x 0.6 (use for the first runs)
 *   DIP3 ON : use junction plan B instead of plan A (see section 3)
 *
 *  Serial Monitor: 115200 baud, "Newline". Type  ?  for the command list.
 *  The console only listens while the robot is stopped.
 * ============================================================================
 */

#include <Arduino.h>
#include <EEPROM.h>

// JSumo XMotion library (install once: download from jsumo.com/xmotion, then
// Sketch > Include Library > Add .ZIP Library). Motors are driven through
// xmotion.MotorControl(), JSumo's own driver code for this board.
#ifndef USE_XMOTION_LIB
#define USE_XMOTION_LIB 0   // raw PWM/DIR pins = same method as the proven simple follower
#endif
#if USE_XMOTION_LIB
#include <xmotion.h>
#endif
#include <stdlib.h>
#include <string.h>
#include <math.h>

// ============================================================================
// 1. PINS  - taken from YOUR working sample code ("PID code 1")
// ============================================================================
const uint8_t PIN_MUX_S0  = A2;   // multiplexer select bit 0 (LSB)
const uint8_t PIN_MUX_S1  = A4;   // select bit 1
const uint8_t PIN_MUX_S2  = A5;   // select bit 2
const uint8_t PIN_MUX_S3  = 4;    // select bit 3 (MSB)
const uint8_t PIN_MUX_SIG = A1;   // XLine digital output through the mux

const uint8_t PIN_L_PWM = 10;     // left motor speed: D10 (proven by the working simple follower)
const uint8_t PIN_L_DIR = 12;     // left motor direction (HIGH = forward)
const uint8_t PIN_R_PWM = 11;     // right motor speed
const uint8_t PIN_R_DIR = 13;     // right motor direction (HIGH = forward)

const int8_t  PIN_START = -1;     // D10 is the LEFT MOTOR PWM on this board -> no start button (-1 = off). Use AUTO_START.
                                  // NOTE: A0 is the battery-voltage sensor, NOT a start input.
const uint8_t PIN_TRIM  = A3;     // trimpot on the XMotion board
const uint8_t PIN_DIP1  = 5;
const uint8_t PIN_DIP2  = 6;
const uint8_t PIN_DIP3  = 7;
const uint8_t PIN_LED_L = 8;
const uint8_t PIN_LED_R = 9;

const int8_t  PIN_FAN   = -1;     // -1 = fan is wired straight to the battery.
                                  // Put a PWM pin here if the board switches it.
const uint8_t FAN_PWM        = 255;
const uint16_t FAN_SPINUP_MS = 300;

// ============================================================================
// 2. HARDWARE OPTIONS
// ============================================================================
#define L_MOTOR_REVERSED   0   // 1 if the left wheel spins backwards on "forward"
#define R_MOTOR_REVERSED   0
#ifndef SENSORS_MIRRORED
#define SENSORS_MIRRORED   0   // 0 = ch 0 RIGHT, ch 14 LEFT (proven by the working simple follower)
#endif
                               // (check with DIP1 test mode: hold the line under
                               //  the right side - the '#' must appear on the right)
#ifndef LINE_LEVEL_AUTO
#define LINE_LEVEL_AUTO    0   // 1 = learn black/white polarity when started (robot on the line)
#endif
#ifndef LINE_LEVEL_FIXED
#define LINE_LEVEL_FIXED   LOW  // black line reads LOW (proven by the working simple follower) // your sample code: BlackIsHigh = true
#endif
#ifndef USE_DIP_SWITCHES
#define USE_DIP_SWITCHES   0   // 0 = ignore DIP1-3 (a DIP left ON used to block the motors)
#endif
#ifndef AUTO_START
#define AUTO_START         1   // 1 = put the robot on the line and it starts by itself
#endif
#define AUTO_START_MS      3000 // ...after the line has been seen steadily for this long
#define DEFAULT_SPEED_PCT  30  // overall speed % (the 9.5 s simulator setting); "set speed 80" to calm it
#define USE_TRIMPOT        0   // 1 = trimpot scales all speeds (60% .. 100%); 0 = full speed
#define MATCH_PWM_FREQ     0   // not needed: D3 and D11 already share the same timer
#define START_DELAY_MS     0   // extra wait after the start signal
#define MIN_BASE_PWM       35  // motors stall below this, never plan a slower base speed
#define USE_BACK_SENSOR    1   // 1 = boost only when the rear centre sensor is on the line

const uint8_t SENSOR_BACK = 15;    // rear centre sensor (channel 15)
const uint8_t N_FRONT     = 15;    // channels 0..14 = front V, 0 = far right

// ============================================================================
// 3. TRACK / ROUTE
// ----------------------------------------------------------------------------
// The AUM track (from your photos/video):
//   start straight (dead end + start mark) -> jog -> long straight -> lobe ->
//   Y-MERGE into the wiggle column -> long straight -> U-turn -> acute corner ->
//   zig-zags -> CROSSING -> loop -> CROSSING again -> 90 degree corner area
//   with the second mark -> top straight -> back into the same Y-merge.
// Driving in that direction every junction is a MERGE (two lines join) and the
// crossing is always "go straight", so no decision is needed: plan A is empty.
//
// A FORK is a line that splits in two in front of the robot (only happens if
// the route is driven the other way, up the wiggle column into the Y).
// For each fork, in order, choose GO_LEFT / GO_RIGHT / GO_STRAIGHT.
// Plan B (DIP3 ON) = reverse direction: at the Y take the RIGHT branch
// (towards the lobe and the dead-end straight).
// ============================================================================
// Each fork also has an "earliest time" (ms after the start, at full speed):
// a fork-looking pattern before that time is ignored. This stops a sharp
// corner or a crossing seen at an angle from using up the plan. The time is
// stretched automatically when the trimpot / safe mode make the robot slower.
enum { GO_STRAIGHT = 0, GO_LEFT = 1, GO_RIGHT = 2 };
const uint8_t  PLAN_A[]        = { GO_STRAIGHT };
const uint16_t PLAN_A_MIN_MS[] = { 0 };
const uint8_t  PLAN_A_LEN      = 0;        // 0 = no forks expected
const uint8_t  PLAN_B[]        = { GO_RIGHT };
const uint16_t PLAN_B_MIN_MS[] = { 5500 }; // the Y comes ~7.5 s after the start
const uint8_t  PLAN_B_LEN      = 1;

// ============================================================================
// 4. TUNING  (defaults - can be changed over USB with  set <name> <value>)
//    Speeds are motor PWM 0..255. Error is -1 (line far left) .. +1 (far right)
// ============================================================================
struct Params {
  float vBoost;     // "boost"  speed on confirmed straights
  float vMax;       // "vmax"   speed on gentle curves
  float vCurve;     // "vcurve" speed in the tightest curves
  float vCorner;    // "vcorner" speed when a 90/acute corner is detected
  float vStart;     // "vstart" speed limit during the first 'startMs'
  float startMs;    // "startms"
  float kpLo;       // "kplo"   P gain at speed 0
  float kpHi;       // "kphi"   P gain at speed 255
  float kdLo;       // "kdlo"   D gain at speed 0   (PWM per (error/s))
  float kdHi;       // "kdhi"   D gain at speed 255
  float accel;      // "accel"  max PWM increase per ms (anti wheel-spin)
  float brakeGain;  // "bgain"  how hard to brake when too fast
  float brakeMax;   // "bmax"   max reverse PWM used for braking
  float pivotFwd;   // "pfwd"   outer wheel in a pivot turn
  float pivotRev;   // "prev"   inner wheel (reverse) in a pivot turn
  float revLimit;   // "rlim"   max reverse PWM any wheel may get in normal following
  float tauMs;      // "tau"    robot speed time-constant model (ms)
  float curv0;      // "c0"     curve level where slowing starts
  float curv1;      // "c1"     curve level where vCurve is reached
  float curvD;      // "cd"     how much error-rate counts as curve
  float stopFinish; // "stop"   1 = stop at the finish mark, 0 = keep going
  float finishArm;  // "arm"    ignore finish marks before this many ms
  float lostStopMs; // "lost"   stop if the line is lost this long (ms)
  float muxUs;      // "mux"    multiplexer settle time (us)
  float adapt;      // "adapt"  1 = slow down 8% after a bad overshoot (comes back 2%/s)
  float speedPct;   // "speed"  overall speed in % (applied on top of the trimpot)
};

const Params DEFAULTS = {
  // values found by simulating thousands of runs on the AUM track map
  /*vBoost*/ 215, /*vMax*/ 180, /*vCurve*/ 85, /*vCorner*/ 130,
  /*vStart*/ 180, /*startMs*/ 250,
  /*kpLo*/ 100, /*kpHi*/ 115,   // was 160/185: too stiff at low speed -> wobble on straights
  /*kdLo*/ 3.5, /*kdHi*/ 5.0,    // was 4.5/9.5
  /*accel*/ 1.9,
  /*brakeGain*/ 1.8, /*brakeMax*/ 200,
  /*pivotFwd*/ 150, /*pivotRev*/ 5, /*revLimit*/ 180,
  /*tauMs*/ 145,
  /*curv0*/ 0.05, /*curv1*/ 0.78, /*curvD*/ 0.006,
  /*stopFinish*/ 0, /*finishArm*/ 4000, /*lostStopMs*/ 2500,
  /*muxUs*/ 30, /*adapt*/ 1, /*speedPct*/ DEFAULT_SPEED_PCT
};

Params P;                         // live parameters
const uint16_t PARAM_MAGIC = 0x5B00 + sizeof(Params);   // changed on purpose: ignores old saved EEPROM values   // changes if the struct changes

// fixed behaviour constants
const uint16_t LOOP_US            = 1000;  // control period (1 kHz)
const uint8_t  LINE_MAX_W         = 4;     // a normal line covers <= 4 sensors
const float    JUMP_MAX           = 0.45f; // max line jump between samples to call it "same line"
const uint16_t CORNER_MEM_MS      = 250;   // remember a corner signature this long
const uint16_t CROSS_MAX_MS       = 250;
const uint16_t MARKER_BLANK_MS    = 200;   // no mark detection right after a crossing
const uint16_t MARKER_DEBOUNCE_MS = 250;
const uint16_t FORK_HOLD_MS       = 220;
const uint16_t PIVOT_MAX_MS       = 700;
const float    PIVOT_EXIT_POS     = 0.95f;
const uint16_t STRAIGHT_CONFIRM_MS= 60;

// ============================================================================
// 5. STATE
// ============================================================================
enum RunState  { ST_IDLE, ST_RUN, ST_FINISH };
enum DriveMode { M_FOLLOW, M_CROSS, M_PIVOT, M_SEARCH };
enum EvType {
  EV_START = 1, EV_MARK, EV_CROSS, EV_FORK, EV_CORNER, EV_PIVOT, EV_LOST,
  EV_FOUND, EV_FINISH, EV_STOPPED, EV_LINELOST_STOP, EV_JUMP
};

RunState  state = ST_IDLE;
DriveMode mode  = M_FOLLOW;

uint16_t  rawMask   = 0;      // 16 bits, bit i = sensor i sees the line
uint16_t  frontMask = 0;      // bits 0..14
bool      backOn    = false;
uint8_t   lineLevel = LOW;    // pin level that means "black"
uint8_t   muxCh     = 0;

float POS[N_FRONT];           // lateral position of each front sensor (-1 left .. +1 right)

// tracking
struct Cl { int8_t a, b; };   // one black patch: sensors a..b (a = right-most index)
Cl cl[8];
float lastPos    = 0;         // last good line position
float stablePos  = 0;         // low-pass filtered position
uint8_t nClusters = 0;
uint8_t activeCnt = 0;
int8_t  trkA = -1, trkB = -1; // tracked cluster sensor range
int8_t  extL = 0, extR = 0;   // contiguous black beyond the line, left / right

// controller
float ePrev = 0, dFilt = 0;
float absEF = 0, dAbsF = 0;
float base = 0, vEst = 0;
float spdScale = 1.0f;
float adaptK   = 1.0f;          // automatic slow-down factor (see "adapt")
float cmdL = 0, cmdR = 0;

// timers (ms since run start)
uint32_t runStartMs = 0;
uint32_t lastLoopUs = 0;
uint32_t cornerMs = 0;   int8_t cornerSide = 0;
uint32_t crossMs = 0;    uint32_t crossEnterMs = 0;
uint32_t markerMs = 0;   uint8_t markerFrames = 0;
bool     markCand = false; uint32_t markCandLastMs = 0; uint32_t lastWideMs = 0;
uint32_t forkMs = 0;     int8_t forkSide = 0;
uint32_t modeMs = 0;
uint32_t lostSinceMs = 0;
uint32_t straightSinceMs = 0;
uint8_t  crossFrames = 0, exitFrames = 0, lostFrames = 0, jumpFrames = 0;
int8_t   pivotDir = 0;
uint8_t  forkFrames = 0;
uint8_t  forkIndex = 0;
uint16_t markerCount = 0;
bool     testMode = false, safeMode = false, planB = false;

// start module
bool     startArmed = false;      // button pressed while stopped (pull-up style)
uint8_t  startRest = HIGH;        // level of D10 when nobody touches it (measured at boot)
uint8_t  startStable = HIGH;      // debounced level
uint8_t  startCnt = 0;
uint32_t startEdgeMs = 0;
uint32_t autoSinceMs = 0;         // auto start: line seen steadily since
bool     autoNeedLift = false;    // after a run: lift the robot before auto start re-arms

// event log (ring buffer: keeps the most recent events of the run)
struct Ev { uint16_t t; uint8_t type; int8_t val; };
#ifndef EV_LOG_SIZE
#define EV_LOG_SIZE 64          // 4 bytes RAM each
#endif
const uint16_t EV_MAX = EV_LOG_SIZE;
Ev evLog[EV_MAX];
uint16_t evN = 0;               // total events logged in this run
uint16_t lastRunMs = 0;

// console
struct PDef { const char *name; float *v; };
char     rxBuf[40];
uint8_t  rxLen = 0;
bool     liveView = false;
uint32_t lastViewMs = 0;

// ============================================================================
// 6. SMALL HELPERS
// ============================================================================
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float signf(float v) { return v > 0 ? 1.0f : (v < 0 ? -1.0f : 0.0f); }
static inline uint32_t runMs() { return millis() - runStartMs; }

void logEv(uint8_t type, int8_t val) {
  Ev &e = evLog[evN % EV_MAX];
  e.t = (uint16_t)runMs();
  e.type = type;
  e.val = val;
  evN++;
}

bool dipOn(uint8_t pin) {
#if USE_DIP_SWITCHES
  return digitalRead(pin) == LOW;
#else
  (void)pin; return false;
#endif
}

// ============================================================================
// 7. MOTORS
// ============================================================================
void motorWrite(uint8_t pwmPin, uint8_t dirPin, bool reversed, float cmd) {
  int16_t v = (int16_t)clampf(cmd, -255, 255);
  bool fwd = v >= 0;
  if (reversed) fwd = !fwd;
  digitalWrite(dirPin, fwd ? HIGH : LOW);
  analogWrite(pwmPin, (uint8_t)(v >= 0 ? v : -v));
}

void setMotors(float l, float r) {
  cmdL = clampf(l, -255, 255);
  cmdR = clampf(r, -255, 255);
#if USE_XMOTION_LIB
  int16_t L = (int16_t)cmdL, Rr = (int16_t)cmdR;
  if (L_MOTOR_REVERSED) L = -L;
  if (R_MOTOR_REVERSED) Rr = -Rr;
  xmotion.MotorControl(L, Rr);
#else
  motorWrite(PIN_L_PWM, PIN_L_DIR, L_MOTOR_REVERSED, cmdL);
  motorWrite(PIN_R_PWM, PIN_R_DIR, R_MOTOR_REVERSED, cmdR);
#endif
}

void stopMotors() { setMotors(0, 0); }

void fanOn(bool on) {
  if (PIN_FAN >= 0) analogWrite((uint8_t)PIN_FAN, on ? FAN_PWM : 0);
}

// ============================================================================
// 8. SENSORS
// ============================================================================
// Gray-code order: only ONE select line changes between two readings.
const uint8_t GRAY[16] = {0,1,3,2,6,7,5,4,12,13,15,14,10,11,9,8};

void muxSelect(uint8_t ch) {
  uint8_t diff = ch ^ muxCh;
  if (diff & 1) digitalWrite(PIN_MUX_S0, (ch & 1) ? HIGH : LOW);
  if (diff & 2) digitalWrite(PIN_MUX_S1, (ch & 2) ? HIGH : LOW);
  if (diff & 4) digitalWrite(PIN_MUX_S2, (ch & 4) ? HIGH : LOW);
  if (diff & 8) digitalWrite(PIN_MUX_S3, (ch & 8) ? HIGH : LOW);
  muxCh = ch;
}

// reads all 16 channels; returns bit mask of raw HIGH levels
uint16_t scanRawLevels() {
  uint16_t hi = 0;
  uint8_t settle = (uint8_t)clampf(P.muxUs, 0, 200);
  for (uint8_t k = 0; k < 16; k++) {
    uint8_t ch = GRAY[k];
    muxSelect(ch);
    if (settle) delayMicroseconds(settle);
    if (digitalRead(PIN_MUX_SIG) == HIGH) hi |= (uint16_t)1 << ch;
  }
  return hi;
}

uint8_t popcount16(uint16_t m) { uint8_t c = 0; while (m) { m &= (uint16_t)(m - 1); c++; } return c; }

// decide which pin level means "black": the floor is the majority
void learnLinePolarity() {
#if LINE_LEVEL_AUTO
  uint8_t highs = 0;
  for (uint8_t i = 0; i < 5; i++) {
    uint16_t hi = scanRawLevels() & 0x7FFF;
    highs += popcount16(hi);
  }
  // 5 scans x 15 sensors; floor majority HIGH -> black reads LOW
  lineLevel = (highs > (5 * 15) / 2) ? LOW : HIGH;
#else
  lineLevel = LINE_LEVEL_FIXED;
#endif
}

void readSensors() {
  uint16_t hi = scanRawLevels();
  uint16_t m = (lineLevel == HIGH) ? hi : (uint16_t)~hi;
#if SENSORS_MIRRORED
  uint16_t r = 0;
  for (uint8_t i = 0; i < N_FRONT; i++) if (m & ((uint16_t)1 << i)) r |= (uint16_t)1 << (N_FRONT - 1 - i);
  r |= m & 0x8000;
  m = r;
#endif
  rawMask   = m;
  frontMask = m & 0x7FFF;
  backOn    = (m & 0x8000) != 0;
  activeCnt = popcount16(frontMask);
}

// ============================================================================
// 9. LINE TRACKING  (clusters + continuity)
// ============================================================================
uint8_t findClusters(uint16_t m) {
  uint8_t n = 0; int8_t i = 0;
  while (i < (int8_t)N_FRONT) {
    if (m & ((uint16_t)1 << i)) {
      int8_t a = i;
      while (i < (int8_t)N_FRONT && (m & ((uint16_t)1 << i))) i++;
      if (n < 8) { cl[n].a = a; cl[n].b = i - 1; n++; }
    } else i++;
  }
  return n;
}

// index (float) of a position
static inline float idxOf(float pos) { return 7.0f - pos * 7.0f; }

// distance from a position to a cluster span (0 if inside)
float clDist(const Cl &c, float pos) {
  float pr = POS[c.a], pl = POS[c.b];          // pr >= pl
  if (pos > pr) return pos - pr;
  if (pos < pl) return pl - pos;
  return 0;
}

float clCenter(const Cl &c) { return 0.5f * (POS[c.a] + POS[c.b]); }

// line position inside a cluster: narrow -> centre, wide -> 3-sensor window
// nearest to the prediction (so a side branch does not pull us off)
float linePosIn(const Cl &c, float pred, int8_t &wa, int8_t &wb) {
  int8_t w = c.b - c.a + 1;
  if (w <= LINE_MAX_W) { wa = c.a; wb = c.b; return clCenter(c); }
  int8_t ic = (int8_t)lroundf(idxOf(pred));
  if (ic < c.a + 1) ic = c.a + 1;
  if (ic > c.b - 1) ic = c.b - 1;
  wa = ic - 1; wb = ic + 1;
  return POS[ic];
}

// returns index of the cluster to follow, -1 if none
int8_t chooseCluster(float pred) {
  if (nClusters == 0) return -1;
  // a fork decision is active: take the left-most / right-most nearby branch
  if (forkSide != 0 && (millis() - forkMs) < FORK_HOLD_MS && nClusters >= 2) {
    int8_t best = -1; float bestP = 0;
    for (uint8_t k = 0; k < nClusters; k++) {
      if (clDist(cl[k], pred) > 0.7f) continue;
      float c = clCenter(cl[k]);
      if (best < 0 || (forkSide > 0 ? c > bestP : c < bestP)) { best = k; bestP = c; }
    }
    if (best >= 0) return best;
  }
  int8_t best = 0; float bd = 99;
  for (uint8_t k = 0; k < nClusters; k++) {
    float d = clDist(cl[k], pred);
    if (d < bd) { bd = d; best = k; }
  }
  return best;
}

bool isMarkerPattern(int8_t t) {
  // [dash] gap [line] gap [dash]  : exactly 3 clusters, tracked one in the middle
  if (nClusters != 3 || t != 1) return false;
  for (uint8_t k = 0; k < 3; k++) if (cl[k].b - cl[k].a + 1 > LINE_MAX_W) return false;
  return true;
}

// ============================================================================
// 10. SPEED + STEERING
// ============================================================================
float speedTarget(uint32_t tRun) {
  // curve level from |error| and its rate
  float curv = absEF + P.curvD * dAbsF;
  float k = clampf((curv - P.curv0) / (P.curv1 - P.curv0 + 1e-3f), 0, 1);
  float tgt = P.vMax - (P.vMax - P.vCurve) * k;

  // straight: centre (+ rear sensor) on the line and small error for a while
  bool straight = absEF < 0.12f && fabsf(lastPos) < 0.2f && dAbsF < 4.0f;
#if USE_BACK_SENSOR
  straight = straight && backOn;
#endif
  if (!straight) straightSinceMs = millis();
  else if (millis() - straightSinceMs > STRAIGHT_CONFIRM_MS) tgt = P.vBoost;

  if (millis() - cornerMs < CORNER_MEM_MS) tgt = fminf(tgt, P.vCorner);
  if (tRun < (uint32_t)P.startMs) tgt = fminf(tgt, P.vStart);
  return fmaxf(tgt * spdScale * adaptK, MIN_BASE_PWM);
}

void driveFollow(float e, float dt) {
  float tgt = speedTarget(runMs());

  // base command: ramp up slowly, drop / brake instantly
  float bMax = P.brakeMax * spdScale;
  if (vEst > tgt + 15 * spdScale) {
    base = tgt - P.brakeGain * (vEst - tgt);
    if (base < -bMax) base = -bMax;
  } else if (tgt > base) {
    base += P.accel * dt * 1000.0f;
    if (base > tgt) base = tgt;
  } else base = tgt;

  // PD with speed-scheduled gains
  float s  = clampf(vEst, 0, 255) * (1.0f / 255.0f);
  float kp = P.kpLo + (P.kpHi - P.kpLo) * s;
  float kd = P.kdLo + (P.kdHi - P.kdLo) * s;
  // gentler near the centre (straights), full strength for big errors (curves)
  float soft = 0.55f + 0.45f * clampf(fabsf(e) / 0.35f, 0, 1);
  float corr = (kp * soft * e + kd * dFilt) * spdScale;   // gains were tuned for full speed -> scale with speed
  corr = clampf(corr, -400, 400);

  float l = base + corr, r = base - corr;
  // keep the steering difference when a wheel would saturate
  float hi = fmaxf(l, r);
  if (hi > 255) { l -= hi - 255; r -= hi - 255; }
  float lim = (base < 0) ? 255.0f : P.revLimit * spdScale;
  l = clampf(l, -lim, 255);
  r = clampf(r, -lim, 255);
  setMotors(l, r);
}

// ============================================================================
// 11. ONE CONTROL STEP
// ============================================================================
void enterMode(DriveMode m) { mode = m; modeMs = millis(); }

bool sawLineInPivot = false;
uint32_t foundMs = 0;  int8_t foundDir = 0;
bool gentlePivot = false;

void startPivot(int8_t dir) {
  // lost again right after a pivot, on the other side = we over-rotated:
  // turn back gently and on the spot instead of swinging round again
  gentlePivot = (millis() - foundMs < 200) && dir == -foundDir;
  pivotDir = dir;
  sawLineInPivot = false;
  enterMode(M_PIVOT);
  logEv(EV_PIVOT, dir);
}

void controlStep(float dt) {
  uint32_t now = millis();
  nClusters = findClusters(frontMask);

  // ---------------- choose the line ----------------
  float pred = lastPos;
  int8_t t = chooseCluster(pred);
  bool  sameLine = false;
  int8_t wa = 0, wb = 0;
  float pos = lastPos;
  if (t >= 0) {
    pos = linePosIn(cl[t], pred, wa, wb);
    sameLine = clDist(cl[t], pred) <= JUMP_MAX;
    trkA = cl[t].a; trkB = cl[t].b;
    extR = wa - cl[t].a;      // black beyond the line on the RIGHT
    extL = cl[t].b - wb;      // ... on the LEFT
  } else { trkA = trkB = -1; extL = extR = 0; }

  int8_t tw = (t >= 0) ? (int8_t)(cl[t].b - cl[t].a + 1) : 0;
  // crossing = the line we follow suddenly becomes wide on BOTH sides
  // (the start/finish mark has gaps next to the line, so it never counts)
  bool wideBoth = (t >= 0) && ((extL >= 2 && extR >= 2) || tw >= 9);
  // corner = black running from the line all the way to the edge of the bar on ONE
  // side (a perpendicular line). A merging branch arrives as a separate patch first,
  // so it does not trigger this.
  bool sideR    = (t >= 0) && extR >= 3 && cl[t].a <= 1 && extL <= 1;
  bool sideL    = (t >= 0) && extL >= 3 && cl[t].b >= (int8_t)N_FRONT - 2 && extR <= 1;
  if (wideBoth || tw >= 5) lastWideMs = now;

  // ---------------- corner signature ----------------
  if (mode == M_FOLLOW && (sideR || sideL)) {
    int8_t sd = sideR ? 1 : -1;
    if (now - cornerMs > CORNER_MEM_MS || cornerSide != sd) logEv(EV_CORNER, sd);
    cornerSide = sd; cornerMs = now;
  }

  // ---------------- crossing ----------------
  // a crossing shows up on both sides at once; if one side came first it is a
  // corner (or the acute corner's second leg) - keep the corner memory then
  bool cornerJustSeen = (now - cornerMs < 40) && cornerSide != 0;
  crossFrames = (wideBoth && !cornerJustSeen) ? crossFrames + 1 : 0;
  if (mode == M_FOLLOW && crossFrames >= 1) {
    enterMode(M_CROSS);
    crossEnterMs = now;
    cornerMs = 0;                      // a crossing is not a corner
    logEv(EV_CROSS, 0);
  }

  // ---------------- marks (start / finish) ----------------
  // pattern  [dash] gap [line] gap [dash]. Confirmed only when it has ENDED and
  // no wide patch (crossing / corner) was seen around it - a crossing seen by a
  // V-shaped bar can look like 3 patches for a moment, a real mark never widens.
  bool pat = isMarkerPattern(t) && mode == M_FOLLOW && (now - crossMs > MARKER_BLANK_MS);
  if (pat) {
    markerFrames++; markCandLastMs = now;
    if (markerFrames >= 2) markCand = true;
  } else {
    markerFrames = 0;
  }
  if (now - lastWideMs < 5 || mode != M_FOLLOW) { markCand = false; markerFrames = 0; }
  if (markCand && !pat && now - markCandLastMs >= 25) {
    markCand = false;
    if (now - lastWideMs > 150 && now - markerMs > MARKER_DEBOUNCE_MS) {
      markerMs = now; markerCount++;
      logEv(EV_MARK, (int8_t)markerCount);
      if (P.stopFinish > 0.5f && runMs() >= (uint32_t)P.finishArm) {
        logEv(EV_FINISH, (int8_t)markerCount);
        state = ST_FINISH; modeMs = now;
        return;
      }
    }
  }

  // ---------------- fork (line splits in front of us) ----------------
  // two patches, one on EACH side of where the line was, both close to it,
  // robot centred, no crossing / corner just before. A merging branch or the
  // second leg of a corner appears on one side only, so it is not a fork.
  bool forkNow = false;
  uint8_t plen = planB ? PLAN_B_LEN : PLAN_A_LEN;
  const uint8_t  *pl  = planB ? PLAN_B : PLAN_A;
  const uint16_t *pmn = planB ? PLAN_B_MIN_MS : PLAN_A_MIN_MS;
  bool forkDue = forkIndex < plen && runMs() >= (uint32_t)(pmn[forkIndex] / sqrtf(spdScale));
  if (forkDue && mode == M_FOLLOW && nClusters == 2 && forkSide == 0 &&
      now - cornerMs > 150 && now - crossMs > 150 && fabsf(stablePos) < 0.45f) {
    float cR = clCenter(cl[0]), cLft = clCenter(cl[1]);   // cl[0] is the right one
    float gapR = POS[cl[0].b], gapL = POS[cl[1].a];        // edges of the white gap
    forkNow = pred <= gapR + 0.05f && pred >= gapL - 0.05f &&          // we were in the gap
              (cR - pred) < 0.6f && (pred - cLft) < 0.6f &&             // both branches close
              fabsf((cR - pred) - (pred - cLft)) < 0.35f;               // roughly symmetric
  }
  forkFrames = forkNow ? forkFrames + 1 : 0;
  bool holdPred = forkNow && forkFrames < 2;   // keep "where the line was" until confirmed
  if (forkFrames == 2) {
    uint8_t act = pl[forkIndex];
    forkIndex++;
    forkSide = (act == GO_RIGHT) ? 1 : (act == GO_LEFT ? -1 : 0);
    forkMs = now;
    logEv(EV_FORK, forkSide);
    if (forkSide != 0) { t = chooseCluster(pred); if (t >= 0) { pos = linePosIn(cl[t], pred, wa, wb); sameLine = true; } }
  }
  if (forkSide != 0 && now - forkMs > FORK_HOLD_MS) forkSide = 0;

  // ================= MODES =================
  float e = lastPos;

  if (mode == M_CROSS) {
    // drive straight over the crossing on the heading we had before it
    e = stablePos;
    bool narrow = (t >= 0) && !wideBoth && (cl[t].b - cl[t].a + 1) <= LINE_MAX_W;
    exitFrames = narrow ? exitFrames + 1 : 0;
    if (exitFrames >= 3 || now - crossEnterMs > CROSS_MAX_MS) {
      crossMs = now;
      exitFrames = 0;
      enterMode(M_FOLLOW);
      if (t >= 0 && sameLine) { lastPos = pos; }
    }
    driveFollow(e, dt);
    ePrev = e; dFilt *= 0.9f;
    return;
  }

  if (mode == M_PIVOT || mode == M_SEARCH) {
    // spin toward the line; leave when it is back near the centre
    float fwd = (mode == M_PIVOT ? P.pivotFwd : P.pivotFwd * 0.7f) * spdScale;
    float rev = (mode == M_PIVOT ? P.pivotRev : P.pivotRev * 0.7f) * spdScale;
    if (gentlePivot) { fwd *= 0.6f; rev = fwd; }
    // still fast? turn the outer wheel into a brake as well
    float over = vEst - P.vCorner * spdScale;
    if (over > 0) fwd = fmaxf(fwd - 1.5f * over, -P.brakeMax * spdScale);
    if (pivotDir > 0) setMotors(fwd, -rev); else setMotors(-rev, fwd);
    base = 0;
    if (t >= 0) {
      lostSinceMs = now;
      // two sightings in a row give the speed at which the line sweeps in;
      // hand over to the PD right away so its D-term can stop the rotation
      // ...but only once the line is really sweeping in toward the centre
      // (a branch that is simply sitting at the side must not end the pivot)
      bool sweepingIn = fabsf(pos) < fabsf(ePrev) - 0.01f || fabsf(pos) < 0.3f;
      if (sawLineInPivot && fabsf(pos) < PIVOT_EXIT_POS && sweepingIn && now - modeMs >= 6) {
        dFilt = (pos - ePrev) / dt;
        ePrev = pos;
        lastPos = pos; stablePos = pos;
        absEF = fabsf(pos);
        base = P.vCorner * spdScale * 0.5f;
        logEv(EV_FOUND, (int8_t)(pos * 100));
        foundMs = now; foundDir = pivotDir;
        // a long search means we arrived too fast: slow everything down a bit
        if (P.adapt > 0.5f && (mode == M_SEARCH || now - modeMs > 250)) adaptK = fmaxf(0.75f, adaptK - 0.08f);
        enterMode(M_FOLLOW);
        cornerMs = 0;
        return;
      }
      ePrev = pos; sawLineInPivot = true;
    } else sawLineInPivot = false;
    if (now - modeMs > PIVOT_MAX_MS && mode == M_PIVOT) { enterMode(M_SEARCH); }
    if (now - lostSinceMs > (uint32_t)P.lostStopMs) {
      logEv(EV_LINELOST_STOP, 0);
      state = ST_FINISH; modeMs = now;
    }
    return;
  }

  // ---------------- M_FOLLOW ----------------
  if (holdPred) {
    e = lastPos;                 // possible fork: do not jump to either branch yet
  } else if (t < 0) {
    // nothing under the bar
    lostFrames++;
    if (lostFrames >= 2) {
      int8_t dir;
      if (now - cornerMs < CORNER_MEM_MS && cornerSide != 0) dir = cornerSide;
      else dir = (lastPos >= 0) ? 1 : -1;
      lostSinceMs = now;
      startPivot(dir);
      return;
    }
    e = lastPos;   // keep last steering for one sample
  } else {
    lostFrames = 0;
    lostSinceMs = now;
    if (!sameLine) {
      // the line we followed vanished and another one is visible
      jumpFrames++;
      bool cornerSideOk = (now - cornerMs < CORNER_MEM_MS) && signf(pos) == cornerSide;
      if (cornerSideOk || jumpFrames >= 3) {
        logEv(EV_JUMP, (int8_t)(pos * 100));
        jumpFrames = 0;
        lastPos = pos; ePrev = pos; dFilt = 0;
        if (fabsf(pos) > 0.6f) { startPivot(pos > 0 ? 1 : -1); return; }
      }
      e = lastPos;
    } else {
      jumpFrames = 0;
      lastPos = pos;
      e = pos;
      bool cornerRecent = (now - cornerMs < CORNER_MEM_MS) && cornerSide != 0;
      // after a corner signature: the straight part is gone (no black at the centre
      // any more, only the branch on the corner side) or the line is at the edge
      bool centreGone = cornerSide > 0 ? (POS[cl[t].b] > 0.2f) : (POS[cl[t].a] < -0.2f);
      if (cornerRecent && (centreGone || (fabsf(pos) > 0.8f && signf(pos) == cornerSide))) {
        startPivot(cornerSide);
        return;
      }
    }
  }
  stablePos += (e - stablePos) * clampf(dt / 0.012f, 0, 1);

  // derivative (filtered) and curve indicators
  float de = (e - ePrev) / dt;
  ePrev = e;
  dFilt += (de - dFilt) * clampf(dt / 0.012f, 0, 1);   // smoother D (was 4 ms): fewer jolts on straights
  absEF += (fabsf(e) - absEF) * clampf(dt / 0.008f, 0, 1);
  dAbsF += (fabsf(dFilt) - dAbsF) * clampf(dt / 0.015f, 0, 1);

  driveFollow(e, dt);
}

// ============================================================================
// 12. RUN CONTROL
// ============================================================================
void resetRun() {
  uint32_t longAgo = millis() - 60000UL;   // "never happened" for every event timer
  mode = M_FOLLOW;
  lastPos = stablePos = 0;
  ePrev = dFilt = absEF = dAbsF = 0;
  base = 0; vEst = 0; adaptK = 1.0f;
  cornerMs = crossMs = markerMs = forkMs = foundMs = longAgo;
  markCandLastMs = lastWideMs = longAgo;
  foundDir = 0; gentlePivot = false;
  cornerSide = forkSide = 0;
  crossFrames = exitFrames = lostFrames = jumpFrames = markerFrames = 0;
  forkFrames = 0;
  markCand = false;
  forkIndex = 0; markerCount = 0;
  evN = 0;
}

void startRun() {
  testMode = dipOn(PIN_DIP1);
  safeMode = dipOn(PIN_DIP2);
  planB    = dipOn(PIN_DIP3);
  spdScale = 1.0f;
#if USE_TRIMPOT
  spdScale = 0.6f + 0.4f * (analogRead(PIN_TRIM) / 1023.0f);
#endif
  if (safeMode) spdScale *= 0.6f;
  spdScale *= clampf(P.speedPct, 10, 130) / 100.0f;
  learnLinePolarity();
  readSensors();
  // initial line position from what we see now
  nClusters = findClusters(frontMask);
  if (nClusters) {
    int8_t t = chooseCluster(0); int8_t a, b;
    lastPos = stablePos = linePosIn(cl[t], 0, a, b);
  }
  if (testMode) return;            // DIP1: never drive
  if (PIN_FAN >= 0) { fanOn(true); delay(FAN_SPINUP_MS); }
  if (START_DELAY_MS) delay(START_DELAY_MS);
  resetRun();
  nClusters = findClusters(frontMask);
  if (nClusters) {
    int8_t t = chooseCluster(0); int8_t a, b;
    lastPos = stablePos = ePrev = linePosIn(cl[t], 0, a, b);
  }
  runStartMs = millis();
  modeMs = runStartMs; lostSinceMs = runStartMs; straightSinceMs = runStartMs;
  logEv(EV_START, (int8_t)(spdScale * 100));
  state = ST_RUN;
  lastLoopUs = micros();
}

void finishRun(uint8_t why) {
  // short reverse pulse proportional to speed, then stop
  if (vEst > 60) {
    float b = -clampf(vEst, 0, P.brakeMax);
    setMotors(b, b);
    delay((uint16_t)clampf(vEst * 0.35f, 10, 90));
  }
  stopMotors();
  fanOn(false);
  if (why) logEv(why, 0);
  lastRunMs = (uint16_t)runMs();
  state = ST_IDLE;
  startArmed = false;
  autoNeedLift = true;
  autoSinceMs = millis();
}

// Start / stop on D10. Works with both kinds of start hardware:
//  * MicroStart module (rests LOW): START (HIGH) = go, STOP (LOW) = stop.
//  * board button: click = go, click again = stop.
// The rest level is measured at power-on, so do not touch the button then.
void serviceStartPin() {
  if (PIN_START < 0) return;
  uint8_t lvl = digitalRead((uint8_t)PIN_START);
  if (lvl != startStable) { if (++startCnt < 5) return; }   // 5 samples debounce
  else { startCnt = 0; return; }
  startCnt = 0;
  startStable = lvl;
  uint32_t now = millis();
  bool away = (lvl != startRest);           // pressed / module started
  if (startRest == LOW) {                    // module style, or button pulling HIGH
    if (away) {
      if (state == ST_IDLE) startRun(); else finishRun(EV_STOPPED);   // click while running = stop
      startEdgeMs = now;
    } else if (state == ST_RUN && now - startEdgeMs > 1500) {
      finishRun(EV_STOPPED);                 // module STOP (signal was held, not a click)
    }
  } else {                                   // pull-up button: press arms, release goes
    if (away) {
      if (state == ST_RUN) finishRun(EV_STOPPED); else startArmed = true;
    } else if (startArmed && state == ST_IDLE) {
      startArmed = false; startRun();
    }
  }
}

// ============================================================================
// 13. USB CONSOLE
// ============================================================================
PDef PTAB[] = {
  {"boost",&P.vBoost},{"vmax",&P.vMax},{"vcurve",&P.vCurve},{"vcorner",&P.vCorner},
  {"vstart",&P.vStart},{"startms",&P.startMs},{"kplo",&P.kpLo},{"kphi",&P.kpHi},
  {"kdlo",&P.kdLo},{"kdhi",&P.kdHi},{"accel",&P.accel},{"bgain",&P.brakeGain},
  {"bmax",&P.brakeMax},{"pfwd",&P.pivotFwd},{"prev",&P.pivotRev},{"rlim",&P.revLimit},
  {"tau",&P.tauMs},{"c0",&P.curv0},{"c1",&P.curv1},{"cd",&P.curvD},
  {"stop",&P.stopFinish},{"arm",&P.finishArm},{"lost",&P.lostStopMs},{"mux",&P.muxUs},
  {"adapt",&P.adapt},{"speed",&P.speedPct}
};
const uint8_t PTAB_N = sizeof(PTAB) / sizeof(PTAB[0]);

void saveParams() { EEPROM.put(0, PARAM_MAGIC); EEPROM.put(2, P); }
bool loadParams() {
  uint16_t m = 0; EEPROM.get(0, m);
  if (m != PARAM_MAGIC) return false;
  EEPROM.get(2, P);
  return true;
}

void printParams() {
  for (uint8_t i = 0; i < PTAB_N; i++) {
    Serial.print(PTAB[i].name); Serial.print(F(" = ")); Serial.println(*PTAB[i].v, 3);
  }
}

const char *evName(uint8_t t) {
  switch (t) {
    case EV_START: return "START(scale%)"; case EV_MARK: return "MARK#";
    case EV_CROSS: return "CROSSING"; case EV_FORK: return "FORK(side)";
    case EV_CORNER: return "CORNER(side)"; case EV_PIVOT: return "PIVOT(dir)";
    case EV_LOST: return "LINE LOST(dir)"; case EV_FOUND: return "FOUND(pos%)";
    case EV_FINISH: return "FINISH"; case EV_STOPPED: return "STOP SIGNAL";
    case EV_LINELOST_STOP: return "STOP: LINE LOST"; case EV_JUMP: return "JUMP(pos%)";
  }
  return "?";
}

void printLog() {
  Serial.print(F("Last run: ")); Serial.print(lastRunMs); Serial.println(F(" ms"));
  uint16_t first = evN > EV_MAX ? evN - EV_MAX : 0;
  if (first) { Serial.print(first); Serial.println(F(" older events dropped")); }
  for (uint16_t i = first; i < evN; i++) {
    const Ev &e = evLog[i % EV_MAX];
    Serial.print(e.t); Serial.print(F(" ms  "));
    Serial.print(evName(e.type)); Serial.print(' ');
    Serial.println(e.val);
  }
}

void printSensors() {
  // left of the robot on the left of the screen
  Serial.print(F("L |"));
  for (int8_t i = N_FRONT - 1; i >= 0; i--) Serial.print((frontMask >> i) & 1 ? '#' : '.');
  Serial.print(F("| R  back:")); Serial.print(backOn ? '#' : '.');
  nClusters = findClusters(frontMask);
  Serial.print(F("  clusters:")); Serial.print(nClusters);
  if (nClusters) {
    int8_t t = chooseCluster(lastPos); int8_t a, b;
    float p = linePosIn(cl[t], lastPos, a, b); lastPos = p;
    Serial.print(F("  pos:")); Serial.print(p, 2);
  }
  Serial.print(F("  black=")); Serial.print(lineLevel == LOW ? F("LOW") : F("HIGH"));
  Serial.print(F("  trim%:")); Serial.println((int)(analogRead(PIN_TRIM) / 10.23f));
}

void help() {
  Serial.println(F("XPulse Robotex console. Commands:"));
  Serial.println(F("  p              print parameters"));
  Serial.println(F("  set <name> <v> change a parameter (e.g. set vmax 200)"));
  Serial.println(F("  save | load | defaults"));
  Serial.println(F("  s              live sensor view on/off"));
  Serial.println(F("  log            events of the last run"));
  Serial.println(F("  m              motor test (both wheels forward 0.4 s, robot in the air!)"));
}

void handleCommand(char *line) {
  char *cmd = strtok(line, " \t");
  if (!cmd) return;
  if (!strcmp(cmd, "?") || !strcmp(cmd, "help")) help();
  else if (!strcmp(cmd, "p")) printParams();
  else if (!strcmp(cmd, "save")) { saveParams(); Serial.println(F("saved")); }
  else if (!strcmp(cmd, "load")) { Serial.println(loadParams() ? F("loaded") : F("nothing saved")); }
  else if (!strcmp(cmd, "defaults")) { P = DEFAULTS; Serial.println(F("defaults (not saved)")); }
  else if (!strcmp(cmd, "s")) { liveView = !liveView; learnLinePolarity(); }
  else if (!strcmp(cmd, "log")) printLog();
  else if (!strcmp(cmd, "m")) {
    Serial.println(F("motor test"));
    setMotors(120, 120); delay(400); setMotors(-120, -120); delay(200); stopMotors();
  }
  else if (!strcmp(cmd, "set")) {
    char *name = strtok(NULL, " \t"); char *val = strtok(NULL, " \t");
    if (!name || !val) { Serial.println(F("usage: set <name> <value>")); return; }
    for (uint8_t i = 0; i < PTAB_N; i++) {
      if (!strcmp(name, PTAB[i].name)) {
        *PTAB[i].v = (float)atof(val);
        Serial.print(name); Serial.print(F(" = ")); Serial.println(*PTAB[i].v, 3);
        return;
      }
    }
    Serial.println(F("unknown name (type p)"));
  }
  else Serial.println(F("? for help"));
}

void serviceConsole() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { rxBuf[rxLen] = 0; handleCommand(rxBuf); rxLen = 0; }
    else if (rxLen < sizeof(rxBuf) - 1) rxBuf[rxLen++] = c;
  }
}

// ============================================================================
// 14. SETUP / LOOP
// ============================================================================
void setup() {
  pinMode(PIN_MUX_S0, OUTPUT); pinMode(PIN_MUX_S1, OUTPUT);
  pinMode(PIN_MUX_S2, OUTPUT); pinMode(PIN_MUX_S3, OUTPUT);
  digitalWrite(PIN_MUX_S0, LOW); digitalWrite(PIN_MUX_S1, LOW);
  digitalWrite(PIN_MUX_S2, LOW); digitalWrite(PIN_MUX_S3, LOW);
  muxCh = 0;
  pinMode(PIN_MUX_SIG, INPUT);
  pinMode(PIN_L_PWM, OUTPUT); pinMode(PIN_L_DIR, OUTPUT);
  pinMode(PIN_R_PWM, OUTPUT); pinMode(PIN_R_DIR, OUTPUT);
  if (PIN_START >= 0) pinMode((uint8_t)PIN_START, INPUT_PULLUP);
  delay(50);
  if (PIN_START >= 0) startRest = startStable = digitalRead((uint8_t)PIN_START);
  pinMode(PIN_DIP1, INPUT_PULLUP); pinMode(PIN_DIP2, INPUT_PULLUP); pinMode(PIN_DIP3, INPUT_PULLUP);
  pinMode(PIN_LED_L, OUTPUT); pinMode(PIN_LED_R, OUTPUT);
  if (PIN_FAN >= 0) pinMode((uint8_t)PIN_FAN, OUTPUT);
  stopMotors();
  fanOn(false);

#if MATCH_PWM_FREQ && defined(__AVR_ATmega32U4__)
  // Timer1 (D9/D10) from 490 Hz phase-correct to ~980 Hz fast PWM, same as Timer0 (D11)
  TCCR1B |= _BV(WGM12);
#endif

  for (uint8_t i = 0; i < N_FRONT; i++) POS[i] = (7.0f - (float)i) / 7.0f;  // 0 = right (+1)

  P = DEFAULTS;
  loadParams();
  // "I am alive": both LEDs blink 3 times after upload / power-on
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(PIN_LED_L, HIGH); digitalWrite(PIN_LED_R, HIGH); delay(120);
    digitalWrite(PIN_LED_L, LOW);  digitalWrite(PIN_LED_R, LOW);  delay(120);
  }
  Serial.begin(115200);   // never wait for the PC: the robot must run without USB
  learnLinePolarity();
}

void loop() {
  uint32_t nowUs = micros();

  if (state == ST_RUN) {
    if ((uint32_t)(nowUs - lastLoopUs) < LOOP_US) return;
    float dt = (nowUs - lastLoopUs) * 1e-6f;
    if (dt > 0.01f) dt = 0.01f;
    lastLoopUs = nowUs;

    readSensors();
    serviceStartPin();
    if (state != ST_RUN) return;
    controlStep(dt);
    if (adaptK < 1.0f) adaptK = fminf(1.0f, adaptK + 0.02f * dt);
    // speed model (PWM units)
    vEst += (0.5f * (cmdL + cmdR) - vEst) * clampf(dt * 1000.0f / fmaxf(P.tauMs, 1.0f), 0, 1);
    if (state == ST_FINISH) finishRun(0);
    // LEDs: which side the line is on
    digitalWrite(PIN_LED_L, lastPos < -0.15f ? HIGH : LOW);
    digitalWrite(PIN_LED_R, lastPos >  0.15f ? HIGH : LOW);
    return;
  }

  // ---------------- not running ----------------
  static uint32_t lastIdleUs = 0;
  if ((uint32_t)(nowUs - lastIdleUs) < 1000) return;
  lastIdleUs = nowUs;
  stopMotors();
  serviceStartPin();
  if (state == ST_RUN) return;
  serviceConsole();

  uint32_t ms = millis();
  bool test = dipOn(PIN_DIP1);

#if AUTO_START
  // ---- auto start: one clean line under the middle of the bar for 3 s -> go ----
  {
    static uint32_t lastPolMs = 0;
    if (ms - lastPolMs > 500) { lastPolMs = ms; learnLinePolarity(); }
    readSensors();
    uint8_t n = findClusters(frontMask);
    bool onLine = false;
    if (n == 1) {
      float c = clCenter(cl[0]); int8_t w = cl[0].b - cl[0].a + 1;
      onLine = w <= LINE_MAX_W && fabsf(c) < 0.45f;
    }
    if (frontMask == 0) autoNeedLift = false;          // lifted / off the line
    if (!onLine || autoNeedLift || test) autoSinceMs = ms;
    else if (ms - autoSinceMs > AUTO_START_MS) {
      if (Serial) Serial.println(F("AUTO START"));
      startRun();
      if (state == ST_RUN) return;
    }
    // countdown: both LEDs blink faster and faster
    if (onLine && !autoNeedLift && ms - autoSinceMs > 300) {
      uint32_t left = AUTO_START_MS - (ms - autoSinceMs);
      bool ph = (ms / (left > 1500 ? 250 : 80)) & 1;
      digitalWrite(PIN_LED_L, ph); digitalWrite(PIN_LED_R, ph);
      return;
    }
  }
#endif
  if (test || liveView) {
    if (ms - lastViewMs > 120) learnLinePolarity();
    readSensors();
    digitalWrite(PIN_LED_R, (frontMask & 0x007F) ? HIGH : LOW);
    digitalWrite(PIN_LED_L, (frontMask & 0x7F00) ? HIGH : LOW);
    if (ms - lastViewMs > 120 && Serial) { lastViewMs = ms; printSensors(); }
  } else {
    // waiting: slow alternating blink, fast if a run log is waiting
    // waiting: right LED on (like your sample code); both on = armed, release to go
    digitalWrite(PIN_LED_R, HIGH);
    digitalWrite(PIN_LED_L, startArmed ? HIGH : LOW);
    static uint32_t lastStatusMs = 0;
    if (Serial && ms - lastStatusMs > 1500) {
      lastStatusMs = ms;
      readSensors();
      Serial.print(F("WAITING  "));
      Serial.print(F("auto-start: leave robot on the line 3 s"));
      Serial.print(F(" rest=")); Serial.print(startRest == HIGH ? F("HIGH") : F("LOW"));
      Serial.print(startArmed ? F(" (armed: release to go)") : F(" (click button / remote START)"));
      Serial.print(F("  A0(battery raw)=")); Serial.print(analogRead(A0));
      Serial.print(F("  speed%=")); Serial.print((int)P.speedPct);
      Serial.print(F("  sensors L|"));
      for (int8_t i = N_FRONT - 1; i >= 0; i--) Serial.print((frontMask >> i) & 1 ? '#' : '.');
      Serial.println(F("|R"));
    }
  }
}

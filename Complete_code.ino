// ============================================================
//  FLOOD FILL NAVIGATION (selectable, default = right-wall): reverse-BFS flood fill ported
//  from the simulator, using the existing maze map; one cell per decision; existing turn,
//  drive, recovery and mapping code do the physical work.
//  MAZE MAP (passive observer, added on top of the version below - movement code unchanged):
//  tracks the robot's cell (row 0-7, col 0-7) and direction, records the walls seen by the
//  existing senseWalls() calls, counts real cell-to-cell crossings, and shows an 8x8 map on
//  the dashboard. The map is never used for any driving decision.
//  Micromouse - RIGHT-WALL FOLLOWER - v13 + "Drive N cells" test
//  CELL TEST (only addition): web input N (1-7) + "Drive N cells" button. Drives one
//  straight of N x CELL_SIZE_MM from rest with the normal straight-drive code, ignoring
//  side openings, with wall-edge position snapping and distance self-calibration OFF for
//  that test only, then stops and stays idle and reports encoder-estimated distances.
//  Micromouse - RIGHT-WALL FOLLOWER - v13
//  v13:
//  - YAW IS TRACKED ALL THE TIME (also during pauses / waits). Before, rotation that
//    happened after a turn had stopped (sliding on the slippery floor) was never
//    counted - that is why the same turn came out over one run and under the next.
//  - EVERY TURN IS CHECKED AND FIXED: after a turn stops, the robot waits, measures
//    how far it really ended past (overshoot) or short of (undershoot) the target,
//    and makes small correction turns (up to 3) until it is within 1 deg.
//  - GYRO SCALE LEARNS ITSELF from the walls: the first wall-angle fit after a turn
//    measures the turn's REAL error (the gyro can't see its own scale error). The
//    scale is nudged after every such turn, so turns get more accurate as it runs.
//  - Gyro bias is only re-measured if the robot was truly still (no slow slide).
//  - Removed the "knocked crooked > 20 deg = blocked" check, so you can push the
//    robot to test the PID. (Both-wheels-stalled and slipping checks are still on.)
//  - STUCK SENSOR RESTART tries several times: stop, soft reset, init, and checks a
//    real reading arrives; if all fail it frees the I2C bus (9 clock pulses) and
//    tries again. Sensors that fail at power-up are also retried.
//  Micromouse - v12
//  v12:
//  - FRONT STOP: a wall only counts after 2 NEW "close" readings in a row (one bad
//    reading used to stop the robot early -> early turn). Front centre can be set
//    live from the page ("Set FRONT centre = now").
//  - TURNS: new gyro controller - speed profile (accelerate, cruise, slow down so it
//    ARRIVES slowly) with a PID on the rotation speed. Brakes if it's too fast near
//    the end, corrects back slowly if it still overshoots. GYRO_SCALE calibration +
//    "Turn test" button (4 x 90 deg) with overshoot report.
//  - WALL ALIGNMENT: while driving next to straight walls, the side sensors measure
//    the robot's real angle to the wall and correct the gyro heading, so small turn
//    errors are removed on the next straight.
//  - STUCK RECOVERY: better "blocked" detection (both wheels stopped, wheels slipping
//    while the front distance doesn't change, or knocked crooked). Recovery backs up
//    further each time, straightens, shifts away from a close wall, and after the 2nd
//    block in the same place stops trying to go straight (treats it as a wall).
//  Micromouse - v11
//  v11: I2C made more robust for long jumper wires - 100 kHz instead of 400 kHz,
//  every multiplexer switch is checked (a failed switch skips that read instead
//  of reading the wrong sensor), and the page shows I2C errors + sensor restarts.
//  Micromouse - v10
//  v10 fixes (from the 7-real / 12-counted cells run log):
//  - FRONT SENSOR WATCHDOG: the front read 361 mm for the whole run (frozen), so
//    the robot drove into walls. A sensor with no new reading for 250 ms is now
//    marked STUCK, the robot stops if it's the front one, and the sensor is
//    restarted automatically. The page shows STUCK instead of an old number.
//  - LEFT ENCODER over-counted ~2.4x (1003 counts while you placed the robot,
//    right wheel 20). Now: glitch filter in the encoder interrupts, and if one
//    wheel counts far more than the other the robot uses the smaller (sane) one
//    for distance and skips encoder sync in turns. A warning shows on the page.
//  - BLOCKED DETECTION: if the wheels stop turning while driving (robot pushing a
//    wall), it stops within 0.6 s, backs up and re-checks instead of pushing.
//  - Slower cruise speed: 88/81 -> 78/72
//  Micromouse - v9 (combined: v6 + ToF offsets + best of v8)
//  v9 changes (on top of v6 + ToF offset calibration):
//  - Side-centre calibration at GO uses only FRESH readings and a trimmed mean
//    (idea from v8), but keeps ONE shared centre = (LEFT + RIGHT) / 2, so the
//    robot centres on the REAL middle no matter where you place it at GO
//  - Side sensors pass through a median-of-3 filter (removes single spikes)
//  - Earlier stop leads (the soft stop rolls ~35 mm): front 15->40, cell 14->30
//  - The page shows WHY the ESP32 last restarted (e.g. BROWNOUT = battery)
//  v6 changes:
//  - Removed encoder "traction control" from straights: it was cancelling the
//    heading PID's steering (it slowed whichever wheel the PID sped up)
//  - Steering is no longer clipped at low speed (both wheels shift up together)
//  - Integral term ON and remembered between straights -> learns the motor mismatch
//  - Gyro bias is re-measured during every 1 s pre-turn stop (no slow drift)
//  - LIVE PID TUNING on the web page + "Straight test" button with wobble stats
//  Configured for a 16x16 maze with 19 x 19 cm cells, 4 cm wheels
//  - Smooth acceleration ramp, gentle stops (coast then light brake)
//  - Slower, ramped turns without hard braking; turn braking by coasting
//  - (v5 traction control removed in v6 - it fought the heading PID)
//  - Distance self-calibration only when the wheels can't be slipping
//  - Distance calibrated from your run log: ~197 encoder counts per wheel turn
//  - Front-wall squaring no longer stalls; extra clearance before turns
//  - Turn-stall recovery: if a turn gets blocked, back up a little and retry
//  - Drives through corridors continuously, checking the right side of every
//    cell WHILE MOVING; stops at the centre of the first cell with a right opening
//  - Slower cruise speed; distance self-calibrates from the front sensor
//  - Wall-edge correction: side sensors are AHEAD of the axle, so the moment a
//    side wall starts/ends fixes the robot's position
//  - Stop + 1 s pause + re-check + front-wall squaring before every turn
//  - Turns drive BOTH wheels in opposite directions, kept in sync by encoders
//  ESP32 + DRV8833 + N20 encoders + MPU6050 + 3x VL53L0X (via TCA9548A)
// ============================================================
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <VL53L0X.h>      // "VL53L0X" library by Pololu
#include <WiFi.h>
#include <WebServer.h>
#include <stdarg.h>
#include <esp_system.h>   // restart reason (brownout / crash)

// Types used in function parameters are declared FIRST, so the prototypes the
// Arduino IDE generates automatically can always see them.
enum Action { GO_FORWARD, TURN_RIGHT, TURN_LEFT, TURN_AROUND };
enum StopReason { STOP_FRONT_WALL, STOP_RIGHT_OPEN, STOP_UNSURE, STOP_TIMEOUT, STOP_ABORT,
                  STOP_BLOCKED, STOP_FRONT_STUCK,
                  STOP_CELL_TARGET,     // CELL TEST: distance target of the "Drive N cells" test reached
                  STOP_FLOOD_CELL };    // FLOOD FILL: one-cell drive reached the next cell centre
// MAZE MAP: absolute maze directions (row 0 = north/top, col 0 = west/left)
enum Direction { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };
// FLOOD FILL: which navigation layer chooses the moves
enum NavigationMode { NAV_RIGHT_WALL = 0, NAV_FLOOD_FILL = 1 };

// Median-of-3 filter for the side sensors: one bad reading can't get through
struct Median3 {
  uint16_t v[3] = {0, 0, 0};
  uint8_t  n = 0, i = 0;
  uint16_t push(uint16_t x) {
    v[i] = x; i = (i + 1) % 3; if (n < 3) n++;
    if (n < 3) return x;                       // not enough history yet
    uint16_t a = v[0], b = v[1], c = v[2];
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) { uint16_t t = b; b = c; c = t; }
    if (a > b) { uint16_t t = a; a = b; b = t; }
    return b;                                  // the middle value
  }
};

// ---------- Pins (fixed hardware assignments) ----------
const int IN1 = 32;   // Left motor
const int IN2 = 33;
const int IN3 = 25;   // Right motor
const int IN4 = 26;

const int LEFT_ENCODER_C1  = 34;
const int LEFT_ENCODER_C2  = 35;
const int RIGHT_ENCODER_C1 = 13;
const int RIGHT_ENCODER_C2 = 27;

const int SDA_PIN = 19;
const int SCL_PIN = 18;

const uint8_t TCA_ADDRESS = 0x70;
const uint8_t MPU_CHANNEL = 4;

const uint8_t FRONT_TOF_CHANNEL = 5;
const uint8_t LEFT_TOF_CHANNEL  = 6;
const uint8_t RIGHT_TOF_CHANNEL = 7;

// ToF offset calibration: (what the sensor reads) - (real distance), in mm
const int FRONT_TOF_OFFSET_MM = 30;   // NEW front sensor: page showed 58-63 (with +15) at a real 45 -> raw ~76, 76-45 = ~30
const int LEFT_TOF_OFFSET_MM  = -3;   // NEW left sensor: raw ~52 at a real 55 (page showed 15-20 with the old +35)
const int RIGHT_TOF_OFFSET_MM = 2;    // reads ~57 at a real 55

// ============================================================
//  MAZE / ROBOT CALIBRATION
// ============================================================
const int   CELL_SIZE_MM      = 190;   // 19 x 19 cm cells
const float WHEEL_DIAMETER_MM = 40.0;  // 4 cm wheels

// Encoder counts for ONE full wheel turn.
// From your run log: front 542 -> 53 mm (489 mm) while the encoders counted ~766
// => 1.57 counts/mm => ~197 counts per wheel turn (40 mm wheel).
long COUNTS_PER_WHEEL_REV = 197;

// Starting value - the robot also self-calibrates this from the front sensor
// the first time it drives toward a wall (see "Distance self-cal" in Serial / page)
float countsPerMM = COUNTS_PER_WHEEL_REV / (PI * WHEEL_DIAMETER_MM);
bool  distCalibrated = false;

// How far IN FRONT of the wheel axle the side sensors are (mm) - MEASURE
// (ruler from the centre of the wheel to the centre of the side sensor, along the robot)
const int SIDE_SENSOR_AHEAD_MM = 45;

// Extra correction for sensor lag when a wall edge is detected.
// Robot stops too FAR past the centre after an opening -> increase. Too SHORT -> decrease.
const int EDGE_LAG_MM = 10;

// Readings with the robot centred in a cell.
// SIDE is auto-measured when you press GO if there are walls on both sides.
int        sideCenterMM    = 87;    // 72 mm in 16 cm cells + 15 mm per side in 19 cm cells; auto-measured at GO
// Front reading when the WHEEL AXLE is over the cell centre, facing a wall.
// MEASURE: place the robot by hand like that and read FRONT on the page.
int  frontCenterMM = 65;            // v12: live-settable from the page ("Set FRONT centre = now")
// Before a turn with a wall ahead, stand this much further back from it so the
// front corners don't hit the wall while spinning. Robot still scrapes -> increase.
const int  FRONT_TURN_CLEARANCE_MM = 15;   // (not used any more - see frontStopMM)
// FRONT STOP DISTANCE: front reading (mm) the robot stops at before turning when there's a
// wall ahead. Live-settable from the page ("Front stop" box). Bigger = stops further from
// the wall; smaller = closer (corners may scrape the wall while turning if too small).
int frontStopMM = 50;                       // = old 65 centre + 15 clearance
const int  BACKUP_MM = 20;          // how far to back up if a turn gets blocked

// Wall-detection thresholds (reading below this = wall). Updated after auto-calibration.
int sideWallMM            = sideCenterMM + CELL_SIZE_MM / 2;
int frontWallMM          = 65 + CELL_SIZE_MM / 2;   // updated whenever frontCenterMM changes

// ---------- Maze driving behaviour ----------
float WALL_CENTER_GAIN         = 0.15;  // deg of heading per mm off-centre (live-tunable)
const float MAX_WALL_STEER_DEG = 10.0;
const int   FRONT_STOP_LEAD_MM = 25;    // start stopping this far BEFORE frontStopMM (it rolls a bit)
const int   FRONT_ALIGN_PWM    = 100;   // 80 was too weak to start the motors
const int   FRONT_ALIGN_TOL_MM = 6;
const unsigned long DRIVE_TIMEOUT_MS   = 30000; // longest straight (15 cells = 2.85 m) at slow speed
const int   DECIDE_BEFORE_MM   = 35;    // decide about a cell's right side this far before its centre
const int   STOP_LEAD_MM       = 30;    // was 14 - same roll at right-opening stops
const float APPROACH_MIN_SCALE = 0.70;  // slowest speed when approaching a stop (x cruise)
const unsigned long ARRIVE_SETTLE_MS   = 250;   // after stopping in a cell, before looking
const unsigned long PRE_TURN_PAUSE_MS  = 1000;  // full stop before every turn
const unsigned long POST_TURN_PAUSE_MS = 300;
const bool  AUTO_START = false;

// ---------- Wi-Fi ----------
const char* WIFI_NAME     = "Micromouse";
const char* WIFI_PASSWORD = "robot1234";
const unsigned long TOF_POLL_PERIOD_MS = 10;   // sensors give a new reading every ~33 ms
const uint16_t TOF_MAX_VALID_MM = 2000;

// ---------- Base speeds (your trim values) ----------
const int FORWARD_LEFT_SPEED   = 120;   // your trim values (kept for the ratio)
const int FORWARD_RIGHT_SPEED  = 110;

// Maze cruising speed - SLOWER than before so the sensors get more readings per cell.
// Too slow and it stalls -> raise both a little (keep the same ratio).
const int CRUISE_LEFT_PWM  = 78;   // v10: slower (was 88) - more readings per cell
const int CRUISE_RIGHT_PWM = 72;   // keeps the same left/right ratio (was 81)

// ---------- ANTI-SLIP (tune these for the slippery maze) ----------
const unsigned long ACCEL_RAMP_MS = 450;  // time to ramp from START_SCALE to full cruise speed
const float START_SCALE           = 0.70; // starting power (x cruise) - just enough to move
const unsigned long COAST_MS      = 90;   // stop: coast first (no skid)...
const unsigned long SOFT_BRAKE_MS = 60;   // ...then a short brake to finish
const unsigned long TURN_RAMP_MS  = 250;  // turns ramp up their power too
const int BACKWARD_LEFT_SPEED  = 110;
const int BACKWARD_RIGHT_SPEED = 100;

// ---------- Heading PID (straight driving) - all live-tunable from the web page ----------
// Low-friction floor: the robot yaws more easily, so it needs a bit more damping (Kd)
// and an integral term to cancel the steady left/right motor mismatch.
float Kp = 3.0;    // PWM per degree of heading error
float Ki = 1.5;    // PWM per degree*second (learns the motor mismatch)
float Kd = 0.6;    // PWM per deg/s of rotation (damping - stops wobbling)
const float I_TERM_MAX = 25.0;   // integral can add/remove at most this much PWM
float headingIntegral = 0;       // kept between straights (reset at GO)

// Live values for the web page
volatile float liveHeading = 0;
volatile float statMaxHdg = 0, statAvgHdg = 0;
volatile int   statCrossings = 0;
volatile bool  straightTestReq = false;

// ---------- Straight-drive limits ----------
const float MAX_CORRECTION = 60.0;
const int   MIN_PWM        = 60;
const int   MAX_PWM        = 255;

// ---------- Turn controller (v12: gyro speed profile + PID on rotation speed) ----------
// GYRO_SCALE: robot physically turns TOO FAR -> increase it; NOT FAR ENOUGH -> decrease.
// (4 turns in the Turn test should bring it back to exactly the start direction)
float GYRO_SCALE     = 0.990;     // live-tunable ("Gyro" box) - v13: also self-learns from walls
float TURN_MAX_RATE  = 180.0;     // deg/s top rotation speed
float TURN_DECEL     = 300.0;     // deg/s^2 - LOWER = starts slowing earlier = less overshoot (live-tunable)
const float TURN_ACCEL     = 900.0;   // deg/s^2 spin-up (gentle = no slipping)
const float TURN_MIN_RATE  = 20.0;    // deg/s: arrive at the target this slowly
const float TURN_RATE_KP   = 0.30;    // PWM per deg/s of speed error
const float TURN_RATE_KI   = 2.0;     // PWM per deg of accumulated speed error
const float TURN_I_MAX     = 35.0;    // integral adds at most this much PWM
const int   TURN_START_PWM = 78;      // PWM where the robot just starts rotating (feed-forward)
const float TURN_FF_PER_DPS = 0.15;   // extra PWM per deg/s wanted (feed-forward)
const int   MAX_TURN_PWM   = 125;
const int   TURN_WHEEL_MIN_PWM = 70;  // each wheel gets at least this while turning (both always move)
volatile float lastTurnOvershoot = 0, lastTurnError = 0;
// v13: check + fix every turn
const float TURN_VERIFY_TOL        = 1.0;   // deg: end error allowed after the check
const unsigned long TURN_VERIFY_MS = 300;   // wait this long (yaw still tracked) before checking
const int   TURN_MAX_FIXES         = 3;     // correction turns at most
volatile float lastTurnFirstError = 0;      // error of the first pass (+ over / - under)
volatile int   lastTurnFixes = 0;
// v13: gyro-scale self-learning from wall fits after a turn
const float SCALE_LEARN_RATE = 0.25;        // use 25% of each measurement
const float SCALE_MIN = 0.90, SCALE_MAX = 1.10;
bool  headingKnown = true;                  // heading checked against a wall since the last disturbance
bool  learnArmed   = false;                 // next wall fit measures the last turn
float learnDelta   = 0;                     // size of that turn (deg, signed)
volatile int scaleLearnCount = 0;
volatile bool  turnTestReq = false;
// ---------- CELL TEST: "Drive N cells" test ----------
// FLOOD FILL: navigation mode (default = existing right-wall follower) and the one-shot
// "stop after one cell" request read by driveUntilEvent()
NavigationMode navigationMode = NAV_RIGHT_WALL;
bool floodOneCellDrive = false;
volatile bool  cellTestReq = false;        // set by /celltest, cleared when the test ends or by STOP
volatile int   cellTestN = 1;              // requested cells (1-7)
bool  cellTestActive   = false;            // true only while the test's straight is being driven
float cellTestTargetMM = 0;                // N x CELL_SIZE_MM
float cellTestPosMM    = 0;                // encoder-estimated distance, updated every control step
int   cellTestCount    = 0;                // test number since power-on
const unsigned long CELL_TEST_SETTLE_MS = 500;   // wait after braking before reading the encoders
const int CELL_TEST_MAX_N = 7;
char  cellReport[560] = "No Drive-N-cells test yet.";
const float TURN_TRIM_LEFT  = 1.0;                                        // same ratio as
const float TURN_TRIM_RIGHT = (float)FORWARD_RIGHT_SPEED / FORWARD_LEFT_SPEED; // your forward trim
const float TURN_SYNC_GAIN  = 1.5;   // PWM per count of difference between the two wheels
const float TURN_SYNC_MAX   = 50.0;
const float TURN_TOLERANCE  = 0.7;    // deg
const float SETTLE_RATE     = 6.0;    // deg/s = "stopped"
const unsigned long TURN_SETTLE_MS  = 150;
const unsigned long TURN_TIMEOUT_MS = 3500;

const int GYRO_SIGN = 1;   // a RIGHT turn must make yaw go toward -90
const unsigned long CONTROL_PERIOD_US = 5000;   // 200 Hz

// ---------- Globals ----------
Adafruit_MPU6050 mpu;
float gyroBiasZ = 0.0;

float yaw         = 0.0;   // deg, + = counter-clockwise / left
float yawRate     = 0.0;
float pathHeading = 0.0;
unsigned long lastYawUs = 0;
float lastRawRate = 0;     // v13: for trapezoid integration across short gaps

volatile long leftCount  = 0;
volatile long rightCount = 0;
portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

VL53L0X tofFront, tofLeft, tofRight;
bool frontOk = false, leftOk = false, rightOk = false;
volatile uint16_t frontMM = 0, leftMM = 0, rightMM = 0;
volatile uint32_t frontSeq = 0, leftSeq = 0, rightSeq = 0;   // +1 on every NEW reading

Median3 leftFilter, rightFilter;
unsigned long lastToFPoll = 0;

// ---------- v10: sensor watchdog ----------
const unsigned long TOF_STALE_MS   = 250;   // no new reading this long = STUCK
const unsigned long TOF_REINIT_MS  = 1000;  // try restarting a stuck sensor at most this often
volatile unsigned long frontLastMs = 0, leftLastMs = 0, rightLastMs = 0;
unsigned long frontReinitMs = 0, leftReinitMs = 0, rightReinitMs = 0;
int tofRestarts = 0;
volatile long i2cErrors = 0;   // v11: failed multiplexer switches (wiring / noise problems)
bool fresh(volatile unsigned long &lastMs) { return millis() - lastMs < TOF_STALE_MS; }

// ---------- v10: encoder health ----------
const unsigned long ENC_MIN_US   = 300;    // ignore encoder pulses closer than this (noise)
const float ENC_MISMATCH_RATIO   = 1.35;   // one wheel counting > 1.35x the other = suspect
volatile unsigned long leftLastEncUs = 0, rightLastEncUs = 0;
bool encWarned = false;

// ---------- v10/v12: blocked detection + recovery ----------
const unsigned long BLOCKED_MS = 600;      // BOTH wheels not turning this long while driving = blocked
int  blockedStreak = 0;                    // blocks in a row at the same place
long blockedAtCell = -999;
bool forceFrontBlocked = false;            // after repeated blocks: don't try straight again
// RECOVERY FIX: settings + bookkeeping for the improved recovery
const int SLIP_CHECK_MAX_MM        = 600;  // slip check only trusts front readings closer than this
const int SLIP_CONFIRM_WINDOWS     = 2;    // slip must be seen in this many 500 ms checks in a row
const int RECOVERY_WALL_CONFIRM_MM = 180;  // after repeated blocks: front must read below this to map a WALL
bool forceFrontConfirmed = false;          // RECOVERY FIX: the front sensor confirmed the repeated block
int  lastBlockCause = 0;                   // RECOVERY FIX: 1 = wheels stopped, 2 = slipping
int  lastBlockFront = 9999;                // RECOVERY FIX: front reading when the block was detected

// ---------- v12: front wall confirmation ----------
const int FRONT_CONFIRM_READINGS = 2;      // front must be "close" on this many NEW readings in a row

// ---------- v12: wall-angle heading correction ----------
bool  WALL_ALIGN_ON          = true;
const float WALL_ALIGN_WINDOW_MM = 90.0;   // fit the wall over this much travel
const float WALL_ALIGN_GAIN      = 0.5;    // apply half of the measured error each time
const float WALL_ALIGN_MAX_STEP  = 3.0;    // never correct more than this per fit (deg)
const float WALL_ALIGN_GAIN_TURN = 0.8;    // v13: first fit after a turn fixes most of the turn error
const float WALL_ALIGN_MAX_TURN  = 6.0;    // v13: ...and may correct up to this much
const float WALL_FIT_MAX_RMS     = 3.0;    // wall must be straight to within this (mm)
volatile int wallAlignCount = 0;
volatile float lastWallAlignDeg = 0;

// Least-squares line fit of side distance vs travelled distance
struct WallFit {
  int n = 0;
  float x0 = 0, sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0;
  float hMin = 999, hMax = -999, hSum = 0, xMax = 0;
  void reset() { n = 0; sx = sy = sxx = sxy = syy = 0; hMin = 999; hMax = -999; hSum = 0; xMax = 0; }
  void add(float x, float y, float h) {
    if (n == 0) x0 = x;
    x -= x0;
    n++; sx += x; sy += y; sxx += x * x; sxy += x * y; syy += y * y;
    if (x > xMax) xMax = x;
    hSum += h; if (h < hMin) hMin = h; if (h > hMax) hMax = h;
  }
  float span() { return xMax; }
  // returns true + slope (mm per mm) if the fit is good
  bool slope(float &b, float &rms) {
    float d = n * sxx - sx * sx;
    if (n < 6 || d <= 0) return false;
    b = (n * sxy - sx * sy) / d;
    float a = (sy - b * sx) / n;
    float sse = syy - a * sy - b * sxy;
    rms = sqrtf(max(0.0f, sse / n));
    return true;
  }
};

volatile bool runEnabled = false;
bool wasRunning = false;
volatile long cellsMoved = 0;

char stateBuf[2][112] = {"STARTING", ""};
volatile int stateIdx = 0;
const char* volatile robotState = stateBuf[0];

WebServer server(80);

const char* actionName(Action a) {
  switch (a) {
    case GO_FORWARD: return "FORWARD";
    case TURN_RIGHT: return "TURN RIGHT";
    case TURN_LEFT:  return "TURN LEFT";
    default:         return "TURN AROUND";
  }
}

// ============================================================
//  Web page (http://192.168.4.1)
// ============================================================
const char PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Micromouse</title>
<style>
 body{font-family:system-ui,sans-serif;background:#111;color:#eee;margin:0;padding:16px}
 h1{font-size:20px;margin:0 0 8px}
 #state{color:#8cf;margin-bottom:12px;min-height:1.2em}
 .row{display:flex;gap:10px;flex-wrap:wrap}
 .box{flex:1;min-width:90px;background:#222;border-radius:10px;padding:12px;text-align:center}
 .lbl{color:#aaa;font-size:14px}
 .val{font-size:30px;font-weight:bold;margin-top:4px}
 .btns{margin:12px 0;display:flex;gap:10px;flex-wrap:wrap}
 button{padding:10px 18px;font-size:16px;border:0;border-radius:8px;cursor:pointer}
 #go{background:#2a7;color:#fff} #stop{background:#c33;color:#fff}
 #log{margin-top:12px;background:#000;border-radius:8px;padding:8px;height:38vh;overflow-y:auto;
      font-family:monospace;font-size:13px;white-space:pre}
 .mz{display:grid;grid-template-columns:repeat(8,1fr);width:min(92vw,400px);aspect-ratio:1;background:#0d0d0d;border-radius:6px}
 .mz div{box-sizing:border-box;display:flex;align-items:center;justify-content:center;font-size:20px;font-weight:bold;position:relative}
 .mz .s{position:absolute;left:3px;top:1px;font-size:11px;color:#f6c453}
 .mz .fl{position:absolute;right:3px;bottom:1px;font-size:11px;font-weight:normal;color:#9fb3c8}
 .mapbox{display:flex;gap:14px;flex-wrap:wrap;align-items:flex-start;margin-top:12px}
 .mapinfo{background:#222;border-radius:10px;padding:12px;font-size:14px;line-height:1.7;min-width:220px;flex:1}
</style></head><body>
<h1>Micromouse - right wall follower</h1>
<div id="state">State: -</div>
<div class="btns">
 <button id="go" onclick="fetch('/go')">GO</button>
 <button id="stop" onclick="fetch('/stop')">STOP</button>
 <button onclick="fetch('/resetenc')">Reset encoders</button>
 <button onclick="paused=!paused;this.textContent=paused?'Resume log':'Pause log'">Pause log</button>
 <button onclick="downloadLog()">Download log</button>
 <button onclick="resetLog()">Reset log</button>
 <button onclick="fetch('/test')" style="background:#46a;color:#fff">Straight test</button>
 <button onclick="fetch('/turntest')" style="background:#46a;color:#fff">Turn test</button>
 <span style="align-self:center">Cells <input id="cn" size="2" value="1"></span>
 <button onclick="cellTest()" style="background:#46a;color:#fff">Drive N cells</button>
 <button onclick="fetch('/setfc')">Set FRONT centre = now</button>
</div>
<div class="btns" style="align-items:center">
 Kp <input id="kp" size="4"> Ki <input id="ki" size="4"> Kd <input id="kd" size="4">
 Wall <input id="wg" size="4"> Gyro <input id="gs" size="5"> Decel <input id="td" size="4"> Front stop mm <input id="fs" size="4">
 <button onclick="fetch(`/set?kp=${kp.value}&ki=${ki.value}&kd=${kd.value}&wg=${wg.value}&gs=${gs.value}&td=${td.value}&fs=${fs.value}`)">Apply gains</button>
</div>
<div class="row">
 <div class="box"><div class="lbl">LEFT mm</div><div class="val" id="l">-</div></div>
 <div class="box"><div class="lbl">FRONT mm</div><div class="val" id="f">-</div></div>
 <div class="box"><div class="lbl">RIGHT mm</div><div class="val" id="r">-</div></div>
</div>
<div class="row" style="margin-top:10px">
 <div class="box"><div class="lbl">Enc L</div><div class="val" id="el">-</div></div>
 <div class="box"><div class="lbl">Enc R</div><div class="val" id="er">-</div></div>
 <div class="box"><div class="lbl">Cells</div><div class="val" id="c">-</div></div>
 <div class="box"><div class="lbl">counts/mm</div><div class="val" id="cpm">-</div></div>
 <div class="box"><div class="lbl">I2C err / ToF restarts</div><div class="val" id="ie">-</div></div>
</div>
<div class="row" style="margin-top:10px">
 <div class="box"><div class="lbl">Heading err &deg;</div><div class="val" id="h">-</div></div>
 <div class="box"><div class="lbl">Last straight: max / avg &deg;</div><div class="val" id="st">-</div></div>
 <div class="box"><div class="lbl">Wobbles</div><div class="val" id="wb">-</div></div>
 <div class="box"><div class="lbl">Front centre mm</div><div class="val" id="fc">-</div></div>
 <div class="box"><div class="lbl">Last turn: 1st-pass err / fixes / final err &deg; (+over -under)</div><div class="val" id="to">-</div></div>
 <div class="box"><div class="lbl">Wall-align fixes (last &deg;)</div><div class="val" id="wa">-</div></div>
</div>
<h2 style="font-size:17px;margin:16px 0 6px">MAZE MAP</h2>
<div class="btns" style="align-items:center;margin:6px 0">
 Navigation <select id="nav"><option value="0">Right Wall</option><option value="1">Flood Fill</option></select>
 <button onclick="fetch('/navmode?m='+document.getElementById('nav').value).then(r=>r.text()).then(t=>{if(t!='OK')alert(t)})">Apply mode</button>
 Start corner <select id="msc"><option value="0">bottom-left (7,0)</option><option value="1">bottom-right (7,7)</option></select>
 <button onclick="fetch('/mapreset?sc='+document.getElementById('msc').value).then(r=>r.text()).then(t=>{if(t!='OK')alert(t)})">Reset Map</button>
 <span style="color:#aaa">place the robot FACING NORTH = the start cell's only exit</span>
</div>
<div class="mapbox">
 <div id="mz" class="mz"></div>
 <div class="mapinfo" id="mi">-</div>
</div>
<pre id="cr" style="background:#222;border-radius:10px;padding:10px;white-space:pre-wrap;margin-top:10px">Drive N cells report: -</pre>
<div id="log"></div>
<script>
// CELL TEST: start the test (whole number 1-7 only) and show the report
function cellTest(){
  const v=document.getElementById('cn').value.trim();
  if(!/^[1-7]$/.test(v)){ alert('Cells must be a whole number from 1 to 7'); return; }
  fetch('/celltest?n='+v).then(r=>r.text()).then(t=>{ if(t!='OK') alert(t); });
}
async function crTick(){
  try{ document.getElementById('cr').textContent=await (await fetch('/cellres')).text(); }catch(e){}
  setTimeout(crTick,1000);
}
crTick();
// MAZE MAP: draw the 8x8 map (bits: 1=N 2=E 4=S 8=W; known nibble + wall nibble per cell)
let mapCornerLoaded=false;
function mapEdge(k,w,bit){ if(!(k&bit)) return '1px dashed #555'; return (w&bit)?'3px solid #ff5a5a':'1px solid #262626'; }
async function mapTick(){
  try{
    const m=await (await fetch('/mapdata')).json();
    if(!mapCornerLoaded){ document.getElementById('msc').value=m.sc; document.getElementById('nav').value=m.nav; mapCornerLoaded=true; }
    const g=document.getElementById('mz'); let h='';
    const sr=7, sc=m.sc?7:0, arrows=['\u2191','\u2192','\u2193','\u2190'], names=['NORTH','EAST','SOUTH','WEST'];
    for(let r=0;r<8;r++) for(let c=0;c<8;c++){
      const i=r*8+c, k=parseInt(m.k[i*2],16), w=parseInt(m.k[i*2+1],16), vis=m.vis[i]=='1', robot=(r==m.r&&c==m.c);
      const bg=robot?'#5a4710':(vis?'#1d3550':'#151515');
      h+=`<div style="background:${bg};border-top:${mapEdge(k,w,1)};border-right:${mapEdge(k,w,2)};border-bottom:${mapEdge(k,w,4)};border-left:${mapEdge(k,w,8)}">`+
         `${(r==sr&&c==sc)?'<span class="s">S</span>':''}${robot?`<span style="color:#ffd166">${arrows[m.d]}</span>`:''}`+
         `<span class="fl">${m.f?(parseInt(m.f.substr(i*2,2),16)==255?'\u00b7':parseInt(m.f.substr(i*2,2),16)):''}</span></div>`;
    }
    g.innerHTML=h;
    document.getElementById('mi').innerHTML=
      `Current cell: <b>(${m.r}, ${m.c})</b><br>Facing: <b>${names[m.d]}</b><br>`+
      `Cell transitions: <b>${m.t}</b><br>Unique visited cells: <b>${m.v} / 64</b><br>`+
      `Position: <b>${m.b?'less certain ('+m.b+' blocked stop'+(m.b>1?'s':'')+')':'OK'}</b><br>`+
      `Map warnings: <b>${m.w}</b><br><span style="color:#aaa">Last: ${m.e}</span><br>`+
      `Navigation: <b>${m.nav==1?'FLOOD FILL':'RIGHT WALL'}</b><br>`+
      `Current flood value: <b>${m.fv<0?'no path':m.fv}</b><br>`+
      `Target direction: <b>${m.td<0?'-':names[m.td]}</b><br><span style="color:#aaa">Flood: ${m.fs}</span><br>`+
      `<span style="color:#aaa">red = wall, dashed = unknown, faint = open, blue = visited</span>`;
  }catch(e){}
  setTimeout(mapTick,500);
}
mapTick();
let paused=false, lastS='', gainsLoaded=false; const log=document.getElementById('log');
// LOG DOWNLOAD: every line is also kept here in full (the on-screen log trims old lines)
let fullLog=[];
function downloadLog(){
  const n=new Date(), p=x=>String(x).padStart(2,'0');
  const name=`micromouse_log_${n.getFullYear()}-${p(n.getMonth()+1)}-${p(n.getDate())}_${p(n.getHours())}-${p(n.getMinutes())}-${p(n.getSeconds())}.txt`;
  const url=URL.createObjectURL(new Blob([fullLog.join('')],{type:'text/plain'}));
  const a=document.createElement('a'); a.href=url; a.download=name;
  document.body.appendChild(a); a.click(); a.remove();
  setTimeout(()=>URL.revokeObjectURL(url),1000);
}
function resetLog(){ fullLog=[]; log.textContent=''; }
function fmt(ok,v,st){ if(!ok) return 'ERR'; if(st) return 'STUCK'; if(v==0||v>=%MAXMM%) return '---'; return v; }
async function tick(){
  try{
    const d=await (await fetch('/data')).json();
    const f=fmt(d.fok,d.f,d.fst), l=fmt(d.lok,d.l,d.lst), r=fmt(d.rok,d.r,d.rst);
    for (const [k,v] of [['f',f],['l',l],['r',r],['el',d.el],['er',d.er],['c',d.c],['cpm',d.cpm.toFixed(2)+(d.cal?' âœ“':' ?')],['ie',d.ie+' / '+d.tr]])
      document.getElementById(k).textContent=v;
    document.getElementById('state').textContent=(d.run?'RUNNING - ':'STOPPED - ')+d.s;
    document.getElementById('h').textContent=d.h.toFixed(1);
    document.getElementById('st').textContent=d.mx.toFixed(1)+' / '+d.av.toFixed(1);
    document.getElementById('wb').textContent=d.wb;
    document.getElementById('fc').textContent=d.fc;
    document.getElementById('to').textContent=d.tf.toFixed(1)+' / '+d.tn+' / '+d.te.toFixed(1);
    document.getElementById('wa').textContent=d.wa+' ('+d.wd.toFixed(1)+')';
    if(!gainsLoaded){ kp.value=d.kp; ki.value=d.ki; kd.value=d.kd; wg.value=d.wg; gs.value=d.gs; td.value=d.td; fs.value=d.fs; gainsLoaded=true; }
    fullLog.push(`${(d.t/1000).toFixed(1)}s  F:${f} L:${l} R:${r}  enc:${d.el}/${d.er}  [${d.s}]\n`);   // LOG DOWNLOAD
    if(!paused){
      log.textContent+=`${(d.t/1000).toFixed(1)}s  F:${f} L:${l} R:${r}  enc:${d.el}/${d.er}  [${d.s}]\n`;
      if(log.textContent.length>20000) log.textContent=log.textContent.slice(-15000);
      log.scrollTop=log.scrollHeight;
    }
  }catch(e){ document.getElementById('state').textContent='connection lost...'; }
  setTimeout(tick,150);
}
tick();
</script></body></html>
)rawliteral";

void readEncoders(long &l, long &r);

void handleRoot() {
  String page = FPSTR(PAGE_HTML);
  page.replace("%MAXMM%", String(TOF_MAX_VALID_MM));
  server.send(200, "text/html", page);
}

void handleData() {
  long el, er;
  readEncoders(el, er);
  char json[800];
  snprintf(json, sizeof(json),
           "{\"t\":%lu,\"f\":%u,\"l\":%u,\"r\":%u,\"fok\":%d,\"lok\":%d,\"rok\":%d,"
           "\"fst\":%d,\"lst\":%d,\"rst\":%d,\"ie\":%ld,\"tr\":%d,"
           "\"el\":%ld,\"er\":%ld,\"c\":%ld,\"cpm\":%.3f,\"cal\":%d,\"run\":%d,"
           "\"h\":%.2f,\"mx\":%.2f,\"av\":%.2f,\"wb\":%d,"
           "\"kp\":%.2f,\"ki\":%.2f,\"kd\":%.2f,\"wg\":%.3f,"
           "\"gs\":%.4f,\"td\":%.0f,\"fc\":%d,\"fs\":%d,\"to\":%.2f,\"te\":%.2f,\"tf\":%.2f,\"tn\":%d,\"wa\":%d,\"wd\":%.2f,\"s\":\"%s\"}",
           millis(), frontMM, leftMM, rightMM, frontOk, leftOk, rightOk,
           (int)(frontOk && !fresh(frontLastMs)), (int)(leftOk && !fresh(leftLastMs)),
           (int)(rightOk && !fresh(rightLastMs)), (long)i2cErrors, tofRestarts,
           el, er, (long)cellsMoved, countsPerMM, (int)distCalibrated, (int)runEnabled,
           (float)liveHeading, (float)statMaxHdg, (float)statAvgHdg, (int)statCrossings,
           Kp, Ki, Kd, WALL_CENTER_GAIN,
           GYRO_SCALE, TURN_DECEL, frontCenterMM, frontStopMM, (float)lastTurnOvershoot, (float)lastTurnError,
           (float)lastTurnFirstError, (int)lastTurnFixes,
           (int)wallAlignCount, (float)lastWallAlignDeg, (const char*)robotState);
  server.send(200, "application/json", json);
}

void handleGo()   { runEnabled = true;  server.send(200, "text/plain", "GO"); }
void handleStop() { runEnabled = false; straightTestReq = false; turnTestReq = false;
                    cellTestReq = false;   // CELL TEST: a cancelled test must not start on the next GO
                    server.send(200, "text/plain", "STOP"); }
void handleTest() { straightTestReq = true; runEnabled = true; server.send(200, "text/plain", "TEST"); }

// CELL TEST: /celltest?n=3  -> drive 3 cells straight. Only accepted while the robot is idle
// and no other test is waiting, so two requests can never cause unexpected motion.
void handleCellTest() {
  if (!server.hasArg("n")) { server.send(400, "text/plain", "missing n (1-7)"); return; }
  float v = server.arg("n").toFloat();
  int n = (int)v;
  if (v != (float)n || n < 1 || n > CELL_TEST_MAX_N) {
    server.send(400, "text/plain", "N must be a whole number from 1 to 7");
    return;
  }
  if (runEnabled || straightTestReq || turnTestReq || cellTestReq) {
    server.send(409, "text/plain", "BUSY - wait until the robot is idle (or press STOP first)");
    return;
  }
  cellTestN   = n;
  cellTestReq = true;
  runEnabled  = true;
  server.send(200, "text/plain", "OK");
}
void handleCellReport() { server.send(200, "text/plain", cellReport); }   // CELL TEST
void handleTurnTest() { turnTestReq = true; runEnabled = true; server.send(200, "text/plain", "TURNTEST"); }
int dist(bool ok, volatile uint16_t &mm);
void setState(const char* fmt, ...);
// v12: place the robot with its wheels over a cell centre facing a wall, then press this
void handleSetFc() {
  int f = dist(frontOk, frontMM);
  if (f > 10 && f < 200) {
    frontCenterMM = f;
    frontWallMM = frontCenterMM + CELL_SIZE_MM / 2;
    setState("FRONT centre set to %d mm (wall threshold %d mm) - copy into the sketch to keep it",
             frontCenterMM, frontWallMM);
    server.send(200, "text/plain", "OK");
  } else {
    server.send(200, "text/plain", "no wall in range");
  }
}

// Live gain changes: /set?kp=3&ki=1.5&kd=0.6&wg=0.15
void handleSet() {
  if (server.hasArg("kp")) Kp = server.arg("kp").toFloat();
  if (server.hasArg("ki")) { Ki = server.arg("ki").toFloat(); headingIntegral = 0; }
  if (server.hasArg("kd")) Kd = server.arg("kd").toFloat();
  if (server.hasArg("wg")) WALL_CENTER_GAIN = server.arg("wg").toFloat();
  if (server.hasArg("gs")) { float g = server.arg("gs").toFloat(); if (g > 0.8 && g < 1.2) GYRO_SCALE = g; }
  if (server.hasArg("td")) { float t = server.arg("td").toFloat(); if (t >= 100 && t <= 2000) TURN_DECEL = t; }
  if (server.hasArg("fs")) { int v = (int)server.arg("fs").toFloat(); if (v >= 30 && v <= 150) frontStopMM = v; }
  Serial.printf("Front stop distance: %d mm\n", frontStopMM);
  Serial.printf("Gains set: Kp %.2f Ki %.2f Kd %.2f wall %.3f\n", Kp, Ki, Kd, WALL_CENTER_GAIN);
  server.send(200, "text/plain", "OK");
}
void handleResetEnc() {
  portENTER_CRITICAL(&encMux);
  leftCount = 0;
  rightCount = 0;
  portEXIT_CRITICAL(&encMux);
  server.send(200, "text/plain", "RESET");
}

void webTask(void *) {
  for (;;) {
    server.handleClient();
    vTaskDelay(2 / portTICK_PERIOD_MS);
  }
}

void setState(const char* fmt, ...) {
  int next = stateIdx ^ 1;
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(stateBuf[next], sizeof(stateBuf[next]), fmt, ap);
  va_end(ap);
  stateIdx = next;
  robotState = stateBuf[next];
  Serial.println(stateBuf[next]);
}

// ============================================================
//  Encoders
// ============================================================
// v10: pulses closer together than ENC_MIN_US are electrical noise, not wheel motion
// (at full speed a real pulse comes every ~1.5-3 ms)
void IRAM_ATTR leftEncoderISR() {
  unsigned long now = micros();
  if (now - leftLastEncUs < ENC_MIN_US) return;
  leftLastEncUs = now;
  portENTER_CRITICAL_ISR(&encMux);
  leftCount++;
  portEXIT_CRITICAL_ISR(&encMux);
}

void IRAM_ATTR rightEncoderISR() {
  unsigned long now = micros();
  if (now - rightLastEncUs < ENC_MIN_US) return;
  rightLastEncUs = now;
  portENTER_CRITICAL_ISR(&encMux);
  rightCount++;
  portEXIT_CRITICAL_ISR(&encMux);
}

// v10: are two wheel-count deltas believable together? (small moves always OK)
bool encodersAgree(long dl, long dr) {
  long hi = max(dl, dr), lo = min(dl, dr);
  if (hi < 40) return true;
  return hi <= ENC_MISMATCH_RATIO * lo + 10;
}

void readEncoders(long &l, long &r) {
  portENTER_CRITICAL(&encMux);
  l = leftCount;
  r = rightCount;
  portEXIT_CRITICAL(&encMux);
}

// ============================================================
//  I2C / gyro
// ============================================================

// Returns false if the multiplexer didn't answer -> the caller must NOT read,
// otherwise it would read whichever sensor was selected before
bool selectMux(uint8_t channel) {
  Wire.beginTransmission(TCA_ADDRESS);
  Wire.write(1 << channel);
  if (Wire.endTransmission() != 0) { i2cErrors++; return false; }
  return true;
}

float readGyroZRaw() {
  selectMux(MPU_CHANNEL);
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  return g.gyro.z * 180.0 / PI;
}

void calibrateGyro(int samples) {
  float sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += readGyroZRaw();
    delay(2);
  }
  gyroBiasZ = sum / samples;
  Serial.print("Gyro Z bias (deg/s): ");
  Serial.println(gyroBiasZ, 4);
}

// v13: yaw is integrated ALL the time (telemetryTick calls updateYaw too), so rotation
// during pauses / sliding after a stop is no longer lost. "Reset" now just brings yaw
// up to date instead of throwing the elapsed time away.
void updateYaw() {
  unsigned long now = micros();
  float dt = (now - lastYawUs) / 1e6;
  lastYawUs = now;
  float rate = GYRO_SIGN * GYRO_SCALE * (readGyroZRaw() - gyroBiasZ);   // v12: scale calibration
  if (dt > 0.5f) dt = 0.005f;                  // long gap (boot / calibration): don't guess
  yaw += 0.5f * (rate + lastRawRate) * dt;     // trapezoid: accurate even over a short gap
  lastRawRate = rate;
  yawRate = rate;
}
void resetYawTimer() { updateYaw(); }

// ============================================================
//  ToF sensors
// ============================================================
// v13: free a stuck I2C bus - a sensor that was cut off mid-byte can hold SDA low
// forever; 9 clock pulses + a STOP let it finish, then the I2C driver is restarted.
void i2cBusRecover() {
  Wire.end();
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL_PIN, LOW);  delayMicroseconds(6);
    digitalWrite(SCL_PIN, HIGH); delayMicroseconds(6);
  }
  pinMode(SDA_PIN, OUTPUT);                    // STOP: SDA low -> high while SCL high
  digitalWrite(SDA_PIN, LOW);  delayMicroseconds(6);
  digitalWrite(SCL_PIN, HIGH); delayMicroseconds(6);
  digitalWrite(SDA_PIN, HIGH); delayMicroseconds(6);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Serial.println("I2C bus recovered (9 clocks + restart)");
}

// v13: one start attempt: stop, soft reset, init, start, then wait for a REAL reading
bool tryStartToF(VL53L0X &s, uint8_t channel, int attempt) {
  if (!selectMux(channel)) return false;
  s.setTimeout(50);
  if (attempt > 1) {                          // soft reset (register 0xBF) from the 2nd try on
    s.writeReg(0xBF, 0x00); delay(2);
    s.writeReg(0xBF, 0x01); delay(5);
  } else {
    s.stopContinuous(); delay(2);
  }
  if (!selectMux(channel)) return false;
  if (!s.init()) return false;
  s.startContinuous(0);
  unsigned long t0 = millis();                // "init OK" is not enough - a reading must arrive
  while (millis() - t0 < 120) {
    if (!selectMux(channel)) return false;
    if ((s.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) != 0) return true;
    delay(5);
  }
  return false;
}

// v13: several attempts with growing pauses; if all fail, free the bus and try once more
bool startToFWithRetries(VL53L0X &s, uint8_t channel, const char *name) {
  const int TRIES = 3;
  for (int a = 1; a <= TRIES; a++) {
    if (tryStartToF(s, channel, a)) {
      Serial.printf("%s ToF started (attempt %d)\n", name, a);
      return true;
    }
    Serial.printf("%s ToF attempt %d failed\n", name, a);
    updateYaw();                               // keep the heading tracked while we wait
    delay(10 * a);
  }
  i2cBusRecover();
  bool ok = tryStartToF(s, channel, TRIES + 1);
  Serial.printf("%s ToF after bus recovery: %s\n", name, ok ? "OK" : "STILL FAILING");
  return ok;
}

bool initToF(VL53L0X &s, uint8_t channel, const char *name) {
  bool ok = startToFWithRetries(s, channel, name);
  Serial.print(name); Serial.println(ok ? " ToF ready" : " ToF NOT found! (check its wires / 3.3V)");
  return ok;
}

// Non-blocking read; bumps seq on every new measurement; removes the sensor's offset;
// optional median-of-3 filter (side sensors only - the front needs the fastest response)
void pollToF(VL53L0X &s, uint8_t channel, bool ok,
             volatile uint16_t &mm, volatile uint32_t &seq, int offsetMM, Median3 *filter,
             volatile unsigned long &lastMs) {
  if (!ok) return;
  if (!selectMux(channel)) return;            // v11: don't read the wrong sensor
  if ((s.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) return;
  uint16_t raw = s.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
  s.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
  uint16_t corrected;
  if (raw == 0 || raw >= TOF_MAX_VALID_MM) corrected = TOF_MAX_VALID_MM;   // "no target"
  else corrected = (uint16_t)max(1, (int)raw - offsetMM);                  // remove offset
  mm = filter ? filter->push(corrected) : corrected;
  seq++;
  lastMs = millis();
}

// v10: restart a sensor that stopped producing readings
// v13: several attempts + bus recovery (startToFWithRetries)
void restartToF(VL53L0X &s, uint8_t channel, const char *name) {
  bool ok = startToFWithRetries(s, channel, name);
  tofRestarts++;
  Serial.printf("%s ToF was STUCK -> restart %s (restart #%d)\n", name, ok ? "OK" : "FAILED", tofRestarts);
}

void telemetryTick() {
  if (micros() - lastYawUs >= 5000) updateYaw();   // v13: yaw never stops being tracked
  unsigned long now = millis();
  if (now - lastToFPoll < TOF_POLL_PERIOD_MS) return;
  lastToFPoll = now;
  pollToF(tofFront, FRONT_TOF_CHANNEL, frontOk, frontMM, frontSeq, FRONT_TOF_OFFSET_MM, nullptr,      frontLastMs);
  pollToF(tofLeft,  LEFT_TOF_CHANNEL,  leftOk,  leftMM,  leftSeq,  LEFT_TOF_OFFSET_MM,  &leftFilter,  leftLastMs);
  pollToF(tofRight, RIGHT_TOF_CHANNEL, rightOk, rightMM, rightSeq, RIGHT_TOF_OFFSET_MM, &rightFilter, rightLastMs);

  // v10 watchdog: restart any sensor that has gone quiet
  if (frontOk && !fresh(frontLastMs) && now - frontReinitMs > TOF_REINIT_MS) {
    frontReinitMs = now; restartToF(tofFront, FRONT_TOF_CHANNEL, "Front"); frontReinitMs = millis();
  }
  if (leftOk && !fresh(leftLastMs) && now - leftReinitMs > TOF_REINIT_MS) {
    leftReinitMs = now; restartToF(tofLeft, LEFT_TOF_CHANNEL, "Left"); leftReinitMs = millis();
  }
  if (rightOk && !fresh(rightLastMs) && now - rightReinitMs > TOF_REINIT_MS) {
    rightReinitMs = now; restartToF(tofRight, RIGHT_TOF_CHANNEL, "Right"); rightReinitMs = millis();
  }
}

// v10: wait (up to maxMs) until all working sensors are giving fresh readings
bool waitForFreshSensors(unsigned long maxMs) {
  unsigned long start = millis();
  while (millis() - start < maxMs) {
    telemetryTick();
    bool allFresh = (!frontOk || fresh(frontLastMs)) && (!leftOk || fresh(leftLastMs)) &&
                    (!rightOk || fresh(rightLastMs));
    if (allFresh) return true;
    delay(5);
  }
  return false;
}

void waitWithTelemetry(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    telemetryTick();
    delay(2);
  }
}

// Distance in mm, or 9999 if missing / nothing in range
int dist(bool ok, volatile uint16_t &mm) {
  uint16_t v = mm;
  if (!ok || v == 0 || v >= TOF_MAX_VALID_MM) return 9999;
  return v;
}

// ============================================================
//  Motors
// ============================================================
void setMotors(int dir, int leftPWM, int rightPWM) {
  if (dir > 0) {
    analogWrite(IN1, 0);
    analogWrite(IN2, leftPWM);
    analogWrite(IN3, rightPWM);
    analogWrite(IN4, 0);
  } else {
    analogWrite(IN1, leftPWM);
    analogWrite(IN2, 0);
    analogWrite(IN3, 0);
    analogWrite(IN4, rightPWM);
  }
}

// In-place spin: BOTH wheels driven, in opposite directions
void spinWheels(bool clockwise, int leftPWM, int rightPWM) {
  if (clockwise) {             // turn right: left wheel forward, right wheel backward
    analogWrite(IN1, 0);
    analogWrite(IN2, leftPWM);
    analogWrite(IN3, 0);
    analogWrite(IN4, rightPWM);
  } else {                     // turn left: left wheel backward, right wheel forward
    analogWrite(IN1, leftPWM);
    analogWrite(IN2, 0);
    analogWrite(IN3, rightPWM);
    analogWrite(IN4, 0);
  }
}

void brakeMotors() {
  analogWrite(IN1, 255);
  analogWrite(IN2, 255);
  analogWrite(IN3, 255);
  analogWrite(IN4, 255);
}

void stopMotors() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}

// Gentle stop: coasting first lets the wheels slow with the robot instead of
// locking up and skidding; a short brake at the end stops the last bit of rolling.
void brakeAndStop() {
  stopMotors();
  delay(COAST_MS);
  brakeMotors();
  delay(SOFT_BRAKE_MS);
  stopMotors();
}

// ============================================================
//  Side-wall edge tracker
//  Confirms a wall/opening change after 2 consecutive NEW readings.
// ============================================================
struct EdgeTracker {
  int confirmed = -1;     // -1 unknown, 0 open, 1 wall
  int candidate = -1;
  int candCount = 0;
  uint32_t lastSeq = 0;

  // returns true when a confirmed change (wall->open or open->wall) happens
  bool update(bool ok, volatile uint16_t &mm, volatile uint32_t &seq) {
    if (!ok || seq == lastSeq) return false;
    lastSeq = seq;
    int s = dist(ok, mm) < sideWallMM ? 1 : 0;
    if (confirmed == -1) { confirmed = s; return false; }
    if (s == confirmed) { candCount = 0; return false; }
    if (s == candidate) candCount++; else { candidate = s; candCount = 1; }
    if (candCount >= 2) { confirmed = s; candCount = 0; return true; }
    return false;
  }
};

// ============================================================
//  DRIVE UNTIL SOMETHING INTERESTING HAPPENS (efficient right-wall following)
//  Drives straight through corridors WITHOUT stopping in every cell, and while
//  moving it keeps checking:
//   - the RIGHT side of every cell it's about to reach. If a cell has an
//     opening on the right, it slows down and stops with the axle at that
//     cell's centre, ready to turn right.
//   - the FRONT wall: stops at the cell centre in front of a wall.
//  Position (mm) comes from the encoders and is corrected by:
//   - wall edges seen by either side sensor (the sensor is then exactly on a
//     cell boundary)
//   - self-calibration of counts/mm from the front sensor while approaching a wall
// ============================================================
const char* reasonName(StopReason r) {
  switch (r) {
    case STOP_FRONT_WALL: return "front wall";
    case STOP_RIGHT_OPEN: return "right opening";
    case STOP_UNSURE:     return "unsure - checking";
    case STOP_TIMEOUT:    return "timeout";
    case STOP_BLOCKED:    return "BLOCKED - wheels stopped turning";
    case STOP_FRONT_STUCK:return "FRONT SENSOR STUCK - safety stop";
    case STOP_CELL_TARGET:return "distance target reached";      // CELL TEST
    case STOP_FLOOD_CELL: return "flood fill: next cell centre reached";   // FLOOD FILL
    default:              return "stopped";
  }
}

// ignoreRight = true: just drive straight to the next front wall (used by Straight test)
bool driveUntilEvent(int &cellsDone, StopReason &why, bool ignoreRight) {
  const float HALF = CELL_SIZE_MM / 2.0;
  long sL, sR;
  readEncoders(sL, sR);

  float posOffset = 0;            // mm corrections from wall edges
  int   nextK = 1;                // next cell centre ahead of us (1 = next cell)
  int   openVotes = 0, totalVotes = 0;
  int   leftOpenVotes = 0, leftVotes = 0;   // MAZE MAP: left side of each cell (recorded only, not used for driving)
  float plannedStop = -1;         // mm, where to stop (cell centre), -1 = none
  why = STOP_TIMEOUT;
  if (cellTestActive) {           // CELL TEST: distance target set from the very beginning
    plannedStop = cellTestTargetMM;
    why = STOP_CELL_TARGET;
  }
  // FLOOD FILL: optional one-cell stopping mode. The flag is read and cleared here, so it can
  // never leak into a later drive. Existing right-wall behaviour is unchanged when it is not set.
  bool floodOneCell = floodOneCellDrive;
  floodOneCellDrive = false;
  if (floodOneCell && !cellTestActive) {
    plannedStop = CELL_SIZE_MM;     // stop at the centre of the next cell (same stopping code as a planned stop)
    why = STOP_FLOOD_CELL;
  }

  EdgeTracker edgeL, edgeR;
  uint32_t lastRSeq = rightSeq, lastFSeq = frontSeq;
  uint32_t lastLSeqMap = leftSeq;           // MAZE MAP
  float calCounts0 = -1;          // front-sensor self-calibration anchor
  int   calFront0  = 0;

  resetYawTimer();
  unsigned long startMs   = millis();
  unsigned long lastUs    = micros();
  unsigned long lastPrint = 0;
  float pos = 0;

  // v10: blocked detection
  long blkL = sL, blkR = sR;
  unsigned long blkLMs = millis(), blkRMs = millis();
  // v12: slip-against-obstacle check + front confirmation + wall-angle fits
  unsigned long slipCheckMs = 0;
  float slipEnc0 = 0;
  int   slipF0 = 0;
  int   slipHits = 0;                 // RECOVERY FIX: slip windows in a row
  uint32_t slipSeq0 = frontSeq;       // RECOVERY FIX: front reading count at the window start
  uint32_t lastFrontStopSeq = frontSeq;
  int   frontCloseCnt = 0;
  WallFit fitL, fitR;
  uint32_t lastLFit = leftSeq, lastRFit = rightSeq;

  // straightness statistics (after the start ramp)
  float sumAbsHdg = 0, maxAbsHdg = 0;
  long  nHdg = 0;
  int   crossings = 0, lastSign = 0;

  while (true) {
    if (!runEnabled) { brakeAndStop(); why = STOP_ABORT; cellsDone = 0; return false; }
    telemetryTick();

    unsigned long nowUs = micros();
    if (nowUs - lastUs < CONTROL_PERIOD_US) continue;
    float dt = (nowUs - lastUs) / 1e6;
    lastUs = nowUs;

    updateYaw();
    float heading = yaw - pathHeading;

    long l, r;
    readEncoders(l, r);
    // v10: if one encoder counts far more than the other, it's picking up noise ->
    // use the smaller (believable) count for distance
    long dL = l - sL, dR = r - sR;
    float cnt;
    if (encodersAgree(dL, dR)) {
      cnt = (dL + dR) / 2.0;
    } else {
      cnt = min(dL, dR);
      if (!encWarned) {
        encWarned = true;
        setState("WARNING: %s encoder counts %.1fx the other - using %s wheel for distance",
                 dL > dR ? "LEFT" : "RIGHT", (float)max(dL, dR) / max(1L, min(dL, dR)),
                 dL > dR ? "right" : "left");
      }
    }

    // v10: blocked? (a wheel that stops turning while we're driving = pushing a wall)
    if (l != blkL) { blkL = l; blkLMs = millis(); }
    if (r != blkR) { blkR = r; blkRMs = millis(); }

    int f = dist(frontOk, frontMM);

    // ---- Self-calibrate counts per mm using the front sensor ----
    if (frontSeq != lastFSeq && !cellTestActive) {   // CELL TEST: no distance self-cal during the test
      lastFSeq = frontSeq;
      bool steady = (millis() - startMs) > ACCEL_RAMP_MS;   // not accelerating = not slipping
      if (steady && f > 120 && f < 1100 && fabs(heading) < 6) {
        if (calCounts0 < 0) { calCounts0 = cnt; calFront0 = f; }
        else if (calFront0 - f >= 150) {
          float est = (cnt - calCounts0) / (float)(calFront0 - f);
          // Accept only estimates near the measured ~1.57 counts/mm (slip gives nonsense)
          float nominal = COUNTS_PER_WHEEL_REV / (PI * WHEEL_DIAMETER_MM);
          if (est > 0.75 * nominal && est < 1.35 * nominal) {
            countsPerMM = distCalibrated ? 0.7 * countsPerMM + 0.3 * est : est;
            distCalibrated = true;
            Serial.printf("Distance self-cal: %.2f counts/mm (this sample %.2f)\n", countsPerMM, est);
          }
          calCounts0 = cnt; calFront0 = f;
        }
      } else {
        calCounts0 = -1;
      }
    }

    pos = cnt / countsPerMM + posOffset;
    if (cellTestActive) cellTestPosMM = pos;          // CELL TEST: last encoder estimate (= when braking began)

    // ---- Wall edges: side sensor is exactly on a cell boundary -> fix position ----
    bool eL = edgeL.update(leftOk,  leftMM,  leftSeq);
    bool eR = edgeR.update(rightOk, rightMM, rightSeq);
    if ((eL || eR) && !cellTestActive) {               // CELL TEST: no wall-edge snapping during the test
      float sp = pos + SIDE_SENSOR_AHEAD_MM;                         // sensor position
      float b  = roundf((sp - HALF) / CELL_SIZE_MM) * CELL_SIZE_MM + HALF; // nearest boundary
      float corr = (b + EDGE_LAG_MM) - sp;
      if (fabs(corr) < 45) {
        posOffset += corr;
        pos += corr;
        Serial.printf("EDGE %s: position corrected by %.0f mm -> %.0f mm\n",
                      eR ? "right" : "left", corr, pos);
      }
    }

    // ---- v12: WALL-ANGLE HEADING CORRECTION ----
    // Next to a straight wall, (side distance) vs (distance travelled) is a line whose
    // slope is the robot's REAL angle to the wall. Compare with the gyro's heading and
    // nudge the gyro reference so the two agree -> small turn errors disappear.
    if (eL) fitL.reset();
    if (eR) fitR.reset();
    {
      bool cruising = millis() - startMs > ACCEL_RAMP_MS + 150;
      if (leftSeq != lastLFit) {
        lastLFit = leftSeq;
        int lmF = dist(leftOk, leftMM);
        if (WALL_ALIGN_ON && cruising && lmF < sideWallMM - 30) fitL.add(pos, lmF, heading);
        else fitL.reset();
      }
      if (rightSeq != lastRFit) {
        lastRFit = rightSeq;
        int rmF = dist(rightOk, rightMM);
        if (WALL_ALIGN_ON && cruising && rmF < sideWallMM - 30) fitR.add(pos, rmF, heading);
        else fitR.reset();
      }
      float bL, rmsL, bR, rmsR;
      bool okL = fitL.span() >= WALL_ALIGN_WINDOW_MM && fitL.slope(bL, rmsL) && rmsL < WALL_FIT_MAX_RMS
                 && (fitL.hMax - fitL.hMin) < 1.5;
      bool okR = fitR.span() >= WALL_ALIGN_WINDOW_MM && fitR.slope(bR, rmsR) && rmsR < WALL_FIT_MAX_RMS
                 && (fitR.hMax - fitR.hMin) < 1.5;
      if (okL || okR) {
        // right wall getting further away = pointing LEFT (+); left wall further = pointing RIGHT (-)
        float thR = okR ? atanf(bR) * 180.0 / PI : 0;
        float thL = okL ? -atanf(bL) * 180.0 / PI : 0;
        float hR  = okR ? fitR.hSum / fitR.n : 0;
        float hL  = okL ? fitL.hSum / fitL.n : 0;
        float wallAngle = (okL && okR) ? (thL + thR) / 2 : (okR ? thR : thL);
        float gyroAngle = (okL && okR) ? (hL + hR) / 2 : (okR ? hR : hL);
        float d = gyroAngle - wallAngle;          // how wrong the gyro reference is
        if (fabs(d) < 8.0) {
          // v13: the first fit after a turn measures that turn's REAL error
          bool afterTurn = !headingKnown;
          if (learnArmed && fabs(learnDelta) > 45 && fabs(d) < 6.0) {
            // gyro said the turn was exactly learnDelta; the wall says the robot is d deg
            // off -> real turn = learnDelta - d -> scale must change by -d/learnDelta
            float frac = -d / learnDelta;               // + = turned too far -> raise scale
            float old = GYRO_SCALE;
            GYRO_SCALE = constrain(GYRO_SCALE * (1.0f + SCALE_LEARN_RATE * frac), SCALE_MIN, SCALE_MAX);
            scaleLearnCount++;
            Serial.printf("GYRO-LEARN: last %+.0f deg turn really ended %+.1f deg %s -> scale %.4f -> %.4f\n",
                          learnDelta, fabs(d), (frac > 0) ? "TOO FAR" : "SHORT", old, GYRO_SCALE);
          }
          learnArmed = false;
          float gain = afterTurn ? WALL_ALIGN_GAIN_TURN : WALL_ALIGN_GAIN;
          float mx   = afterTurn ? WALL_ALIGN_MAX_TURN  : WALL_ALIGN_MAX_STEP;
          float corr = constrain(gain * d, -mx, mx);
          pathHeading += corr;
          headingKnown = true;
          wallAlignCount++;
          lastWallAlignDeg = corr;
          Serial.printf("WALL-ALIGN: wall %.1f deg, gyro %.1f deg -> heading ref %+.2f deg%s\n",
                        wallAngle, gyroAngle, corr, afterTurn ? " (after turn)" : "");
        }
        fitL.reset();
        fitR.reset();
      }
    }

    // ---- MAZE MAP: count LEFT-wall readings for the next cell (same window as the right side) ----
    if (!ignoreRight && plannedStop < 0 && leftSeq != lastLSeqMap) {
      lastLSeqMap = leftSeq;
      float spL = pos + SIDE_SENSOR_AHEAD_MM;
      if (spL >= nextK * CELL_SIZE_MM - HALF + 20) {       // sensor is inside cell k
        leftVotes++;
        if (dist(leftOk, leftMM) >= sideWallMM) leftOpenVotes++;
      }
    }
    // ---- Look at the RIGHT side of the next cell while driving ----
    if (!ignoreRight && plannedStop < 0 && rightSeq != lastRSeq) {
      lastRSeq = rightSeq;
      float sp = pos + SIDE_SENSOR_AHEAD_MM;
      if (sp >= nextK * CELL_SIZE_MM - HALF + 20) {       // sensor is inside cell k
        totalVotes++;
        if (dist(rightOk, rightMM) >= sideWallMM) openVotes++;
      }
    }
    if (!ignoreRight && plannedStop < 0 && pos >= nextK * CELL_SIZE_MM - DECIDE_BEFORE_MM) {
      mapNoteDriveLeft(nextK, leftOpenVotes, leftVotes);   // MAZE MAP: hand the left result to the map
      if (totalVotes >= 2 && openVotes * 2 > totalVotes) {
        plannedStop = nextK * CELL_SIZE_MM;
        why = STOP_RIGHT_OPEN;
        setState("driving: cell +%d has a RIGHT OPENING -> stopping at its centre", nextK);
      } else if (totalVotes < 2) {
        plannedStop = nextK * CELL_SIZE_MM;          // not enough readings: stop and look
        why = STOP_UNSURE;
      } else {
        setState("driving: cell +%d right wall, continuing", nextK);
        nextK++;
        openVotes = totalVotes = 0;
        leftOpenVotes = leftVotes = 0;               // MAZE MAP: start counting the next cell's left side
      }
    }

    // ---- Stop conditions ----
    if (frontOk && !fresh(frontLastMs)) { why = STOP_FRONT_STUCK; break; }   // v10: can't see ahead
    if (millis() - startMs > ACCEL_RAMP_MS) {
      // (a) BOTH wheels stopped turning (one flaky encoder can't trigger this)
      if (millis() - blkLMs > BLOCKED_MS && millis() - blkRMs > BLOCKED_MS) {
        lastBlockCause = 1; lastBlockFront = f;          // RECOVERY FIX: remember why (for the log)
        why = STOP_BLOCKED; break;
      }
      // (b) wheels turning but the front wall isn't getting closer = slipping against something
      float encMM = cnt / countsPerMM;
      if (slipCheckMs == 0) { slipCheckMs = millis(); slipEnc0 = encMM; slipF0 = f; slipSeq0 = frontSeq; }
      else if (millis() - slipCheckMs > 500) {
        // RECOVERY FIX: only trust the front sensor when it is close (far readings are too noisy),
        // only with a NEW reading in this window, and only if it happens in 2 windows in a row.
        bool slipNow = f < SLIP_CHECK_MAX_MM && slipF0 < SLIP_CHECK_MAX_MM && frontSeq != slipSeq0 &&
                       encMM - slipEnc0 > 40 && (slipF0 - f) < 8;
        if (slipNow) slipHits++; else slipHits = 0;
        if (slipHits >= SLIP_CONFIRM_WINDOWS) {
          Serial.println("BLOCKED: wheels turning but front distance not changing (slipping on something)");
          lastBlockCause = 2; lastBlockFront = f;        // RECOVERY FIX
          why = STOP_BLOCKED; break;
        }
        slipCheckMs = millis(); slipEnc0 = encMM; slipF0 = f; slipSeq0 = frontSeq;
      }
      // (c) v13: "knocked crooked" check REMOVED - pushing the robot no longer stops it;
      //     the heading PID just steers it back.
    }
    // v12: front wall only after FRONT_CONFIRM_READINGS new "close" readings in a row
    // (a single bad reading used to stop the robot early -> early turn)
    if (frontSeq != lastFrontStopSeq) {
      lastFrontStopSeq = frontSeq;
      if (f <= frontStopMM + FRONT_STOP_LEAD_MM) frontCloseCnt++; else frontCloseCnt = 0;
    }
    if (frontCloseCnt >= FRONT_CONFIRM_READINGS) { why = STOP_FRONT_WALL; break; }
    if (plannedStop >= 0 && pos >= plannedStop - STOP_LEAD_MM) break;
    if (millis() - startMs > DRIVE_TIMEOUT_MS) { why = STOP_TIMEOUT; break; }

    // ---- Wall centering (+ offset = robot is right of centre) ----
    int lm = dist(leftOk, leftMM);
    int rm = dist(rightOk, rightMM);
    bool lw = lm < sideWallMM;
    bool rw = rm < sideWallMM;
    float offset = 0;
    if (lw && rw)  offset = (lm - rm) / 2.0;
    else if (lw)   offset = lm - sideCenterMM;
    else if (rw)   offset = sideCenterMM - rm;
    float targetHeading = constrain(WALL_CENTER_GAIN * offset,
                                    -MAX_WALL_STEER_DEG, MAX_WALL_STEER_DEG);

    // ---- Heading PID ----
    float error = targetHeading - heading;
    bool upToSpeed = (millis() - startMs) > ACCEL_RAMP_MS;

    // Integral: learns the steady left/right motor mismatch. Only while cruising,
    // clamped so it can never add more than I_TERM_MAX PWM, and kept between straights.
    if (Ki > 0.001 && upToSpeed) {
      float lim = I_TERM_MAX / Ki;
      headingIntegral = constrain(headingIntegral + error * dt, -lim, lim);
    }
    float correction = constrain(Kp * error + Ki * headingIntegral - Kd * yawRate,
                                 -MAX_CORRECTION, MAX_CORRECTION);

    liveHeading = heading;
    if (upToSpeed) {
      float a = fabs(heading);
      sumAbsHdg += a; nHdg++;
      if (a > maxAbsHdg) maxAbsHdg = a;
      int sgn = heading > 0.7 ? 1 : (heading < -0.7 ? -1 : 0);   // count side-to-side swings
      if (sgn != 0 && lastSign != 0 && sgn != lastSign) crossings++;
      if (sgn != 0) lastSign = sgn;
    }

    // ---- Speed: slow down approaching a planned stop or a front wall ----
    float scale = 1.0;
    if (plannedStop >= 0) scale = min(scale, constrain((plannedStop - pos) / 70.0f, 0.0f, 1.0f));
    if (f < 9999)         scale = min(scale, constrain((f - frontCenterMM) / 100.0f, 0.0f, 1.0f));
    scale = max(scale, APPROACH_MIN_SCALE);

    // Smooth start: never ask for more than the ramp allows
    float ramp = START_SCALE + (1.0 - START_SCALE) * (millis() - startMs) / (float)ACCEL_RAMP_MS;
    scale = min(scale, constrain(ramp, START_SCALE, 1.0f));

    // Apply the steering. If a wheel would drop below MIN_PWM, raise BOTH wheels
    // instead of clipping one - clipping would throw away the correction.
    float pl = CRUISE_LEFT_PWM  * scale - correction;
    float pr = CRUISE_RIGHT_PWM * scale + correction;
    float lowest = min(pl, pr);
    if (lowest < MIN_PWM) { pl += MIN_PWM - lowest; pr += MIN_PWM - lowest; }
    int leftPWM  = constrain((int)pl, MIN_PWM, MAX_PWM);
    int rightPWM = constrain((int)pr, MIN_PWM, MAX_PWM);
    setMotors(+1, leftPWM, rightPWM);

    if (millis() - lastPrint >= 100) {
      lastPrint = millis();
      Serial.printf("drive pos:%.0fmm k:%d votes:%d/%d stop:%.0f  off:%.0f hdg:%.1f  L:%d R:%d F:%d\n",
                    pos, nextK, openVotes, totalVotes, plannedStop, offset, heading,
                    leftPWM, rightPWM, f);
    }
  }

  brakeAndStop();
  liveHeading = 0;
  if (nHdg > 0) {
    statMaxHdg = maxAbsHdg;
    statAvgHdg = sumAbsHdg / nHdg;
    statCrossings = crossings;
    Serial.printf("Straightness: max %.1f deg, avg %.1f deg, %d wobbles, I-term %.1f PWM\n",
                  maxAbsHdg, sumAbsHdg / nHdg, crossings, Ki * headingIntegral);
  }
  cellsDone = max(0, (int)lroundf(pos / CELL_SIZE_MM));
  Serial.printf("Stopped: %s after %.0f mm (%d cells)\n", reasonName(why), pos, cellsDone);
  return true;
}

// ============================================================
//  Square up to a front wall: creep until the front reading = targetMM
//  (puts the axle at the cell centre, plus any extra turn clearance)
//  Raises the power automatically if the wheels don't start moving.
// ============================================================
void alignToFrontWall(int targetMM) {
  if (!frontOk) return;
  unsigned long start = millis();
  uint32_t lastSeq = frontSeq;
  int inTolCount = 0;
  int pwm = FRONT_ALIGN_PWM;
  long lastEnc = 0;
  { long l, r; readEncoders(l, r); lastEnc = l + r; }
  unsigned long lastMoveMs = millis();
  resetYawTimer();

  while (millis() - start < 2000 && runEnabled) {
    telemetryTick();
    updateYaw();

    // Wheels not turning? push harder (up to 150)
    long l, r;
    readEncoders(l, r);
    if (l + r != lastEnc) { lastEnc = l + r; lastMoveMs = millis(); }
    else if (millis() - lastMoveMs > 120 && pwm < 150) { pwm += 10; lastMoveMs = millis(); }

    if (frontSeq == lastSeq) { delay(3); continue; }   // act only on new readings
    lastSeq = frontSeq;

    int f = dist(frontOk, frontMM);
    if (f >= frontWallMM) break;
    int err = f - targetMM;
    if (abs(err) <= FRONT_ALIGN_TOL_MM) {
      brakeMotors();
      if (++inTolCount >= 2) break;     // two good readings in a row
      continue;
    }
    inTolCount = 0;
    int pr = pwm * FORWARD_RIGHT_SPEED / FORWARD_LEFT_SPEED;
    if (err > 0) setMotors(+1, pwm, pr);   // too far: creep forward
    else         setMotors(-1, pwm, pr);   // too close: creep back
  }
  brakeAndStop();
  Serial.printf("Front align: target %d mm, now %d mm\n", targetMM, dist(frontOk, frontMM));
}

// Back straight up a short distance (used when a turn gets blocked)
void backUp(int mm) {
  long l0, r0, l, r;
  readEncoders(l0, r0);
  long need = lroundf(mm * countsPerMM);
  int pwm = FRONT_ALIGN_PWM;
  unsigned long start = millis();
  while (millis() - start < 800 && runEnabled) {
    readEncoders(l, r);
    if (((l - l0) + (r - r0)) / 2 >= need) break;
    setMotors(-1, pwm, pwm * FORWARD_RIGHT_SPEED / FORWARD_LEFT_SPEED);
    telemetryTick();
    delay(3);
  }
  brakeAndStop();
}

// ============================================================
//  Wall sensing while stopped (majority of 5 fresh readings)
// ============================================================
void senseWalls(bool &wl, bool &wf, bool &wr) {
  // v10: make sure every sensor is really updating before trusting it
  // (the watchdog restarts stuck ones while we wait)
  if (!waitForFreshSensors(1500)) Serial.println("senseWalls: a sensor is still STUCK");

  int vL = 0, vF = 0, vR = 0;
  for (int i = 0; i < 5; i++) {
    waitWithTelemetry(40);
    if (dist(leftOk,  leftMM)  < sideWallMM)    vL++;
    if (dist(frontOk, frontMM) < frontWallMM) vF++;
    if (dist(rightOk, rightMM) < sideWallMM)    vR++;
  }
  wl = vL >= 3;
  wf = vF >= 3;
  wr = vR >= 3;

  // A sensor that is STILL stuck is treated as a WALL: never drive or turn
  // somewhere we can't see.
  if (frontOk && !fresh(frontLastMs)) wf = true;
  if (leftOk  && !fresh(leftLastMs))  wl = true;
  if (rightOk && !fresh(rightLastMs)) wr = true;
}

Action decide(bool wl, bool wf, bool wr) {
  if (!wr) return TURN_RIGHT;      // right-hand rule: right > straight > left > back
  if (!wf) return GO_FORWARD;
  if (!wl) return TURN_LEFT;
  return TURN_AROUND;
}

// ============================================================
//  v12 GYRO TURN IN PLACE - speed-profiled, PID on the rotation speed
//  1) spin up gently (TURN_ACCEL), 2) cruise at TURN_MAX_RATE, 3) slow down along
//     v = sqrt(vmin^2 + 2*decel*remaining) so it ARRIVES at only TURN_MIN_RATE,
//  4) brake inside TURN_TOLERANCE and wait until really still.
//  If it still overshoots, the error changes sign and it creeps back slowly.
//  Both wheels always driven in opposite directions, kept in step by the encoders.
// ============================================================
// Returns false if the turn got BLOCKED (robot stopped rotating before the target)
bool turnToHeading(float target) {
  unsigned long startTime    = millis();
  unsigned long settledSince = 0;
  unsigned long lastUs       = micros();
  unsigned long lastPrint    = 0;
  bool stalled = false;

  long l0, r0;
  readEncoders(l0, r0);
  resetYawTimer();

  float progressYaw = yaw;               // stall detection
  unsigned long progressMs = millis();
  float rateInt = 0;
  int   lastDir = 0;
  const int startDir = (target - yaw) >= 0 ? 1 : -1;
  float maxOvershoot = 0;

  while (millis() - startTime < TURN_TIMEOUT_MS) {
    if (!runEnabled) break;
    telemetryTick();

    unsigned long nowUs = micros();
    if (nowUs - lastUs < CONTROL_PERIOD_US) continue;
    float dt = (nowUs - lastUs) / 1e6;
    lastUs = nowUs;

    updateYaw();
    float error = target - yaw;          // + = still need to turn left (CCW)
    float rem   = fabs(error);
    if (error * startDir < 0 && rem > maxOvershoot) maxOvershoot = rem;   // went past the target

    // ---- arrived: brake and wait until it's really still ----
    if (rem <= TURN_TOLERANCE) {
      brakeMotors();
      rateInt = 0;
      if (fabs(yawRate) < SETTLE_RATE) {
        if (settledSince == 0) settledSince = millis();
        if (millis() - settledSince >= TURN_SETTLE_MS) break;
      } else {
        settledSince = 0;
      }
      continue;
    }
    settledSince = 0;

    // ---- blocked by a wall? ----
    if (fabs(yaw - progressYaw) > 2.0) { progressYaw = yaw; progressMs = millis(); }
    else if (millis() - progressMs > 500 && rem > 5.0) { stalled = true; break; }

    int dir = error > 0 ? 1 : -1;
    if (dir != lastDir) { rateInt = 0; lastDir = dir; }

    // ---- desired rotation speed (deg/s) from the profile ----
    float t = (millis() - startTime) / 1000.0;
    float desired = sqrtf(TURN_MIN_RATE * TURN_MIN_RATE + 2.0f * TURN_DECEL * rem);
    desired = min(desired, TURN_MAX_RATE);
    desired = min(desired, TURN_MIN_RATE + TURN_ACCEL * t);          // gentle spin-up

    // ---- PID on rotation speed ----
    float actual  = yawRate * dir;                  // + = rotating toward the target
    float rateErr = desired - actual;
    float iLim = TURN_I_MAX / TURN_RATE_KI;
    rateInt = constrain(rateInt + rateErr * dt, -iLim, iLim);

    if (actual > desired + 40) { brakeMotors(); continue; }          // much too fast -> brake now

    float u = TURN_START_PWM + TURN_FF_PER_DPS * desired + TURN_RATE_KP * rateErr + TURN_RATE_KI * rateInt;
    if (u < TURN_START_PWM * 0.6) { stopMotors(); continue; }       // a bit too fast -> coast

    int base = constrain((int)u, 0, MAX_TURN_PWM);

    // Keep both wheels turning the same amount -> robot spins about its axle centre
    long l, r;
    readEncoders(l, r);
    long dl = l - l0, dr = r - r0;
    float sync = encodersAgree(dl, dr)
                 ? constrain(TURN_SYNC_GAIN * (dl - dr), -TURN_SYNC_MAX, TURN_SYNC_MAX) : 0;
    int pl = constrain((int)(base * TURN_TRIM_LEFT  - sync), TURN_WHEEL_MIN_PWM, 255);
    int pr = constrain((int)(base * TURN_TRIM_RIGHT + sync), TURN_WHEEL_MIN_PWM, 255);

    spinWheels(dir < 0, pl, pr);

    if (millis() - lastPrint >= 50) {
      lastPrint = millis();
      Serial.printf("turn err:%.1f want:%.0f got:%.0f deg/s  pwm L:%d R:%d\n",
                    error, desired * dir, yawRate, pl, pr);
    }
  }

  brakeAndStop();
  // let it come fully to rest, then record the result
  unsigned long w = millis();
  resetYawTimer();
  while (millis() - w < 150) { updateYaw(); telemetryTick(); delay(5); }
  lastTurnOvershoot = maxOvershoot;
  lastTurnError     = yaw - target;
  long l, r;
  readEncoders(l, r);
  Serial.printf("Turn %s. end error %+.2f deg, overshoot %.2f deg, wheel counts L:%ld R:%ld\n",
                stalled ? "BLOCKED" : "done", yaw - target, maxOvershoot, l - l0, r - r0);
  return !stalled;
}

// v13: turn, then CHECK the result after the robot has fully stopped (yaw keeps being
// tracked, so a late slide is seen) and make correction turns until it's within
// TURN_VERIFY_TOL. Reports the first-pass error: + = overshoot, - = undershoot.
bool turnVerified(float target) {
  float startYaw = yaw;
  int dir = (target - startYaw) >= 0 ? 1 : -1;       // +1 = left turn
  bool ok = turnToHeading(target);
  float firstOvershoot = lastTurnOvershoot;
  int fixes = 0;
  float firstErr = 0;
  for (int k = 0; ok && runEnabled; k++) {
    waitWithTelemetry(TURN_VERIFY_MS);                // let any slide finish, keep measuring
    float err = (yaw - target) * dir;                 // + = went too far, - = not far enough
    if (k == 0) firstErr = err;
    if (fabs(err) <= TURN_VERIFY_TOL) break;
    if (k >= TURN_MAX_FIXES) {
      Serial.printf("TURN CHECK: still %+.2f deg after %d fixes - leaving it for the walls\n", err, fixes);
      break;
    }
    fixes++;
    Serial.printf("TURN CHECK: %s by %.2f deg -> correction %d\n",
                  err > 0 ? "OVERSHOOT" : "UNDERSHOOT", fabs(err), fixes);
    setState("turn check: %s %.1f deg -> fixing (%d)", err > 0 ? "overshoot" : "undershoot", fabs(err), fixes);
    ok = turnToHeading(target);
  }
  lastTurnOvershoot  = firstOvershoot;
  lastTurnFirstError = firstErr;
  lastTurnFixes      = fixes;
  lastTurnError      = (yaw - target) * dir;
  Serial.printf("TURN RESULT: first pass %+.2f deg, %d fix(es), final %+.2f deg (+ over, - under)\n",
                firstErr, fixes, (float)lastTurnError);
  return ok;
}

// Drive straight forward a short distance (used by the recovery manoeuvres)
void forwardBy(int mm) {
  long l0, r0, l, r;
  readEncoders(l0, r0);
  long need = lroundf(mm * countsPerMM);
  int pwm = FRONT_ALIGN_PWM;
  unsigned long start = millis();
  while (millis() - start < 1000 && runEnabled) {
    readEncoders(l, r);
    if (((l - l0) + (r - r0)) / 2 >= need) break;
    if (dist(frontOk, frontMM) < 45) break;        // never drive into a wall doing this
    setMotors(+1, pwm, pwm * FORWARD_RIGHT_SPEED / FORWARD_LEFT_SPEED);
    telemetryTick();
    delay(3);
  }
  brakeAndStop();
}

// v12: if the robot sits too close to a side wall, slide away from it:
// turn 25 deg away, drive forward, turn back, back up the same forward distance.
void shiftAwayFromCloseWall() {
  waitForFreshSensors(800);
  waitWithTelemetry(120);
  int lm = dist(leftOk, leftMM), rm = dist(rightOk, rightMM);
  bool lw = lm < sideWallMM, rw = rm < sideWallMM;
  float off = 0;                                  // + = robot is right of centre
  if (lw && rw)  off = (lm - rm) / 2.0;
  else if (lw)   off = lm - sideCenterMM;
  else if (rw)   off = sideCenterMM - rm;
  if (fabs(off) < 15) return;                     // close enough to the middle

  const float ang = 25.0;
  int d = constrain((int)(fabs(off) / sinf(ang * PI / 180.0)), 20, 70);
  float base = pathHeading;
  learnArmed = false; headingKnown = false;        // v13: don't learn scale from this
  setState("RECOVERY: %d mm off-centre - sliding %s", (int)fabs(off), off > 0 ? "left" : "right");
  turnToHeading(base + (off > 0 ? ang : -ang));  // off > 0 -> need to move left -> turn left
  forwardBy(d);
  turnVerified(base);                              // v13: checked + fixed
  backUp((int)(d * cosf(ang * PI / 180.0)));      // undo the forward part
}

// RECOVERY FIX: median of 3 NEW front readings (9999 = no valid reading)
int recoveryFrontMedian() {
  int v[3];
  int n = 0;
  uint32_t seq = frontSeq;
  unsigned long start = millis();
  while (n < 3 && millis() - start < 400) {
    telemetryTick();
    if (frontSeq != seq) { seq = frontSeq; v[n++] = dist(frontOk, frontMM); }
    else delay(3);
  }
  if (n == 0) return dist(frontOk, frontMM);
  for (int i = 0; i < n; i++)
    for (int j = i + 1; j < n; j++)
      if (v[j] < v[i]) { int t = v[i]; v[i] = v[j]; v[j] = t; }
  return v[n / 2];
}

// v12: called when a drive stops BLOCKED. Escalates each time it happens at the same place.
void recoverFromBlock() {
  if (cellsMoved == blockedAtCell) blockedStreak++; else blockedStreak = 1;
  blockedAtCell = cellsMoved;

  int back = min(40 + 20 * (blockedStreak - 1), 80);
  char frontTxt[16];                                         // RECOVERY FIX: say WHY in the log
  if (lastBlockFront >= 9999) snprintf(frontTxt, sizeof(frontTxt), "none");
  else snprintf(frontTxt, sizeof(frontTxt), "%d mm", lastBlockFront);
  setState("RECOVERY %d: %s, front %s - backing up %d mm and looking around", blockedStreak,
           lastBlockCause == 2 ? "slipping" : "wheels stopped", frontTxt, back);
  backUp(back);
  waitWithTelemetry(200);

  learnArmed = false; headingKnown = false;   // v13: don't learn scale across a block
  turnVerified(pathHeading);           // straighten up (v13: checked + fixed)
  shiftAwayFromCloseWall();            // get off a side wall / corner

  // Something ahead the front sensor can't see (a thin wall end, a post)? After the 2nd
  // block at the same place, stop trying to go straight - treat it as a wall.
  if (blockedStreak >= 2) forceFrontBlocked = true;

  waitForFreshSensors(800);
  // RECOVERY FIX: a repeated block only becomes a MAP wall if the front sensor actually sees one.
  // Open front (e.g. 568 mm) = the robot is snagged / wheels didn't start, NOT a wall ahead.
  int fNow = dist(frontOk, frontMM);
  if (forceFrontBlocked) {
    fNow = recoveryFrontMedian();
    forceFrontConfirmed = (fNow < RECOVERY_WALL_CONFIRM_MM);
    Serial.printf("RECOVERY: front after backing up = %d mm -> %s\n", fNow,
                  forceFrontConfirmed ? "wall confirmed" : "open - not a wall, will not be mapped");
  }
  setState("RECOVERY %d done: L:%d F:%d R:%d%s", blockedStreak,
           dist(leftOk, leftMM), fNow, dist(rightOk, rightMM),
           !forceFrontBlocked ? "" :
           (forceFrontConfirmed ? " - treating FRONT as blocked (wall seen)"
                                : " - front OPEN: avoid once, NOT mapped as wall"));
}

void doTurn(Action a) {
  if (a == TURN_RIGHT)       pathHeading -= 90.0;
  else if (a == TURN_LEFT)   pathHeading += 90.0;
  else if (a == TURN_AROUND) pathHeading -= 180.0;
  else return;

  float delta = (a == TURN_RIGHT) ? -90.0 : (a == TURN_LEFT) ? 90.0 : -180.0;
  bool learnable = headingKnown;       // v13: only if the heading was known before this turn
  learnArmed = false;
  for (int attempt = 1; attempt <= 4 && runEnabled; attempt++) {
    if (turnVerified(pathHeading)) {   // v13: checked + fixed
      learnArmed = learnable && attempt == 1;
      learnDelta = delta;
      headingKnown = false;            // until a wall confirms it
      return;
    }
    learnable = false;
    // blocked while turning: back up further each time and get away from the walls
    int back = BACKUP_MM * attempt;
    setState("turn blocked (try %d) - backing up %d mm, re-centring, retrying", attempt, back);
    backUp(back);
    waitWithTelemetry(200);
    if (attempt >= 2) shiftAwayFromCloseWall();
  }
  headingKnown = false;
  setState("turn still blocked after 4 tries - continuing from here");
}

// Stand still for ms and re-measure the gyro bias (removes slow heading drift).
// Only accepted if the wheels really didn't move and the new value is sensible.
void stillPauseAndRefineGyro(unsigned long ms) {
  long l0, r0, l, r;
  readEncoders(l0, r0);
  unsigned long start = millis();
  float sum = 0, sum2 = 0;
  int n = 0;
  while (millis() - start < ms) {
    telemetryTick();
    if (millis() - start > 300) { float g = readGyroZRaw(); sum += g; sum2 += g * g; n++; }
    delay(4);
  }
  readEncoders(l, r);
  if (n > 50 && l == l0 && r == r0) {
    float b = sum / n;
    float sd = sqrtf(max(0.0f, sum2 / n - b * b));
    // v13: only if it was REALLY still - a slow slide would otherwise be learned as "bias"
    // and make the next turns wrong
    if (fabs(b - gyroBiasZ) < 0.4 && sd < 0.6) gyroBiasZ = 0.7 * gyroBiasZ + 0.3 * b;
    else Serial.printf("Gyro bias NOT updated (change %.2f, noise %.2f deg/s - robot moving?)\n",
                       b - gyroBiasZ, sd);
  }
}

// ============================================================
//  Side-centre calibration at GO
//  - collects CAL_SAMPLES *fresh* readings per side (checks the sequence
//    counters, so a stale value is never counted twice)
//  - trimmed mean per side (drops the lowest/highest 20%) so a spike can't skew it
//  - ONE shared centre = (LEFT + RIGHT) / 2. This is the same whether the robot
//    was placed exactly in the middle or a bit to one side, so the robot always
//    centres on the REAL middle (the sensors' own errors are already removed by
//    the *_TOF_OFFSET_MM values).
//  - reports how spread out the readings were; a big spread = robot moved / noisy
// ============================================================
const int CAL_SAMPLES       = 15;
const int CAL_MAX_SPREAD_MM = 8;

// Sorts v[0..n-1] in place and returns the mean of the middle 60%
int trimmedMean(int *v, int n) {
  for (int i = 1; i < n; i++) {
    int key = v[i], j = i - 1;
    while (j >= 0 && v[j] > key) { v[j + 1] = v[j]; j--; }
    v[j + 1] = key;
  }
  int drop = n / 5;
  long sum = 0;
  int cnt = 0;
  for (int i = drop; i < n - drop; i++) { sum += v[i]; cnt++; }
  return cnt > 0 ? (int)(sum / cnt) : v[n / 2];
}

void autoCalibrateSideCenter() {
  int sampL[CAL_SAMPLES], sampR[CAL_SAMPLES];
  int nL = 0, nR = 0;
  uint32_t lastL = leftSeq, lastR = rightSeq;
  unsigned long start = millis();

  // Poll until both sides have CAL_SAMPLES new readings (max 2 s)
  while ((nL < CAL_SAMPLES || nR < CAL_SAMPLES) && millis() - start < 2000) {
    telemetryTick();
    if (leftSeq != lastL) {
      lastL = leftSeq;
      int lm = dist(leftOk, leftMM);
      if (lm < 180 && nL < CAL_SAMPLES) sampL[nL++] = lm;
    }
    if (rightSeq != lastR) {
      lastR = rightSeq;
      int rm = dist(rightOk, rightMM);
      if (rm < 180 && nR < CAL_SAMPLES) sampR[nR++] = rm;
    }
    delay(2);
  }

  if (nL >= CAL_SAMPLES / 2 && nR >= CAL_SAMPLES / 2) {
    int loL = sampL[0], hiL = sampL[0], loR = sampR[0], hiR = sampR[0];
    for (int i = 1; i < nL; i++) { loL = min(loL, sampL[i]); hiL = max(hiL, sampL[i]); }
    for (int i = 1; i < nR; i++) { loR = min(loR, sampR[i]); hiR = max(hiR, sampR[i]); }
    int avgL = trimmedMean(sampL, nL);
    int avgR = trimmedMean(sampR, nR);

    sideCenterMM = (avgL + avgR) / 2;
    sideWallMM   = sideCenterMM + CELL_SIZE_MM / 2;

    bool noisy = (hiL - loL) > CAL_MAX_SPREAD_MM || (hiR - loR) > CAL_MAX_SPREAD_MM;
    Serial.printf("Side centre = %d mm (L %d, spread %d | R %d, spread %d)%s\n",
                  sideCenterMM, avgL, hiL - loL, avgR, hiR - loR,
                  noisy ? "  <-- NOISY: robot moving or sensor glitch?" : "");
    Serial.printf("Robot placed %d mm %s of the corridor centre (auto-corrects while driving)\n",
                  abs(avgL - avgR) / 2, avgL > avgR ? "right" : "left");
    setState("calibrated: centre %d mm (L %d / R %d)%s", sideCenterMM, avgL, avgR,
             noisy ? " NOISY!" : "");
  } else {
    Serial.printf("No walls on both sides at start (L %d, R %d readings) - using side centre %d mm\n",
                  nL, nR, sideCenterMM);
  }
}

// Why did the ESP32 last (re)start? Shown on the page at IDLE.
const char* resetReason() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "power on";
    case ESP_RST_BROWNOUT: return "BROWNOUT - battery/voltage dipped";
    case ESP_RST_PANIC:    return "CRASH";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      return "WATCHDOG";
    case ESP_RST_SW:       return "software";
    default:               return "other";
  }
}

// ============================================================
// ============================================================
//  MAZE MAP - passive mapping layer (observes only, never changes movement)
//
//  Coordinates : row 0..7 (0 = north / top of the dashboard), col 0..7 (0 = west / left)
//  Start cell  : bottom-left (7,0) by default, or bottom-right (7,7) (dashboard selector).
//                The robot must be placed FACING NORTH = the start cell's only exit.
//  Each cell   : known bits + wall bits (bit 0 N, bit 1 E, bit 2 S, bit 3 W)
//                known=0 -> UNKNOWN, known=1 & wall=0 -> OPEN, known=1 & wall=1 -> WALL
//                locked bits: edges that can never change again (outer perimeter = WALL,
//                edges the robot physically drove through = OPEN)
//  Every edge is always written on BOTH cells that share it, so they can never disagree.
// ============================================================
const int MAZE_ROWS = 8, MAZE_COLS = 8;
struct MazeCell {
  uint8_t known;      // bit set = this edge is known
  uint8_t walls;      // bit set = wall (only meaningful when the known bit is set)
  uint8_t locked;     // bit set = edge can no longer change (perimeter / driven through)
  bool    visited;    // the robot has been in this cell
};
MazeCell maze[MAZE_ROWS][MAZE_COLS];

int       robotRow = 7, robotCol = 0;     // current cell
Direction robotDir = NORTH;               // current facing
int  mapStartCorner     = 0;              // 0 = bottom-left (7,0), 1 = bottom-right (7,7)
long mapTransitions     = 0;              // real one-cell boundary crossings this run
int  mapBlockedEvents   = 0;              // drives that ended BLOCKED (position less certain)
int  mapWarnings        = 0;              // conflicts / attempts to leave the maze
bool mapRunPending      = false;          // set at GO; map is reset when the maze run starts
float mapBaseHeading    = 0;              // pathHeading that corresponds to NORTH
long  mapEncL0 = 0, mapEncR0 = 0;         // encoder counts at the start of a drive
char  mapLastEvent[100] = "map ready";
// MAZE MAP: left-side results noted by driveUntilEvent() for cell +k of the current drive
// (-1 = not decided, 0 = open, 1 = wall). Applied when the map actually moves into that cell.
const int MAP_DRIVE_MAX_K = 16;
int8_t mapDriveLeft[MAP_DRIVE_MAX_K];
void mapNoteDriveLeft(int k, int openVotes, int votes) {
  if (k < 1 || k >= MAP_DRIVE_MAX_K) return;
  if (votes < 2) { mapDriveLeft[k] = -1; return; }           // same rule as the right side: need 2+ readings
  mapDriveLeft[k] = (openVotes * 2 > votes) ? 0 : 1;          // majority open -> open, otherwise wall
}


const char* dirName(Direction d) {
  switch (d) { case NORTH: return "NORTH"; case EAST: return "EAST"; case SOUTH: return "SOUTH"; default: return "WEST"; }
}
Direction oppositeDirection(Direction d) { return (Direction)((d + 2) % 4); }
// robot-relative -> absolute: rel 0 = front, 1 = right, 2 = back, 3 = left
Direction relativeToAbsolute(Direction facing, int rel) { return (Direction)((facing + rel) % 4); }
bool validMazeCell(int row, int col) { return row >= 0 && row < MAZE_ROWS && col >= 0 && col < MAZE_COLS; }
void neighbourCell(int row, int col, Direction d, int &nr, int &nc) {
  nr = row + (d == SOUTH ? 1 : d == NORTH ? -1 : 0);
  nc = col + (d == EAST ? 1 : d == WEST ? -1 : 0);
}
// 0 = UNKNOWN, 1 = OPEN, 2 = WALL (for logs and later flood fill)
int edgeState(int row, int col, Direction d) {
  if (!validMazeCell(row, col)) return 2;
  uint8_t b = 1 << d;
  if (!(maze[row][col].known & b)) return 0;
  return (maze[row][col].walls & b) ? 2 : 1;
}
const char* stateName(int st) { return st == 2 ? "WALL" : st == 1 ? "OPEN" : "UNKNOWN"; }

// Write ONE physical edge on both cells. weakOnly = only fill it if still unknown.
void setMazeEdge(int row, int col, Direction d, bool wall, bool lockIt, bool weakOnly) {
  if (!validMazeCell(row, col)) return;
  int nr, nc;
  neighbourCell(row, col, d, nr, nc);
  uint8_t b = 1 << d, ob = 1 << oppositeDirection(d);
  MazeCell &a = maze[row][col];
  if (!validMazeCell(nr, nc)) {                     // outer perimeter: always WALL, never changed
    if (!wall) {
      mapWarnings++;
      Serial.printf("MAP WARNING: reading says OPEN through the outer wall at (%d,%d) %s - ignored\n",
                    row, col, dirName(d));
    }
    return;
  }
  MazeCell &n = maze[nr][nc];
  bool known = a.known & b, isWall = a.walls & b;
  if (weakOnly && known) return;
  if (a.locked & b) {                               // driven through (or fixed) - cannot change
    if (known && isWall != wall) {
      mapWarnings++;
      Serial.printf("MAP WARNING: reading says %s at (%d,%d) %s but that edge is fixed as %s - kept\n",
                    wall ? "WALL" : "OPEN", row, col, dirName(d), isWall ? "WALL" : "OPEN");
    }
    return;
  }
  a.known |= b;  n.known |= ob;
  if (wall) { a.walls |= b;  n.walls |= ob; }
  else      { a.walls &= ~b; n.walls &= ~ob; }
  if (lockIt) { a.locked |= b; n.locked |= ob; }
}

void logCellWalls(const char *prefix) {
  Serial.printf("MAP: %s (%d,%d): N=%s E=%s S=%s W=%s\n", prefix, robotRow, robotCol,
                stateName(edgeState(robotRow, robotCol, NORTH)), stateName(edgeState(robotRow, robotCol, EAST)),
                stateName(edgeState(robotRow, robotCol, SOUTH)), stateName(edgeState(robotRow, robotCol, WEST)));
}

void resetMazeMap() {
  for (int r = 0; r < MAZE_ROWS; r++)
    for (int c = 0; c < MAZE_COLS; c++) {
      maze[r][c].known = maze[r][c].walls = maze[r][c].locked = 0;
      maze[r][c].visited = false;
    }
  // outer perimeter = WALL, locked
  for (int r = 0; r < MAZE_ROWS; r++)
    for (int c = 0; c < MAZE_COLS; c++) {
      uint8_t per = 0;
      if (r == 0)             per |= 1 << NORTH;
      if (r == MAZE_ROWS - 1) per |= 1 << SOUTH;
      if (c == 0)             per |= 1 << WEST;
      if (c == MAZE_COLS - 1) per |= 1 << EAST;
      maze[r][c].known |= per; maze[r][c].walls |= per; maze[r][c].locked |= per;
    }
  robotRow = MAZE_ROWS - 1;
  robotCol = (mapStartCorner == 1) ? MAZE_COLS - 1 : 0;
  robotDir = NORTH;
  mapBaseHeading = pathHeading;
  mapTransitions = 0; mapBlockedEvents = 0; mapWarnings = 0;
  maze[robotRow][robotCol].visited = true;
  // start cell = three walls, the only exit is NORTH (the way the robot faces)
  setMazeEdge(robotRow, robotCol, NORTH, false, false, false);
  setMazeEdge(robotRow, robotCol, mapStartCorner == 1 ? WEST : EAST, true, false, false);
  snprintf(mapLastEvent, sizeof(mapLastEvent), "map reset - start (%d,%d) facing NORTH", robotRow, robotCol);
  Serial.printf("MAP: start at (%d,%d), facing NORTH\n", robotRow, robotCol);
  logCellWalls("start cell");
  computeFloodFill();                            // FLOOD FILL: initial flood map (perimeter + start cell only)
}

// Walls from the EXISTING senseWalls() result (wl/wf/wr) -> current cell. No back wall (no sensor).
void markCurrentCellWalls(bool wl, bool wf, bool wr) {
  setMazeEdge(robotRow, robotCol, relativeToAbsolute(robotDir, 3), wl, false, false);   // left
  setMazeEdge(robotRow, robotCol, relativeToAbsolute(robotDir, 0), wf, false, false);   // front
  setMazeEdge(robotRow, robotCol, relativeToAbsolute(robotDir, 1), wr, false, false);   // right
  maze[robotRow][robotCol].visited = true;
  snprintf(mapLastEvent, sizeof(mapLastEvent), "sensed (%d,%d) facing %s: L%c F%c R%c",
           robotRow, robotCol, dirName(robotDir), wl ? 'W' : 'o', wf ? 'W' : 'o', wr ? 'W' : 'o');
  logCellWalls("sensed cell");
  computeFloodFill();                            // FLOOD FILL: walls changed -> reflow (all 64 cells)
}

// One real boundary crossing straight ahead. Returns false (and stays put) if it would leave the maze.
bool moveMapForwardOneCell() {
  int nr, nc;
  neighbourCell(robotRow, robotCol, robotDir, nr, nc);
  if (!validMazeCell(nr, nc)) {
    mapWarnings++;
    snprintf(mapLastEvent, sizeof(mapLastEvent), "WARNING: would leave the maze from (%d,%d) going %s - kept",
             robotRow, robotCol, dirName(robotDir));
    Serial.printf("MAP WARNING: crossing %s from (%d,%d) would leave the 8x8 maze - position kept\n",
                  dirName(robotDir), robotRow, robotCol);
    return false;
  }
  if (edgeState(robotRow, robotCol, robotDir) == 2 && !(maze[robotRow][robotCol].locked & (1 << robotDir))) {
    mapWarnings++;
    Serial.printf("MAP WARNING: drove through (%d,%d) %s which a reading had marked WALL - now OPEN\n",
                  robotRow, robotCol, dirName(robotDir));
  }
  setMazeEdge(robotRow, robotCol, robotDir, false, true, false);   // old cell forward = new cell back = OPEN (locked)
  Serial.printf("MAP: crossed %s: (%d,%d) -> (%d,%d)\n", dirName(robotDir), robotRow, robotCol, nr, nc);
  robotRow = nr; robotCol = nc;
  maze[robotRow][robotCol].visited = true;
  mapTransitions++;
  return true;
}

// Advance n cells one at a time; passedRightWall = the existing drive logic only drives
// THROUGH a cell after deciding its right side is a wall, so those cells get a right WALL
// (only filled if still unknown).
int advanceMapCells(int n, bool passedRightWall) {
  int done = 0;
  for (int i = 0; i < n; i++) {
    if (!moveMapForwardOneCell()) break;
    done++;
    if (passedRightWall && i < n - 1)
      setMazeEdge(robotRow, robotCol, relativeToAbsolute(robotDir, 1), true, false, true);
    // left side measured while driving through this cell (only fills edges that are still unknown)
    if (i + 1 < MAP_DRIVE_MAX_K && mapDriveLeft[i + 1] >= 0)
      setMazeEdge(robotRow, robotCol, relativeToAbsolute(robotDir, 3), mapDriveLeft[i + 1] == 1, false, true);
  }
  return done;
}

// Direction = the heading reference the existing turn code set (what the robot drives along next).
void updateMapDirectionAfterTurn() {
  int k = (int)lroundf(-(pathHeading - mapBaseHeading) / 90.0f);   // right turn = pathHeading -90 = +1
  Direction nd = (Direction)(((k % 4) + 4) % 4);
  if (nd != robotDir) {
    int diff = (nd - robotDir + 4) % 4;
    const char *t = diff == 1 ? "RIGHT turn" : diff == 3 ? "LEFT turn" : "TURN AROUND";
    Serial.printf("MAP: %s, direction %s -> %s\n", t, dirName(robotDir), dirName(nd));
    snprintf(mapLastEvent, sizeof(mapLastEvent), "%s: %s -> %s at (%d,%d)", t, dirName(robotDir), dirName(nd),
             robotRow, robotCol);
    robotDir = nd;
  }
}

void mapBeforeDrive() {
  for (int i = 0; i < MAP_DRIVE_MAX_K; i++) mapDriveLeft[i] = -1;   // MAZE MAP: nothing noted yet
  readEncoders(mapEncL0, mapEncR0);
}

// After the EXISTING driveUntilEvent() returned 'cells' and 'why': move the map conservatively.
void mapAfterDrive(int cells, StopReason why) {
  int n = cells;
  bool normal = (why == STOP_FRONT_WALL || why == STOP_RIGHT_OPEN || why == STOP_UNSURE ||
                 why == STOP_FLOOD_CELL);   // FLOOD FILL: a completed one-cell drive is a normal stop
  if (why == STOP_BLOCKED) {
    // A BLOCKED drive: count only the cell the robot will really be in after the recovery
    // backs up (encoder distance minus the recovery's back-up distance).
    long l, r;
    readEncoders(l, r);
    long dL = l - mapEncL0, dR = r - mapEncR0;
    float cnt = encodersAgree(dL, dR) ? (dL + dR) / 2.0f : (float)min(dL, dR);
    float mm = cnt / countsPerMM;
    // recoverFromBlock() (called right after this, unchanged) will back up by exactly this much -
    // read the same state it uses, without changing it, to know where the robot will end up
    int nextStreak = (cellsMoved == blockedAtCell) ? blockedStreak + 1 : 1;
    int back = min(40 + 20 * (nextStreak - 1), 80);
    int safe = (int)lroundf((mm - back) / CELL_SIZE_MM);
    n = constrain(safe, 0, cells);
    mapBlockedEvents++;
    Serial.printf("MAP: drive ended BLOCKED after ~%.0f mm, recovery will back up %d mm (drive counted %d) -> map counts %d cell(s)\n",
                  mm, back, cells, n);
  } else if (!normal) {
    Serial.printf("MAP: drive ended with '%s' - map counts %d cell(s)\n", reasonName(why), cells);
  }
  // FLOOD FILL: "passed cells have a right wall" is only true for the right-wall corridor drive
  int done = advanceMapCells(n, normal && navigationMode == NAV_RIGHT_WALL);
  if (done > 0 || why == STOP_BLOCKED)
    snprintf(mapLastEvent, sizeof(mapLastEvent), "drove %d cell(s) %s -> (%d,%d) [%s]", done, dirName(robotDir),
             robotRow, robotCol, reasonName(why));
  Serial.printf("MAP: now at (%d,%d), facing %s, transitions %ld\n", robotRow, robotCol, dirName(robotDir),
                mapTransitions);
  computeFloodFill();                            // FLOOD FILL: keep dashboard flood values current
}

// ============================================================
// FLOOD FILL NAVIGATION
// Port of FloodfillMicromouse.cpp (simulator) onto the robot's EXISTING map:
//   - goal = the 4 centre cells (3,3) (3,4) (4,3) (4,4), value 0
//   - reverse BFS from the goals; only a KNOWN WALL (edgeState == 2) blocks;
//     UNKNOWN and OPEN edges are both traversable (simulator: discoveredWalls_ holds walls only)
//   - move to the neighbour with the strictly lowest value, directions tried N, E, S, W
//   - no strictly lower neighbour -> reflow once; current cell unreachable -> stop (no path)
//   - one physical cell per decision; walls are re-sensed at every cell
// It never moves the robot itself and never changes robotRow/robotCol/robotDir.
// ============================================================
const int FLOOD_INF = 9999;
int  floodValues[MAZE_ROWS][MAZE_COLS];
int  floodTargetDir = -1;            // last chosen absolute direction (-1 = none) - dashboard
char floodStatus[80] = "idle";
struct FloodNode { uint8_t row; uint8_t col; };

bool isFloodGoalCell(int row, int col) {
  return (row == 3 || row == 4) && (col == 3 || col == 4);
}

// Simulator rule: an edge blocks only if it is a discovered (known) wall
bool floodEdgeTraversable(int row, int col, Direction dir) {
  if (edgeState(row, col, dir) == 2) return false;
  int nr, nc;
  neighbourCell(row, col, dir, nr, nc);
  return validMazeCell(nr, nc);
}

void resetFloodValues() {
  for (int r = 0; r < MAZE_ROWS; r++)
    for (int c = 0; c < MAZE_COLS; c++) floodValues[r][c] = FLOOD_INF;
}

// Reverse breadth-first search seeded at all four goal cells (FloodfillMicromouse::floodFill)
void computeFloodFill() {
  static FloodNode q[MAZE_ROWS * MAZE_COLS * 4];
  int head = 0, tail = 0;
  resetFloodValues();
  for (int r = 0; r < MAZE_ROWS; r++)
    for (int c = 0; c < MAZE_COLS; c++)
      if (isFloodGoalCell(r, c)) { floodValues[r][c] = 0; q[tail].row = r; q[tail].col = c; tail++; }
  while (head < tail) {
    int r = q[head].row, c = q[head].col; head++;
    for (int d = 0; d < 4; d++) {
      if (!floodEdgeTraversable(r, c, (Direction)d)) continue;   // known wall or outside the maze
      int nr, nc;
      neighbourCell(r, c, (Direction)d, nr, nc);
      if (floodValues[nr][nc] > floodValues[r][c] + 1) {
        floodValues[nr][nc] = floodValues[r][c] + 1;
        if (tail < (int)(sizeof(q) / sizeof(q[0]))) { q[tail].row = nr; q[tail].col = nc; tail++; }
      }
    }
  }
}

// Gradient step (FloodfillMicromouse::run): strictly lower neighbour, N,E,S,W order.
// No candidate -> reflow once and try again. Returns false = no path.
bool chooseFloodDirection(Direction &bestDirection) {
  for (int pass = 0; pass < 2; pass++) {
    if (pass == 1) computeFloodFill();
    int lowest = floodValues[robotRow][robotCol];
    if (lowest >= FLOOD_INF) return false;                  // disconnected from every goal cell
    int best = -1;
    for (int d = 0; d < 4; d++) {
      if (!floodEdgeTraversable(robotRow, robotCol, (Direction)d)) continue;
      int nr, nc;
      neighbourCell(robotRow, robotCol, (Direction)d, nr, nc);
      if (floodValues[nr][nc] < lowest) { lowest = floodValues[nr][nc]; best = d; }
    }
    if (best >= 0) { bestDirection = (Direction)best; return true; }
  }
  return false;
}

// Absolute direction -> existing Action (FloodfillMicromouse::face)
Action actionForDirection(Direction current, Direction target) {
  switch ((target - current + 4) % 4) {
    case 0:  return GO_FORWARD;
    case 1:  return TURN_RIGHT;
    case 3:  return TURN_LEFT;
    default: return TURN_AROUND;
  }
}

void printFloodMap() {
  Serial.println("FLOOD map (row 0 = north):");
  for (int r = 0; r < MAZE_ROWS; r++) {
    char line[64]; int p = 0;
    for (int c = 0; c < MAZE_COLS; c++)
      p += snprintf(line + p, sizeof(line) - p, floodValues[r][c] >= FLOOD_INF ? "  ." : "%3d", floodValues[r][c]);
    Serial.println(line);
  }
}

// Stop the run cleanly (goal / no path)
void floodStop(const char *msg) {
  brakeAndStop();
  runEnabled = false;
  wasRunning = false;               // same as the tests: keep this message on the page
  floodTargetDir = -1;
  snprintf(floodStatus, sizeof(floodStatus), "%s", msg);
  setState("%s", msg);
  Serial.printf("%s\n", msg);
}

// Flood-fill decision for the current cell (walls were just recorded by markCurrentCellWalls).
// Returns false when the run was stopped (goal reached / no path).
bool decideFloodFill(Action &out) {
  // The existing code treats "blocked twice at the same place" as a wall ahead
  // (forceFrontBlocked) - put that into the map so the plan routes around it.
  // RECOVERY FIX: only a front the sensor CONFIRMED is written to the map. An unconfirmed one is
  // avoided for this decision only (temporary wall, undone right after the plan is chosen).
  bool tempFrontWall = false;
  MazeCell savedA, savedN;
  int tnr = -1, tnc = -1;
  if (forceFrontBlocked && forceFrontConfirmed) {
    setMazeEdge(robotRow, robotCol, robotDir, true, false, false);
    Serial.printf("FLOOD: front at (%d,%d) %s treated as WALL (repeated blocks)\n", robotRow, robotCol, dirName(robotDir));
  } else if (forceFrontBlocked) {
    neighbourCell(robotRow, robotCol, robotDir, tnr, tnc);
    if (validMazeCell(tnr, tnc) && edgeState(robotRow, robotCol, robotDir) != 2) {
      savedA = maze[robotRow][robotCol];
      savedN = maze[tnr][tnc];
      setMazeEdge(robotRow, robotCol, robotDir, true, false, false);
      tempFrontWall = true;
    }
  }
  if (isFloodGoalCell(robotRow, robotCol)) {
    if (tempFrontWall) { maze[robotRow][robotCol] = savedA; maze[tnr][tnc] = savedN; }   // RECOVERY FIX
    char msg[80];
    snprintf(msg, sizeof(msg), "FLOOD FILL: GOAL REACHED at (%d,%d) after %ld cell transitions",
             robotRow, robotCol, mapTransitions);
    floodStop(msg);
    return false;
  }
  computeFloodFill();                                      // map may have changed -> reflow
  Direction d;
  bool havePath = chooseFloodDirection(d);
  if (tempFrontWall) {                                     // RECOVERY FIX: undo the temporary wall
    maze[robotRow][robotCol] = savedA;
    maze[tnr][tnc] = savedN;
    computeFloodFill();
    if (havePath) {
      Serial.printf("FLOOD: front at (%d,%d) %s blocked but sensor sees it OPEN - avoiding it this time, NOT mapped\n",
                    robotRow, robotCol, dirName(robotDir));
    } else {
      Serial.printf("FLOOD: front at (%d,%d) %s is the only way and the sensor sees it OPEN - trying it again\n",
                    robotRow, robotCol, dirName(robotDir));
      havePath = chooseFloodDirection(d);
    }
  }
  if (!havePath) {
    floodStop("FLOOD FILL: NO PATH TO GOAL with the walls known so far - stopped");
    printFloodMap();
    return false;
  }
  floodTargetDir = d;
  out = actionForDirection(robotDir, d);
  snprintf(floodStatus, sizeof(floodStatus), "at (%d,%d) value %d -> go %s",
           robotRow, robotCol, floodValues[robotRow][robotCol], dirName(d));
  Serial.printf("FLOOD: at (%d,%d) facing %s, value %d -> target %s (%s)\n", robotRow, robotCol,
                dirName(robotDir), floodValues[robotRow][robotCol], dirName(d), actionName(out));
  printFloodMap();
  return true;
}

// /navmode?m=0|1 - only while the robot is stopped
void handleNavMode() {
  if (runEnabled) { server.send(200, "text/plain", "Navigation mode can only be changed while the robot is stopped"); return; }
  if (server.hasArg("m")) navigationMode = ((int)server.arg("m").toFloat() == 1) ? NAV_FLOOD_FILL : NAV_RIGHT_WALL;
  snprintf(floodStatus, sizeof(floodStatus), "%s", navigationMode == NAV_FLOOD_FILL ? "flood fill selected" : "idle");
  Serial.printf("NAV MODE: %s\n", navigationMode == NAV_FLOOD_FILL ? "FLOOD FILL" : "RIGHT WALL");
  server.send(200, "text/plain", "OK");
}

// ---------- dashboard data: /mapdata (fixed buffer, no big Strings) ----------
void handleMapData() {
  static char buf[900];      // FLOOD FILL: room for the flood values
  int visited = 0;
  for (int r = 0; r < MAZE_ROWS; r++) for (int c = 0; c < MAZE_COLS; c++) if (maze[r][c].visited) visited++;
  int p = snprintf(buf, sizeof(buf),
                   "{\"r\":%d,\"c\":%d,\"d\":%d,\"t\":%ld,\"v\":%d,\"sc\":%d,\"b\":%d,\"w\":%d,\"run\":%d,\"k\":\"",
                   robotRow, robotCol, (int)robotDir, mapTransitions, visited, mapStartCorner,
                   mapBlockedEvents, mapWarnings, (int)runEnabled);
  for (int r = 0; r < MAZE_ROWS; r++)
    for (int c = 0; c < MAZE_COLS; c++)
      p += snprintf(buf + p, sizeof(buf) - p, "%x%x", maze[r][c].known & 15, maze[r][c].walls & 15);
  p += snprintf(buf + p, sizeof(buf) - p, "\",\"vis\":\"");
  for (int r = 0; r < MAZE_ROWS; r++)
    for (int c = 0; c < MAZE_COLS; c++)
      if (p < (int)sizeof(buf) - 2) buf[p++] = maze[r][c].visited ? '1' : '0';
  // FLOOD FILL: flood value per cell (2 hex digits, ff = unreachable), mode, current value, target
  p += snprintf(buf + p, sizeof(buf) - p, "\",\"f\":\"");
  for (int r = 0; r < MAZE_ROWS; r++)
    for (int c = 0; c < MAZE_COLS; c++)
      p += snprintf(buf + p, sizeof(buf) - p, "%02x", floodValues[r][c] >= FLOOD_INF ? 255 : floodValues[r][c]);
  p += snprintf(buf + p, sizeof(buf) - p, "\",\"nav\":%d,\"fv\":%d,\"td\":%d,\"fs\":\"%s",
                (int)navigationMode, floodValues[robotRow][robotCol] >= FLOOD_INF ? -1 : floodValues[robotRow][robotCol],
                floodTargetDir, floodStatus);
  snprintf(buf + p, sizeof(buf) - p, "\",\"e\":\"%s\"}", mapLastEvent);
  server.send(200, "application/json", buf);
}
// /mapreset?sc=0|1  - resets ONLY the map data, and only while the robot is stopped
void handleMapReset() {
  if (runEnabled) { server.send(200, "text/plain", "Map can only be reset while the robot is stopped"); return; }
  if (server.hasArg("sc")) mapStartCorner = ((int)server.arg("sc").toFloat() == 1) ? 1 : 0;
  resetMazeMap();
  server.send(200, "text/plain", "OK");
}

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopMotors();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_NAME, WIFI_PASSWORD);
  WiFi.setSleep(false);
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/go", handleGo);
  server.on("/stop", handleStop);
  server.on("/resetenc", handleResetEnc);
  server.on("/test", handleTest);
  server.on("/celltest", handleCellTest);     // CELL TEST
  server.on("/cellres", handleCellReport);    // CELL TEST
  server.on("/mapdata", handleMapData);       // MAZE MAP
  server.on("/mapreset", handleMapReset);     // MAZE MAP
  server.on("/navmode", handleNavMode);       // FLOOD FILL
  resetMazeMap();                             // MAZE MAP: empty map at power-on
  server.on("/set", handleSet);
  server.on("/turntest", handleTurnTest);
  server.on("/setfc", handleSetFc);
  server.begin();
  xTaskCreatePinnedToCore(webTask, "web", 4096, nullptr, 1, nullptr, 0);

  Serial.print("Wi-Fi network: ");  Serial.println(WIFI_NAME);
  Serial.print("Open in browser: http://");
  Serial.println(WiFi.softAPIP());

  pinMode(LEFT_ENCODER_C1,  INPUT);
  pinMode(LEFT_ENCODER_C2,  INPUT);
  pinMode(RIGHT_ENCODER_C1, INPUT);
  pinMode(RIGHT_ENCODER_C2, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_C1),  leftEncoderISR,  RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_C1), rightEncoderISR, RISING);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);   // v11: 100 kHz is far more tolerant of long jumper wires than 400 kHz

  selectMux(MPU_CHANNEL);
  if (!mpu.begin(0x68)) {
    setState("ERROR: MPU6050 not found on TCA channel 4");
    while (true) delay(100);
  }
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  frontOk = initToF(tofFront, FRONT_TOF_CHANNEL, "Front");
  leftOk  = initToF(tofLeft,  LEFT_TOF_CHANNEL,  "Left");
  rightOk = initToF(tofRight, RIGHT_TOF_CHANNEL, "Right");
  frontLastMs = leftLastMs = rightLastMs = millis();
  frontWallMM = frontCenterMM + CELL_SIZE_MM / 2;

  Serial.printf("Counts/rev %ld -> starting counts/mm %.2f (self-calibrates while driving)\n",
                COUNTS_PER_WHEEL_REV, countsPerMM);

  if (AUTO_START) {
    waitWithTelemetry(3000);
    runEnabled = true;
  }
  setState("IDLE (last reset: %s) - put robot in a cell centre and press GO", resetReason());
}

// ============================================================
//  Main loop: one right-hand-rule decision per cell
// ============================================================
// ============================================================
//  CELL TEST: drive N cells straight from rest, stop, report (encoder estimates only)
// ============================================================
void runCellTest() {
  const int   n      = cellTestN;
  const float target = n * CELL_SIZE_MM;
  const float cpm    = countsPerMM;               // not changed during the test (self-cal is off)
  cellTestCount++;

  long l0, r0;
  readEncoders(l0, r0);                           // baseline before moving
  setState("DRIVE %d CELL(S) TEST #%d - target %.0f mm...", n, cellTestCount, target);
  Serial.printf("\n=== DRIVE N CELLS TEST #%d: %d cell(s) = %.0f mm, counts/mm %.3f, start enc L:%ld R:%ld ===\n",
                cellTestCount, n, target, cpm, l0, r0);

  cellTestTargetMM = target;
  cellTestPosMM    = 0;
  cellTestActive   = true;
  int cells;
  StopReason why;
  bool finished = driveUntilEvent(cells, why, true);   // true = side openings are NOT stop triggers
  cellTestActive   = false;
  if (!finished) why = STOP_ABORT;                      // it only returns false when STOP was pressed
  float brakeMM = cellTestPosMM;

  // let it come to rest; sensors + gyro keep updating, STOP stays responsive (motors are off)
  stopMotors();
  unsigned long t0 = millis();
  while (millis() - t0 < CELL_TEST_SETTLE_MS) { telemetryTick(); delay(2); }

  long l1, r1;
  readEncoders(l1, r1);
  long dL = l1 - l0, dR = r1 - r0;
  bool agree = encodersAgree(dL, dR);
  float cnt = agree ? (dL + dR) / 2.0f : (float)min(dL, dR);   // same rule as the drive itself
  float finalMM = cnt / cpm;
  float diff = finalMM - target;
  bool completed = (why == STOP_CELL_TARGET);

  snprintf(cellReport, sizeof(cellReport),
           "DRIVE N CELLS TEST #%d  (all distances are ENCODER ESTIMATES - measure the real travel with a ruler)\n"
           "Requested: %d cell(s) x %d mm = %.0f mm target\n"
           "Counts/mm used: %.3f%s\n"
           "Encoder travel: L %ld / R %ld counts%s\n"
           "Estimated distance when braking began: %.0f mm\n"
           "Estimated final distance after stopping: %.0f mm\n"
           "Final estimate - target: %+.0f mm\n"
           "Stop reason: %s -> %s",
           cellTestCount, n, CELL_SIZE_MM, target,
           cpm, distCalibrated ? " (self-calibrated earlier since power-on)" : " (starting value, not self-calibrated yet)",
           dL, dR, agree ? " (agree)" : " - WARNING: encoders DISAGREE, distance uses the smaller count",
           brakeMM, finalMM, diff,
           why == STOP_ABORT ? "STOP button pressed" : reasonName(why),
           completed ? "TARGET COMPLETED" : "ENDED EARLY");
  Serial.println(cellReport);
  setState("CELL TEST #%d %s: target %.0f, final est %.0f (%+.0f mm) - %s",
           cellTestCount, completed ? "done" : "ENDED EARLY", target, finalMM, diff,
           why == STOP_ABORT ? "STOP button pressed" : reasonName(why));
}

void loop() {
  if (!runEnabled) {
    if (wasRunning) {
      brakeAndStop();
      setState("STOPPED - press GO to run again");
      wasRunning = false;
    }
    waitWithTelemetry(20);
    return;
  }

  if (!wasRunning) {
    wasRunning = true;
    setState("GO pressed - hands off! calibrating...");
    waitWithTelemetry(1000);
    calibrateGyro(500);
    autoCalibrateSideCenter();
    yaw = 0;
    lastYawUs = micros(); lastRawRate = 0;
    pathHeading = 0;
    headingKnown = true;               // v13: placed straight at GO
    learnArmed = false;
    cellsMoved = 0;
    headingIntegral = 0;
    blockedStreak = 0;
    blockedAtCell = -999;
    forceFrontBlocked = false;
    mapRunPending = true;              // MAZE MAP: a new run - map is reset if this becomes a maze run
  }

  // ---------- CELL TEST: drive N cells straight, stop, stay idle ----------
  if (cellTestReq) {
    runCellTest();
    cellTestReq     = false;
    straightTestReq = false;   // a test pressed meanwhile is cancelled, never started afterwards
    turnTestReq     = false;
    runEnabled      = false;   // no turns, no continuation, no recovery
    wasRunning      = false;
    return;
  }

  // ---------- Straight test: drive to the next front wall and report wobble ----------
  if (straightTestReq) {
    setState("STRAIGHT TEST - driving to the front wall...");
    int c; StopReason w;
    if (driveUntilEvent(c, w, true)) {
      setState("TEST done: max %.1f deg, avg %.1f deg, %d wobbles (%d cells)",
               (float)statMaxHdg, (float)statAvgHdg, (int)statCrossings, c);
    }
    straightTestReq = false;
    runEnabled = false;
    wasRunning = false;
    return;
  }

  // ---------- v12 Turn test: 4 x 90 deg right in place, report overshoot ----------
  if (turnTestReq) {
    setState("TURN TEST - 4 right turns in place (each checked + fixed)...");
    float firsts[4] = {0, 0, 0, 0};
    int totalFixes = 0;
    float maxErr = 0;
    for (int i = 0; i < 4 && runEnabled; i++) {
      pathHeading -= 90.0;
      turnVerified(pathHeading);
      firsts[i] = lastTurnFirstError;
      totalFixes += lastTurnFixes;
      maxErr = max(maxErr, (float)fabs(lastTurnError));
      stillPauseAndRefineGyro(800);
    }
    setState("TURN TEST: 1st-pass errors %+.1f %+.1f %+.1f %+.1f deg (+over -under), %d fixes, "
             "final err max %.1f. Not facing START? too far -> raise Gyro, short -> lower",
             firsts[0], firsts[1], firsts[2], firsts[3], totalFixes, maxErr);
    turnTestReq = false;
    runEnabled = false;
    wasRunning = false;
    return;
  }

  if (mapRunPending) { mapRunPending = false; resetMazeMap(); }   // MAZE MAP: new maze run
  // ---------- 1) Stopped at a cell centre: look at the walls ----------
  waitWithTelemetry(ARRIVE_SETTLE_MS);
  bool wl, wf, wr;
  senseWalls(wl, wf, wr);
  markCurrentCellWalls(wl, wf, wr);              // MAZE MAP: record the raw sensed walls
  if (forceFrontBlocked) wf = true;              // v12: after repeated blocks ahead
  Action a;
  if (navigationMode == NAV_RIGHT_WALL) {        // FLOOD FILL: navigation-layer branch
    a = decide(wl, wf, wr);                      // existing right-wall decision (unchanged)
  } else {
    if (!decideFloodFill(a)) return;             // FLOOD FILL: goal reached / no path -> stopped
  }
  if (blockedStreak >= 3) a = TURN_AROUND;       // v12: 3 blocks at the same place - go back

  if (a != GO_FORWARD) {
    // ---------- 2) Get into a good position BEFORE turning ----------
    setState("cell %ld  L:%c F:%c R:%c -> %s (positioning)", (long)cellsMoved,
             wl ? 'Y' : 'n', wf ? 'Y' : 'n', wr ? 'Y' : 'n', actionName(a));

    // axle to the cell centre (+ a little extra room so the corners clear the wall)
    if (wf) alignToFrontWall(frontStopMM);
    stillPauseAndRefineGyro(PRE_TURN_PAUSE_MS);  // full 1 s stop (also re-zeroes gyro drift)
    if (!runEnabled) return;

    senseWalls(wl, wf, wr);                // look again, standing still, in position
    markCurrentCellWalls(wl, wf, wr);      // MAZE MAP: record the raw sensed walls again
    if (forceFrontBlocked) wf = true;
    Action confirmed;
    if (navigationMode == NAV_RIGHT_WALL) {      // FLOOD FILL: navigation-layer branch
      confirmed = decide(wl, wf, wr);            // existing right-wall re-check (unchanged)
    } else {
      if (!decideFloodFill(confirmed)) return;   // FLOOD FILL: re-plan with the re-sensed walls
    }
    if (blockedStreak >= 3) confirmed = TURN_AROUND;
    if (confirmed != a) {
      Serial.printf("Decision changed after re-check: %s -> %s\n",
                    actionName(a), actionName(confirmed));
      a = confirmed;
    }
  }

  setState("cell %ld  L:%c F:%c R:%c -> %s", (long)cellsMoved,
           wl ? 'Y' : 'n', wf ? 'Y' : 'n', wr ? 'Y' : 'n', actionName(a));

  // ---------- 3) Turn (both wheels, opposite directions) ----------
  if (a != GO_FORWARD) {
    doTurn(a);
    updateMapDirectionAfterTurn();               // MAZE MAP: direction follows the turn
    if (!runEnabled) return;
    waitWithTelemetry(POST_TURN_PAUSE_MS);
  }
  forceFrontBlocked = false;                     // v12: we've turned away from it
  if (blockedStreak >= 3) blockedStreak = 0;

  // FLOOD FILL: after turning, check the front again before moving (simulator step(): wallFront()).
  // Uses the normal majority sensing; only the FRONT is recorded (like the simulator), through the
  // normal map function (mirrored onto the neighbour). A wall -> no move, the next loop pass re-plans.
  if (navigationMode == NAV_FLOOD_FILL && a != GO_FORWARD) {
    bool wl2, wf2, wr2;
    senseWalls(wl2, wf2, wr2);
    setMazeEdge(robotRow, robotCol, robotDir, wf2, false, false);
    computeFloodFill();
    if (wf2) {
      Serial.printf("FLOOD: wall ahead after turning at (%d,%d) %s - not moving, re-planning\n",
                    robotRow, robotCol, dirName(robotDir));
      // RECOVERY FIX (log only): flag when the side we turned to read OPEN just before the turn
      bool sideSaidOpen = (a == TURN_LEFT && !wl) || (a == TURN_RIGHT && !wr);
      setState("flood fill: wall ahead after turning - re-planning%s",
               sideSaidOpen ? " (side read OPEN before turn - position may be off)" : "");
      return;
    }
  }

  // ---------- 4) Drive until a right opening or a front wall ----------
  int cells;
  StopReason why;
  mapBeforeDrive();                              // MAZE MAP: encoder baseline
  floodOneCellDrive = (navigationMode == NAV_FLOOD_FILL);   // FLOOD FILL: stop at the next cell centre
  if (!driveUntilEvent(cells, why, false)) return;
  cellsMoved += cells;
  mapAfterDrive(cells, why);                     // MAZE MAP: move on the map (conservative if BLOCKED)
  setState("stopped after %d cell(s): %s", cells, reasonName(why));
  if (why == STOP_BLOCKED) {        // v12: back off, straighten, re-centre, escalate
    recoverFromBlock();
  } else if (cells >= 1) {
    blockedStreak = 0;               // made real progress - forget old blocks
  }
}

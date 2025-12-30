#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <math.h>

// ================= PIN MAP (your map) =================
#define FLOW_PIN     0    // GPIO0  -> Flow sensor
#define RPM_PIN      1    // GPIO1  -> RPM Hall
#define FUEL_SENSE   2    // GPIO2  -> Fuel pump active (PC817), INPUT_PULLUP, LOW = ON

#define MAX_CS_CAT   3    // GPIO3  -> MAX6675 #1 CS (CAT)
#define MAX_CS_MIX   4    // GPIO4  -> MAX6675 #2 CS (MIXER)
#define MAX_SO       5    // GPIO5  -> MAX6675 SO (shared)
#define MAX_SCK      6    // GPIO6  -> MAX6675 SCK (shared)

#define X9C_UD       7    // GPIO7  -> X9C U/D
#define X9C_CS       8    // GPIO8  -> X9C CS
#define X9C_INC      9    // GPIO9  -> X9C INC

#define RELAY_ALARM  10   // GPIO10 -> Alarm relay (NC strategy)

// ================= USER CALIBRATION / TUNING =================
// Flow sensor label: F(Hz) = 5.5 * Q(L/min)  => 330 pulses per liter (tune with flowCalScale if needed)
const float FLOW_PULSES_PER_LITER = 330.0f;    // datasheet value
const float FLOW_HZ_PER_LMIN      = FLOW_PULSES_PER_LITER / 60.0f; // 5.5 Hz per L/min
const float FLOW_CAL_SCALE_DEFAULT = 1.0f;     // default scale when nothing stored
float flowCalScale = FLOW_CAL_SCALE_DEFAULT;   // persisted scale factor (editable)
bool flowCalScaleNeedsPersist = false;

// Hall RPM: 2 magnets on crank => 2 pulses per revolution (counting ONE edge only)
const float RPM_PULSES_PER_REV = 2.0f;

// Prius engine running threshold (edit here if needed)
const int ENGINE_RPM_MIN = 800;  // <<<<< change this if needed
const uint32_t HALL_RPM_LOW_HOLD_MS = 5000;
const int HALL_RPM_HIGH_THRESHOLD = 6000;

// ===== Pump command mapping =====
const int PUMP_PWM_MIN = 10;      // minimum command when pump should be on (you can tune later)
const int PUMP_PWM_MAX = 99;
const int PUMP_RPM_MAX = 5000;

// ===== Mixer temperature thresholds (°C) =====
const float MIX_WARN_C = 50.0f;   // 1s beep / 30s
const float MIX_HIGH_C = 60.0f;   // 3s beep / 30s + pump boost
const float MIX_CRIT_C = 70.0f;   // continuous alarm + full pump

// ===== CAT temperature thresholds (°C) =====
const float CAT_HIGH_C = 150.0f;  // 3s beep / 30s
const float CAT_CRIT_C = 180.0f;  // continuous

// ===== Flow clamp (ignore insane readings) =====
const float FLOW_CLAMP_LMIN = 120.0f;

// ===== Flow restriction detection (needs calibration table) =====
const float FLOW_RESTRICT_RATIO = 0.80f;     // measured < expected*0.8 => "Flow LOW"
const uint32_t FLOW_RESTRICT_HOLD_MS = 3000; // must persist this long

// ===== Calibration mismatch detection (reference-only map check) =====
const float FLOW_CAL_MISMATCH_TOL_FRAC = 0.10f;     // >10% deviation vs calibration => suspect
const uint32_t FLOW_CAL_CHECK_INTERVAL_MS = 10000;  // evaluate every ~10s
const int FLOW_CAL_MISMATCH_CONSEC = 3;             // need 3 consecutive failures (~30s)

// ===== Real "NO FLOW" detection (pump ON but flow ~0) =====
const float FLOW_MIN_VALID_LMIN = 1.0f;        // ignore tiny wobble/backflow/air
const uint32_t FLOW_STARTUP_GRACE_MS = 5000;   // wait after pump turns on
const uint32_t FLOW_FAULT_HOLD_MS   = 5000;    // must be low this long => fault

// ===== "Unexpected FLOW" detection (flow when pump/conditions say OFF) =====
const float FLOW_UNEXPECTED_LMIN = 1.5f;
const uint32_t FLOW_UNEXPECTED_HOLD_MS = 5000;

// ===== Manual flow-target control (L/min) =====
const float FLOW_TARGET_MAX_LMIN = 60.0f;
const uint32_t FLOW_PRIME_TIMEOUT_MS = 1500;
const float FLOW_PRIME_DETECT_LMIN = 0.3f;
const float FLOW_CONTROL_KP = 1.15f;
const float FLOW_CONTROL_HYST = 0.05f;
const float FLOW_CONTROL_TARGET_TOL_FRAC = 0.01f;   // try to stay within 1% of target
  const float FLOW_CONTROL_MIN_STEP = 0.2f;           // minimum adjustment when reacting
const float FLOW_CONTROL_STEP_UP_PPS = 10.0f;       // closed-loop increase rate (PWM per second)
const float FLOW_CONTROL_STEP_DOWN_PPS = 10.0f;     // closed-loop decrease rate (PWM per second)
const float FLOW_CONTROL_SLOPE_FLOOR = 0.08f;       // l/min per PWM step used for division safety
const float FLOW_CONTROL_FEED_FORWARD_BLEND = 0.20f;// blend between measured-error and map pull
const float FLOW_CONTROL_RAMP_RATE_PPS = 10.0f;     // max PWM change per second when leaving prime
const uint32_t FLOW_CONTROL_UPDATE_MS = 100;        // wait for sensor settle between adjustments (ms)
const float FLOW_CONTROL_KI = 0.8f;                 // integral gain (PWM per L/min * second)
const float FLOW_CONTROL_I_CLAMP = 25.0f;           // clamp for integral term (PWM)

// ===== Mute time =====
const uint32_t MUTE_MS = 5UL * 60UL * 1000UL;  // 5 minutes

// ===== Calibration validation thresholds =====
const float CAL_MIN_REAL_FLOW_LMIN = 2.0f;     // at least some steps must exceed this
const float CAL_MIN_SPREAD_LMIN    = 3.0f;     // max-min must be >= this
const int   CAL_MIN_GOOD_POINTS    = 3;        // how many steps must show "real flow"
const int   CAL_MAX_BAD_DECREASES  = 2;        // allow small noise, but not many decreases
const float CAL_DECREASE_TOL_LMIN  = 0.7f;     // decreases smaller than this are ignored

// ================= WIFI AP =================
const char* ap_ssid = "ECU_DEBUG";
const char* ap_pass = "12345678";

WebServer server(80);
Preferences prefs;

// ================= ECU STATE =================
volatile bool fuelActive = false;
bool autoMode = true;
float manualFlowTargetLmin = 0.0f;
bool manualFlowControlActive = false;
bool manualFlowPriming = false;
bool manualFlowTargetChanged = false;
uint32_t manualFlowPrimeStart = 0;
float manualFlowControlPwmF = 0.0f;
bool manualFlowRamping = false;
float manualFlowRampTarget = 0.0f;
uint32_t manualFlowLastAdjustMs = 0;
float manualFlowIntegral = 0.0f;

volatile int pwmCmd = 0;     // actual command applied
int x9cPos = 0;

float tCat = NAN;
float tMix = NAN;

// Flow
volatile uint32_t flowPulses = 0;
volatile float    flow_Lmin  = 0.0f;
volatile float    flow_Lh    = 0.0f;
volatile uint32_t flowHz_x100 = 0;
volatile uint32_t flowDpLast  = 0;

// RPM
volatile uint32_t rpmPulses = 0;
volatile float    rpmValue  = 0.0f;
volatile uint32_t rpmHz_x100 = 0;
volatile uint32_t rpmDpLast  = 0;
bool hallSensorFault = false;
uint32_t rpmLowFaultSince = 0;

// Alarm
bool alarmActive = false;
int  alarmLevel  = 0;            // 0 none, 1 warn, 2 high, 3 crit
bool alarmMuted  = false;
uint32_t alarmMutedUntil = 0;
bool alarmRelaySilenced = true;  // energized = silent
bool manualAlarmOverride = false;
bool manualAlarmShouldSound = false;

// Flow faults
bool flowRestrictFault = false;
bool flowNoMoveFault   = false;
bool flowUnexpectedFault = false;
bool flowCalMismatchFault = false;

uint32_t pumpNonZeroSince = 0;
uint32_t flowLowSince = 0;
uint32_t flowRestrictSince = 0;
uint32_t flowUnexpectedSince = 0;
uint32_t flowCalSampleMs = 0;
float    flowCalAccum = 0.0f;
uint32_t flowCalCount = 0;
int      flowCalBadCount = 0;

// ================= Event logging =================
enum EventCode : uint8_t {
  EVENT_MIX_WARN = 1,
  EVENT_MIX_HIGH,
  EVENT_MIX_CRIT,
  EVENT_MIX_SENSOR_FAULT,
  EVENT_CAT_HIGH,
  EVENT_CAT_CRIT,
  EVENT_CAT_SENSOR_FAULT,
  EVENT_FLOW_NO_MOVE,
  EVENT_FLOW_RESTRICT,
  EVENT_FLOW_UNEXPECTED,
  EVENT_FLOW_CAL_MISMATCH,
  EVENT_CAL_ATTENTION,
  EVENT_HALL_FAULT
};

struct EventLogEntry {
  uint32_t tsMs;
  uint8_t code;
  uint8_t reserved[3];
  float value;
};

const uint8_t EVENT_LOG_CAPACITY = 10;

struct EventLog {
  EventLogEntry entries[EVENT_LOG_CAPACITY];
  uint8_t count;
  uint8_t head; // index of next write
};

EventLog eventLog = {};

// ================= Calibration table =================
const int CAL_STEPS = 9;
int   calPwmStep[CAL_STEPS] = {20,30,40,50,60,70,80,90,100};
float calFlowLmin[CAL_STEPS];

// Built-in default calibration (dummy safe placeholders; replace later with your real “factory defaults”)
float DEFAULT_CAL_FLOW_LMIN[CAL_STEPS] = {
  0.0f, 1.0f, 3.3f, 5.3f, 7.4f, 9.5f, 11.4f, 13.3f, 15.8f
};

bool  calibrateComplete = false;      // “valid and usable for restriction logic”
bool  calibrating = false;

// keep last known-good calibration in RAM to prevent bad calibration overwrite
float calFlowLmin_lastGood[CAL_STEPS];
bool  hasLastGoodInRam = false;

// Calibration result reporting
bool calibLastOk = true;
String calibLastMsg = "OK";
bool calibFailLatched = false;
bool calibFailOneShot = false;
uint32_t calibFailSerial = 0;

// Calibration status classification:
enum CalState { CAL_OK, CAL_DEFAULT_RECOMMENDED, CAL_ERROR };
CalState calState = CAL_DEFAULT_RECOMMENDED;
bool calUsingDefault = false;

// ============ NVS layout (robust: magic + version + crc) ============
static const uint32_t CAL_MAGIC = 0xEC0CA1B0;
static const uint16_t CAL_VER   = 2;
static const uint32_t LOG_MAGIC = 0xEC0E1701;
static const uint16_t LOG_VER   = 1;

struct CalBlob {
  uint32_t magic;
  uint16_t ver;
  uint16_t reserved;
  float table[CAL_STEPS];
  float flowCalScale;
  uint32_t crc32;
};

// Legacy V1 blob (no flowCalScale)
struct CalBlobV1 {
  uint32_t magic;
  uint16_t ver;
  uint16_t reserved;
  float table[CAL_STEPS];
  uint32_t crc32;
};

struct EventLogBlob {
  uint32_t magic;
  uint16_t ver;
  uint16_t reserved;
  EventLogEntry entries[EVENT_LOG_CAPACITY];
  uint8_t count;
  uint8_t head;
  uint16_t reserved2;
  uint32_t crc32;
};

// Simple CRC32 (software) for small blob
uint32_t crc32_update(uint32_t crc, uint8_t data) {
  crc ^= data;
  for (int i = 0; i < 8; i++) {
    uint32_t mask = -(crc & 1);
    crc = (crc >> 1) ^ (0xEDB88320 & mask);
  }
  return crc;
}
uint32_t crc32_calc(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++) crc = crc32_update(crc, data[i]);
  return ~crc;
}

// ================= INTERRUPTS =================
void IRAM_ATTR isrFlow() { flowPulses++; }
// RPM interrupt counts ONE edge only (FALLING) so you don’t double-count by approach+leave
void IRAM_ATTR isrRpm()  { rpmPulses++; }

// ================= X9C (digital pot) =================
void x9cPulse() {
  digitalWrite(X9C_INC, LOW);  delayMicroseconds(300);
  digitalWrite(X9C_INC, HIGH); delayMicroseconds(300);
}
void x9cStep(bool up, int steps) {
  if (steps <= 0) return;
  digitalWrite(X9C_CS, LOW); delayMicroseconds(300);
  digitalWrite(X9C_UD, up ? HIGH : LOW); delayMicroseconds(300);
  for (int i = 0; i < steps; i++) x9cPulse();
  digitalWrite(X9C_CS, HIGH); delay(5);
}
void x9cHomeMin() {
  digitalWrite(X9C_CS, LOW); delayMicroseconds(300);
  digitalWrite(X9C_UD, LOW); delayMicroseconds(300);
  for (int i = 0; i < 140; i++) x9cPulse();
  digitalWrite(X9C_CS, HIGH); delay(10);
  x9cPos = 0;
}
void x9cSet(int target) {
  target = constrain(target, 0, 99);
  if (target == x9cPos) return;
  if (target > x9cPos) x9cStep(true,  target - x9cPos);
  else                 x9cStep(false, x9cPos - target);
  x9cPos = target;
}

// ================= MAX6675 =================
float max6675ReadC(uint8_t csPin) {
  digitalWrite(csPin, LOW);
  delayMicroseconds(5);

  uint16_t v = 0;
  for (int i = 15; i >= 0; i--) {
    digitalWrite(MAX_SCK, HIGH);
    delayMicroseconds(2);
    int bit = digitalRead(MAX_SO);
    v |= (bit << i);
    digitalWrite(MAX_SCK, LOW);
    delayMicroseconds(2);
  }
  digitalWrite(csPin, HIGH);

  if (v & 0x0004) return NAN;              // open thermocouple
  int tempCounts = (v >> 3) & 0x0FFF;
  return tempCounts * 0.25f;
}

// ================= NC ALARM RELAY STRATEGY =================
void setAlarmSilenced(bool silenced) {
  digitalWrite(RELAY_ALARM, silenced ? HIGH : LOW);
  alarmRelaySilenced = silenced;
}

// ================= HELPERS =================
static inline bool isNum(float x) { return !isnan(x) && isfinite(x); }

bool tableAllZero(const float* t) {
  for (int i = 0; i < CAL_STEPS; i++) {
    if (fabsf(t[i]) > 0.0001f) return false;
  }
  return true;
}

bool tableMatchesDefault(const float* t) {
  const float EPS = 0.01f;
  for (int i = 0; i < CAL_STEPS; i++) {
    if (fabsf(t[i] - DEFAULT_CAL_FLOW_LMIN[i]) > EPS) return false;
  }
  return true;
}

float expectedFlowFromCal(int pwm) {
  if (!calibrateComplete) return NAN;
  if (pwm <= calPwmStep[0]) return calFlowLmin[0];
  if (pwm >= calPwmStep[CAL_STEPS-1]) return calFlowLmin[CAL_STEPS-1];

  for (int i = 0; i < CAL_STEPS - 1; i++) {
    if (pwm >= calPwmStep[i] && pwm <= calPwmStep[i+1]) {
      float a = calPwmStep[i];
      float b = calPwmStep[i+1];
      float t = (pwm - a) / (b - a);
      return calFlowLmin[i] + t * (calFlowLmin[i+1] - calFlowLmin[i]);
    }
  }
  return NAN;
}

float estimatePwmForTargetFlow(float targetLmin) {
  if (targetLmin <= 0.0f) return 0.0f;

  if (calibrateComplete) {
    // find surrounding points by flow
    for (int i = 0; i < CAL_STEPS - 1; i++) {
      float f1 = calFlowLmin[i];
      float f2 = calFlowLmin[i+1];
      if (targetLmin >= f1 && targetLmin <= f2 && fabsf(f2 - f1) > 0.01f) {
        float t = (targetLmin - f1) / (f2 - f1);
        float p1 = calPwmStep[i];
        float p2 = calPwmStep[i+1];
        return p1 + t * (p2 - p1);
      }
    }
    // extrapolate if out of range using last slope
    float fA = calFlowLmin[CAL_STEPS-2];
    float fB = calFlowLmin[CAL_STEPS-1];
    float pA = calPwmStep[CAL_STEPS-2];
    float pB = calPwmStep[CAL_STEPS-1];
    float slope = (fabsf(fB - fA) < 0.01f) ? 0.0f : ((pB - pA) / (fB - fA));
    float extra = slope * (targetLmin - fB);
    return pB + extra;
  }

  // Fallback: simple linear map to the PWM window
  float frac = targetLmin / FLOW_TARGET_MAX_LMIN;
  if (frac < 0.0f) frac = 0.0f;
  if (frac > 1.0f) frac = 1.0f;
  return PUMP_PWM_MIN + frac * float(PUMP_PWM_MAX - PUMP_PWM_MIN);
}

float flowSlopeFromCalAtPwm(float pwm) {
  if (!calibrateComplete) return FLOW_CONTROL_SLOPE_FLOOR;

  if (pwm <= calPwmStep[0]) {
    float slope = (calFlowLmin[1] - calFlowLmin[0]) / float(calPwmStep[1] - calPwmStep[0]);
    return max(FLOW_CONTROL_SLOPE_FLOOR, slope);
  }

  for (int i = 0; i < CAL_STEPS - 1; i++) {
    float p1 = calPwmStep[i];
    float p2 = calPwmStep[i+1];
    if (pwm >= p1 && pwm <= p2) {
      float slope = (calFlowLmin[i+1] - calFlowLmin[i]) / float(p2 - p1);
      return max(FLOW_CONTROL_SLOPE_FLOOR, slope);
    }
  }

  float slope = (calFlowLmin[CAL_STEPS-1] - calFlowLmin[CAL_STEPS-2]) /
                float(calPwmStep[CAL_STEPS-1] - calPwmStep[CAL_STEPS-2]);
  return max(FLOW_CONTROL_SLOPE_FLOOR, slope);
}

float autoFlowTargetFromRpm(float rpm) {
  // Clamp-based starter curve: ~6 L/min at 1000 RPM up to ~45 L/min
  float q = 6.0f + 0.018f * (rpm - 1000.0f);
  if (q < 6.0f) q = 6.0f;
  if (q > 45.0f) q = 45.0f;
  return q;
}

float tempFlowBiasFromMix(float tMix) {
  if (!isNum(tMix)) return 0.0f;
  if (tMix >= MIX_CRIT_C) return 1e9f;
  if (tMix <= MIX_HIGH_C) return 0.0f;

  float t = (tMix - MIX_HIGH_C) / (MIX_CRIT_C - MIX_HIGH_C);
  t = constrain(t, 0.0f, 1.0f);
  return 15.0f * t;
}

void resetManualFlowControl(bool clearTarget) {
  manualFlowControlActive = false;
  manualFlowPriming = false;
  manualFlowTargetChanged = false;
  manualFlowPrimeStart = 0;
  manualFlowControlPwmF = 0.0f;
  manualFlowRamping = false;
  manualFlowRampTarget = 0.0f;
  manualFlowLastAdjustMs = 0;
  manualFlowIntegral = 0.0f;
  if (clearTarget) manualFlowTargetLmin = 0.0f;
}

int updateManualFlowControl(uint32_t now) {
  if (!manualFlowControlActive || manualFlowTargetLmin <= FLOW_CONTROL_HYST) return pwmCmd;

  float target = manualFlowTargetLmin;
  float error = target - flow_Lmin;
  float tol = max(target * 0.01f, FLOW_CONTROL_HYST);          // stop adjusting within ±1%
  float slowBand = target * 0.10f;                            // tighten rate inside ±10% error band

  // Prime at 100% until flow is detected
  if (flow_Lmin < FLOW_MIN_VALID_LMIN) {
    manualFlowPriming = true;
    if (manualFlowPrimeStart == 0) manualFlowPrimeStart = now;
    manualFlowControlPwmF = 99.0f;
    pwmCmd = 99;
    return pwmCmd;
  } else {
    manualFlowPriming = false;
    manualFlowPrimeStart = 0;
  }

  // On a new target, reset adjust timer
  if (manualFlowTargetChanged) {
    manualFlowTargetChanged = false;
    manualFlowLastAdjustMs = now;
  }

  if (fabsf(error) <= tol) {
    // Hold position when inside tolerance
    manualFlowLastAdjustMs = now;
    return pwmCmd;
  }

  uint32_t dtMs = now - manualFlowLastAdjustMs;
  if (dtMs < FLOW_CONTROL_UPDATE_MS) return pwmCmd;
  manualFlowLastAdjustMs = now;

  float ratePwmPerSec = (fabsf(error) > slowBand) ? 10.0f : 2.0f;
  float maxStep = ratePwmPerSec * (dtMs / 1000.0f);
  if (maxStep < FLOW_CONTROL_MIN_STEP) maxStep = FLOW_CONTROL_MIN_STEP;

  float fraction = target > 0.0f ? fabsf(error) / target : 1.0f;
  if (fraction > 1.0f) fraction = 1.0f;

  float delta = (error >= 0.0f ? 1.0f : -1.0f) * maxStep * fraction;
  manualFlowControlPwmF += delta;
  manualFlowControlPwmF = constrain(manualFlowControlPwmF, 0.0f, 99.0f);
  pwmCmd = int(manualFlowControlPwmF + 0.5f);
  return pwmCmd;
}

// ===== Calibration validation =====
bool validateCalibrationTable(const float* t, String &reason) {
  float minV = 1e9f, maxV = -1e9f;
  int goodPts = 0;
  int badDecreases = 0;

  for (int i = 0; i < CAL_STEPS; i++) {
    if (!isNum(t[i]) || t[i] < 0) {
      reason = "Invalid number(s)";
      return false;
    }
    if (t[i] < minV) minV = t[i];
    if (t[i] > maxV) maxV = t[i];
    if (t[i] >= CAL_MIN_REAL_FLOW_LMIN) goodPts++;
  }

  if (maxV < CAL_MIN_REAL_FLOW_LMIN) {
    reason = "No real flow measured";
    return false;
  }
  if (goodPts < CAL_MIN_GOOD_POINTS) {
    reason = "Too few valid points";
    return false;
  }
  if ((maxV - minV) < CAL_MIN_SPREAD_LMIN) {
    reason = "Too flat (pump/sensor?)";
    return false;
  }

  for (int i = 1; i < CAL_STEPS; i++) {
    float d = t[i] - t[i-1];
    if (d < -CAL_DECREASE_TOL_LMIN) badDecreases++;
  }
  if (badDecreases > CAL_MAX_BAD_DECREASES) {
    reason = "Not monotonic";
    return false;
  }

  reason = "OK";
  return true;
}

// ================= NVS Calibration Handling =================
bool loadCalFromNVS(float* outTable, CalState &stateOut) {
  stateOut = CAL_DEFAULT_RECOMMENDED;

  prefs.begin("ECU", true);
  size_t len = prefs.getBytesLength("calblob");
  if (len != sizeof(CalBlob)) {
    if (len == sizeof(CalBlobV1)) {
      CalBlobV1 blobV1;
      prefs.getBytes("calblob", &blobV1, sizeof(blobV1));
      prefs.end();

      // validate legacy blob
      if (blobV1.magic == CAL_MAGIC && blobV1.ver == 1) {
        uint32_t crc = crc32_calc((const uint8_t*)(&blobV1), sizeof(blobV1) - sizeof(blobV1.crc32));
        if (crc == blobV1.crc32) {
          for (int i = 0; i < CAL_STEPS; i++) outTable[i] = blobV1.table[i];
          flowCalScale = FLOW_CAL_SCALE_DEFAULT;
          flowCalScaleNeedsPersist = true; // migrate forward
          if (tableAllZero(outTable)) {
            stateOut = CAL_DEFAULT_RECOMMENDED;
            return true;
          }
          stateOut = CAL_OK;
          return true;
        }
      }
      stateOut = CAL_ERROR;
      return false;
    }

    prefs.end();
    if (len > 0) {
      stateOut = CAL_ERROR; // something stored but wrong size
    } else {
      stateOut = CAL_DEFAULT_RECOMMENDED; // nothing stored
      flowCalScale = FLOW_CAL_SCALE_DEFAULT;
      flowCalScaleNeedsPersist = true; // persist default on first save
    }
    return false;
  }

  CalBlob blob;
  prefs.getBytes("calblob", &blob, sizeof(blob));
  prefs.end();

  // validate blob
  if (blob.magic != CAL_MAGIC || blob.ver != CAL_VER) {
    stateOut = CAL_ERROR;
    return false;
  }

  uint32_t crc = crc32_calc((const uint8_t*)(&blob), sizeof(blob) - sizeof(blob.crc32));
  if (crc != blob.crc32) {
    stateOut = CAL_ERROR;
    return false;
  }

  // copy out
  for (int i = 0; i < CAL_STEPS; i++) outTable[i] = blob.table[i];
  flowCalScale = blob.flowCalScale;
  if (!isNum(flowCalScale) || flowCalScale <= 0.0f || flowCalScale > 10.0f) {
    flowCalScale = FLOW_CAL_SCALE_DEFAULT;
    flowCalScaleNeedsPersist = true;
  }

  if (tableAllZero(outTable)) {
    stateOut = CAL_DEFAULT_RECOMMENDED; // all zeros means "needs calibration"
    return true; // “loaded”, but not usable
  }

  stateOut = CAL_OK;
  return true;
}

void saveCalToNVS(const float* table) {
  CalBlob blob;
  blob.magic = CAL_MAGIC;
  blob.ver = CAL_VER;
  blob.reserved = 0;
  for (int i = 0; i < CAL_STEPS; i++) blob.table[i] = table[i];
  blob.flowCalScale = flowCalScale;
  blob.crc32 = crc32_calc((const uint8_t*)(&blob), sizeof(blob) - sizeof(blob.crc32));

  prefs.begin("ECU", false);
  prefs.putBytes("calblob", &blob, sizeof(blob));
  prefs.end();

  flowCalScaleNeedsPersist = false;
}

String eventLabel(uint8_t code) {
  switch (code) {
    case EVENT_MIX_WARN: return "Mixer temp warning";
    case EVENT_MIX_HIGH: return "Mixer temp high";
    case EVENT_MIX_CRIT: return "Mixer temp critical";
    case EVENT_MIX_SENSOR_FAULT: return "Mixer sensor fault";
    case EVENT_CAT_HIGH: return "Catalyst temp high";
    case EVENT_CAT_CRIT: return "Catalyst temp critical";
    case EVENT_CAT_SENSOR_FAULT: return "Catalyst sensor fault";
    case EVENT_FLOW_NO_MOVE: return "No flow";
    case EVENT_FLOW_RESTRICT: return "Flow restriction";
    case EVENT_FLOW_UNEXPECTED: return "Unexpected flow";
    case EVENT_FLOW_CAL_MISMATCH: return "Flow vs calibration mismatch";
    case EVENT_CAL_ATTENTION: return "Calibration attention";
    case EVENT_HALL_FAULT: return "Hall sensor fault";
    default: return "Unknown event";
  }
}

void saveEventLogToNVS() {
  EventLogBlob blob;
  blob.magic = LOG_MAGIC;
  blob.ver = LOG_VER;
  blob.reserved = 0;
  for (int i = 0; i < EVENT_LOG_CAPACITY; i++) blob.entries[i] = eventLog.entries[i];
  blob.count = eventLog.count;
  blob.head = eventLog.head;
  blob.reserved2 = 0;
  blob.crc32 = crc32_calc((const uint8_t*)(&blob), sizeof(blob) - sizeof(blob.crc32));

  prefs.begin("ECU", false);
  prefs.putBytes("elog", &blob, sizeof(blob));
  prefs.end();
}

bool loadEventLogFromNVS() {
  prefs.begin("ECU", true);
  size_t len = prefs.getBytesLength("elog");
  if (len != sizeof(EventLogBlob)) {
    prefs.end();
    eventLog = {};
    return false;
  }

  EventLogBlob blob;
  prefs.getBytes("elog", &blob, sizeof(blob));
  prefs.end();

  if (blob.magic != LOG_MAGIC || blob.ver != LOG_VER) {
    eventLog = {};
    return false;
  }

  uint32_t crc = crc32_calc((const uint8_t*)(&blob), sizeof(blob) - sizeof(blob.crc32));
  if (crc != blob.crc32) {
    eventLog = {};
    return false;
  }

  eventLog.count = min(blob.count, EVENT_LOG_CAPACITY);
  eventLog.head = blob.head % EVENT_LOG_CAPACITY;
  for (int i = 0; i < EVENT_LOG_CAPACITY; i++) eventLog.entries[i] = blob.entries[i];
  return true;
}

void clearEventLog(bool persist) {
  eventLog = {};
  if (persist) saveEventLogToNVS();
}

void appendEventLog(EventCode code, float value) {
  EventLogEntry e = {};
  e.tsMs = millis();
  e.code = code;
  e.value = value;

  eventLog.entries[eventLog.head] = e;
  eventLog.head = (eventLog.head + 1) % EVENT_LOG_CAPACITY;
  if (eventLog.count < EVENT_LOG_CAPACITY) eventLog.count++;

  saveEventLogToNVS();
}

bool isLogValueNum(float v) {
  return !isnan(v) && isfinite(v);
}

void recordEventEdge(bool active, bool &prevState, EventCode code, float value) {
  if (active && !prevState) appendEventLog(code, value);
  prevState = active;
}

// ================= CALIBRATION PROCESS =================
int   calStepIndex = -1;
uint32_t calStepStart = 0;
float  calStepAccumFlow = 0.0f;
uint32_t calStepAccumCount = 0;

// ================= WEB UI =================
String pageHTML() {
  return R"HTML(
<!doctype html><html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ECU Debug</title>
  <style>
    body { font-family: system-ui, sans-serif; margin: 16px; }
    .card { padding: 14px; border: 1px solid #d0d0d0; border-radius: 12px; margin: 12px 0; }
    .big { font-size: 1.05em; }
    .row { display:flex; justify-content:space-between; padding: 6px 0; gap: 12px; align-items: center; }
    code { background:#f3f3f3; padding:2px 6px; border-radius:6px; }
    .small { font-size: 0.9em; opacity: 0.75; }
    .mono { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace; }
    .error { color: #c40000; font-weight: 800; }
    .ok { color: #0a7a0a; font-weight: 800; }
    .warn { color: #b36b00; font-weight: 800; }
    .muted { opacity: 0.55; }
    .value { font-weight: 700; }
    .mode-btn { padding: 8px 14px; border-radius: 18px; border: 1px solid #aaa; background: #f5f5f5; min-width: 96px; }
    .mode-btn.manual { background: #0b6cff; color: white; border-color: #0b6cff; }
    button { padding: 10px 12px; border-radius: 10px; border: 1px solid #aaa; background: #fafafa; }
    button:disabled { opacity: 0.45; }
    input[type=range] { height: 44px; -webkit-appearance: none; appearance: none; background: transparent; }
    input[type=range]:focus { outline: none; }
    input[type=range]::-webkit-slider-runnable-track { height: 12px; background: #e2e2e2; border-radius: 12px; border: 1px solid #c8c8c8; }
    input[type=range]::-webkit-slider-thumb { -webkit-appearance: none; appearance: none; width: 28px; height: 28px; background: #0b6cff; border-radius: 50%; margin-top: -9px; border: 1px solid #0b6cff; }
    input[type=range]::-moz-range-track { height: 12px; background: #e2e2e2; border-radius: 12px; border: 1px solid #c8c8c8; }
    input[type=range]::-moz-range-thumb { width: 28px; height: 28px; background: #0b6cff; border-radius: 50%; border: 1px solid #0b6cff; }
    .spacer { height: 14px; }
    .status-actions { display: flex; align-items: center; gap: 8px; }
    .modal { position: fixed; inset: 0; background: rgba(0,0,0,0.35); display: none; align-items: center; justify-content: center; padding: 16px; }
    .modal.show { display: flex; }
    .modal-content { background: #fff; border-radius: 12px; padding: 16px; width: min(520px, 100%); box-shadow: 0 10px 30px rgba(0,0,0,0.25); }
    .modal-header { display: flex; justify-content: space-between; align-items: center; gap: 12px; }
    .log-list { max-height: 320px; overflow-y: auto; padding: 8px 0; }
    .log-row { padding: 6px 0; border-bottom: 1px solid #ececec; }
    .modal-actions { display: flex; gap: 10px; justify-content: flex-end; margin-top: 10px; }
    .field-row { display:flex; align-items:center; gap:8px; margin-top:8px; flex-wrap:wrap; }
    .field-row input { padding: 8px; border-radius: 8px; border: 1px solid #bbb; min-width: 90px; }
  </style>
</head>
<body>
  <h2>ECU Debug (AP)</h2>

  <div class="card">
    <div class="row big">
      <div>Status</div>
      <div class="status-actions">
        <div id="statusMsg" class="ok">OK</div>
        <button id="logsBtn" onclick="openLogs()">Logs</button>
      </div>
    </div>
    <div class="row small">
      <div>Mode</div>
      <div>
        <span id="modeStatus" class="mono" style="margin-right:6px;">Auto mode</span>
        <button id="modeBtn" class="mode-btn" onclick="toggleMode()">Change mode</button>
      </div>
    </div>
    <div class="row small">
      <div>Calibration</div>
      <div id="calStatus" class="mono">OK</div>
    </div>
  </div>

  <div class="card">
    <div class="row big"><div>Fuel</div><div class="mono value" id="fuel">?</div></div>

    <div class="row big">
      <div>RPM</div>
      <div class="mono">
        <span class="value" id="rpm">?</span>
        <span class="small">(<span id="rpmhz">?</span> Hz, dp=<span id="rpmdp">?</span>)</span>
      </div>
    </div>

    <div class="row big">
      <div>Flow</div>
      <div class="mono">
        <span class="value" id="flow">?</span> L/min (<span class="value" id="flowh">?</span> L/h)
        <span class="small">- <span id="flowhz">?</span> Hz, dp=<span id="flowdp">?</span></span>
      </div>
    </div>

    <div class="row big"><div>Tcat</div><div class="mono"><span class="value" id="tcat">?</span> &deg;C</div></div>
    <div class="row big"><div>Tmix</div><div class="mono"><span class="value" id="tmix">?</span> &deg;C</div></div>

    <div class="row big"><div>PWM (cmd)</div><div class="mono" id="pwm">?</div></div>
    <div class="row big"><div>X9C (pos)</div><div class="mono" id="x9c">?</div></div>

    <div class="row big"><div>Alarm</div><div class="mono value" id="alarm">?</div></div>
    <div class="row big"><div>Alarm Relay Power (On/Off)</div><div class="mono value" id="relayStatus">?</div></div>
  </div>

  <div class="card">
    <div class="row small">
      <div>Calibration Table</div>
      <div class="mono" id="caltable">...</div>
    </div>
    <div class="row small manual-only" id="flowCalScaleRow">
      <div>Flow cal scale</div>
      <div class="mono" id="flowScaleDisplay">...</div>
    </div>
  </div>

  <div class="card">
    <div class="big">Manual PWM: <span id="sval">0</span></div>
    <input type="range" min="0" max="99" value="0" id="slider" style="width:100%;" oninput="setPWM(this.value)">
    <div class="spacer"></div>
    <div class="big">Target Flow (L/min): <span id="flowSval">0</span></div>
    <input type="range" min="0" max="60" step="0.5" value="0" id="flowSlider" style="width:100%;" oninput="setFlowTarget(this.value)">
    <div class="small muted">Set flow &gt;0 to enable semi-automatic control (priming burst, then closed-loop).</div>

    <div class="spacer"></div>

    <div class="field-row manual-only" id="flowCalScaleControls">
      <label for="flowCalScaleInput" class="big">Flow cal scale:</label>
      <input id="flowCalScaleInput" type="number" min="0.1" max="10" step="0.01" value="1.00">
      <button onclick="saveFlowCalScale()">Save</button>
    </div>
    <div class="small muted">Adjust measured flow scaling without reflashing firmware.</div>

    <div class="spacer"></div>

    <button id="calibBtn" class="manual-only" onclick="startCalibration()">Calibrate</button>

    <div class="spacer"></div>

    <button id="restoreBtn" onclick="restoreDefaults()">Restore Default Calibration</button>

    <div class="spacer"></div>

    <button id="muteBtn" onclick="toggleMute()">Mute Alarm</button>
    <button id="alarmToggleBtn" onclick="toggleAlarm()" style="display:none;">Toggle Alarm</button>

    <div class="small" style="margin-top:10px;">IP: <code>192.168.4.1</code></div>
  </div>

  <div id="logModal" class="modal">
    <div class="modal-content">
      <div class="modal-header">
        <div class="big">Event log (last 10)</div>
        <button onclick="closeLogs()">Close</button>
      </div>
      <div class="small muted">Newest first. Values are captured when each event first triggered.</div>
      <div id="logList" class="log-list mono">Loading...</div>
      <div class="modal-actions">
        <button onclick="clearLogs()">Clear history</button>
        <button onclick="closeLogs()">Close</button>
      </div>
    </div>
  </div>

<script>
function applyLevel(el, level){
  if (!el) return;
  el.classList.remove("warn", "error");
  if (level >= 2) el.classList.add("error");
  else if (level === 1) el.classList.add("warn");
}

let currentAutoMode = true;
let sliderActive = false;
let pendingPwmValue = null;
let sendingPwm = false;

let flowSliderActive = false;
let pendingFlowValue = null;
let sendingFlow = false;

const sliderEl = document.getElementById("slider");
["pointerdown","touchstart","mousedown"].forEach(evt => sliderEl.addEventListener(evt, () => { sliderActive = true; }));
["pointerup","pointercancel","pointerleave","touchend","touchcancel","mouseup"].forEach(evt => sliderEl.addEventListener(evt, () => { sliderActive = false; }));

const flowSliderEl = document.getElementById("flowSlider");
["pointerdown","touchstart","mousedown"].forEach(evt => flowSliderEl.addEventListener(evt, () => { flowSliderActive = true; }));
["pointerup","pointercancel","pointerleave","touchend","touchcancel","mouseup"].forEach(evt => flowSliderEl.addEventListener(evt, () => { flowSliderActive = false; }));

const logModal = document.getElementById("logModal");
const logList = document.getElementById("logList");
const flowCalScaleInput = document.getElementById("flowCalScaleInput");
const flowScaleDisplay = document.getElementById("flowScaleDisplay");
const flowCalScaleRow = document.getElementById("flowCalScaleRow");
const flowCalScaleControls = document.getElementById("flowCalScaleControls");
const manualOnlyEls = Array.from(document.querySelectorAll(".manual-only"));

function formatLogValue(v){
  if (v === null || Number.isNaN(v) || typeof v !== "number") return "n/a";
  if (Math.abs(v) >= 100) return v.toFixed(0);
  return v.toFixed(2);
}

function renderLogs(data){
  if (!logList) return;
  logList.innerHTML = "";
  if (!data || !Array.isArray(data.events) || data.events.length === 0) {
    logList.textContent = "No events recorded.";
    return;
  }
  data.events.forEach(ev => {
    const row = document.createElement("div");
    row.className = "log-row";
    const tsText = (typeof ev.ts === "number") ? ` @${(ev.ts/1000).toFixed(1)}s` : "";
    const label = ev.label || `Code ${ev.code}`;
    row.textContent = `${label} — value: ${formatLogValue(ev.value)}${tsText}`;
    logList.appendChild(row);
  });
}

async function refreshLogs(){
  if (!logList) return;
  logList.textContent = "Loading...";
  try{
    const r = await fetch('/logs', {cache: "no-store"});
    const d = await r.json();
    renderLogs(d);
  }catch(e){
    logList.textContent = "Failed to load logs.";
  }
}

function openLogs(){
  if (!logModal) return;
  logModal.classList.add("show");
  refreshLogs();
}

function closeLogs(){
  if (!logModal) return;
  logModal.classList.remove("show");
}

async function clearLogs(){
  try{ await fetch('/logs?clear=1'); } catch(e){}
  refreshLogs();
}

async function saveFlowCalScale(){
  if (!flowCalScaleInput) return;
  const val = Number(flowCalScaleInput.value);
  if (!isFinite(val) || val <= 0) {
    alert("Please enter a positive number for flow calibration scale.");
    return;
  }
  try{
    await fetch('/set?flowCalScale=' + encodeURIComponent(val.toFixed(3)));
    refresh();
  }catch(e){}
}

async function refresh(){
  try{
    const r = await fetch('/data', {cache: "no-store"});
    const d = await r.json();

    fuel.textContent  = d.fuel ? "ON" : "off";

    rpm.textContent   = (typeof d.rpm === "number" ? d.rpm : 0);
    rpmhz.textContent = (typeof d.rpmHz === "number" ? d.rpmHz.toFixed(2) : "0.00");
    rpmdp.textContent = (typeof d.rpmDp === "number" ? d.rpmDp : 0);

    const flm = (typeof d.flowLmin === "number") ? d.flowLmin : 0;
    const flh = (typeof d.flowLh   === "number") ? d.flowLh   : 0;
    flow.textContent  = flm.toFixed(2);
    flowh.textContent = flh.toFixed(0);
    flowhz.textContent = (typeof d.flowHz === "number" ? d.flowHz.toFixed(2) : "0.00");
    flowdp.textContent = (typeof d.flowDp === "number" ? d.flowDp : 0);

    tcat.textContent  = (typeof d.tCat === "number" ? d.tCat.toFixed(1) : "FAULT");
    tmix.textContent  = (typeof d.tMix === "number" ? d.tMix.toFixed(1) : "FAULT");

    pwm.textContent   = d.pwm;
    x9c.textContent   = d.x9c;
    const alarmLevelText = (typeof d.alarmLevel === "number" && d.alarmLevel > 0) ? ` (Level ${d.alarmLevel})` : "";
    alarm.textContent = d.alarm ? `ON${alarmLevelText}` : "off";

    applyLevel(tmix, Number(d.tMixLevel || 0));
    applyLevel(tcat, Number(d.tCatLevel || 0));
    applyLevel(flow, Number(d.flowLevel || 0));
    applyLevel(flowh, Number(d.flowLevel || 0));
    applyLevel(alarm, d.alarm ? 2 : 0);

    statusMsg.textContent = d.status || "OK";
    statusMsg.className = "";
    statusMsg.classList.add(d.statusClass || "ok");

    calStatus.textContent = d.calMsg || "OK";
    const calClass = d.calFail ? "error" : (d.calOk ? "ok" : "warn");
    calStatus.className = "mono " + calClass;

    currentAutoMode = !!d.autoMode;
    modeBtn.textContent = "Change mode";
    modeBtn.classList.toggle("manual", !currentAutoMode);
    modeStatus.textContent = currentAutoMode ? "Auto mode" : "Manual mode";

    const relayPowered = !!d.alarmRelaySilenced;
    relayStatus.textContent = relayPowered ? "On" : "Off";

    const flowTarget = (typeof d.flowTargetLmin === "number") ? d.flowTargetLmin : 0;
    const manual = !currentAutoMode;
    manualOnlyEls.forEach(el => {
      if (!el) return;
      el.style.display = manual ? "" : "none";
    });
    sliderEl.disabled = !manual || d.calibrating || flowTarget > 0.05;
    flowSliderEl.disabled = !manual || d.calibrating;
    calibBtn.disabled = !manual || d.calibrating;
    restoreBtn.disabled = !manual || d.calibrating;   // ONLY manual mode
    restoreBtn.style.display = manual ? "inline" : "none";

    if (manual) {
      alarmToggleBtn.style.display = "inline";
      alarmToggleBtn.textContent = (d.alarmRelaySilenced ? "Sound Alarm" : "Silence Alarm");
    } else {
      alarmToggleBtn.style.display = "none";
    }

    sval.textContent  = d.pwm;
    if (!sliderActive) sliderEl.value = d.pwm;

    flowSval.textContent = flowTarget.toFixed(1);
    if (!flowSliderActive) flowSliderEl.value = flowTarget;

    // cal table display
    if (Array.isArray(d.cal) && d.cal.length === 9) {
      const steps = [20,30,40,50,60,70,80,90,100];
      let s = "";
      for (let i=0;i<steps.length;i++){
        s += steps[i] + "%=" + Number(d.cal[i]).toFixed(1) + " ";
      }
      caltable.textContent = s.trim();
    }
    if (typeof d.flowCalScale === "number") {
      if (flowScaleDisplay) flowScaleDisplay.textContent = d.flowCalScale.toFixed(3);
      if (flowCalScaleInput && !flowCalScaleInput.matches(":focus")) {
        flowCalScaleInput.value = d.flowCalScale.toFixed(3);
      }
    }

    muteBtn.disabled = !d.alarm;
    muteBtn.textContent = d.alarmMuted ? "Unmute Alarm" : "Mute Alarm";

    calibBtn.textContent = d.calibrating ? "Calibrating..." : "Calibrate";

    if (d.calFailNotice) {
      const reason = d.calFailMsg || d.calMsg || "unknown reason";
      alert(`Calibration failed: ${reason}. Defaults restored. Please check flow sensor/pump and retry.`);
    }

  }catch(e){}
}

async function setPWM(v){
  const val = Number(v);
  sval.textContent = val;
  if (flowSliderEl) {
    flowSliderEl.value = 0;
    flowSval.textContent = "0.0";
  }
  pendingPwmValue = val;
  if (sendingPwm) return;

  sendingPwm = true;
  while (pendingPwmValue !== null) {
    const next = pendingPwmValue;
    pendingPwmValue = null;
    try {
      await fetch('/set?pwm=' + next);
    } catch(e) {
      // swallow
    }
  }
  sendingPwm = false;
}

async function setFlowTarget(v){
  const val = Number(v);
  flowSval.textContent = val.toFixed(1);
  pendingFlowValue = val;
  if (sendingFlow) return;

  sendingFlow = true;
  while (pendingFlowValue !== null) {
    const next = pendingFlowValue;
    pendingFlowValue = null;
    try {
      await fetch('/set?flow=' + next);
    } catch(e) {
      // swallow
    }
  }
  sendingFlow = false;
}

async function toggleMode(){
  const wantManual = currentAutoMode;
  await fetch('/set?mode=' + (wantManual ? 'manual' : 'auto'));
}

async function startCalibration(){
  if (!confirm("Start calibration?\nMake sure discharge hose is in a bucket.")) return;
  if (!confirm("Are you sure? Pump will run through speeds.")) return;
  await fetch('/calibrate');
}

async function restoreDefaults(){
  if (!confirm("Restore DEFAULT calibration table?\nThis overwrites saved calibration.")) return;
  if (!confirm("Are you REALLY sure?")) return;
  await fetch('/restore');
}

async function toggleMute(){
  const wantMute = (muteBtn.textContent.indexOf("Unmute") === -1);
  await fetch('/set?mute=' + (wantMute ? 1 : 0));
}

async function toggleAlarm(){
  const wantSound = (alarmToggleBtn.textContent.indexOf("Sound") !== -1);
  await fetch('/set?alarm=' + (wantSound ? 'on' : 'off'));
}

setInterval(refresh, 400);
refresh();
</script>
</body></html>
)HTML";
}

// ================= WEB HANDLERS =================
void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Pragma", "no-cache");
  server.send(200, "text/html; charset=utf-8", pageHTML());
}

// ================= CAL runtime flags =================
bool calWarningLevel1 = false; // set for calibration errors/failures that need attention

void handleData() {
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Pragma", "no-cache");

  float fHz = flowHz_x100 / 100.0f;
  float rHz = rpmHz_x100  / 100.0f;
  bool calFailNotice = calibFailOneShot;
  calibFailOneShot = false;

  // status string
  String status = "OK";
  String statusClass = "ok";

  if (alarmActive) {
    statusClass = (alarmLevel >= 3) ? "error" : "warn";
    String msg = "";

    if (!isNum(tMix)) msg += "Mixer sensor FAULT; ";
    if (!isNum(tCat)) msg += "Cat sensor FAULT; ";
    if (hallSensorFault) msg += "Hall sensor malfunction; ";

    if (isNum(tMix)) {
      if (tMix >= MIX_CRIT_C) msg += "Mixer OVERHEAT; ";
      else if (tMix >= MIX_HIGH_C) msg += "Mixer temp high; ";
      else if (tMix >= MIX_WARN_C) msg += "Mixer temp warning; ";
    }
    if (isNum(tCat)) {
      if (tCat >= CAT_CRIT_C) msg += "Catalyst OVERHEAT; ";
      else if (tCat >= CAT_HIGH_C) msg += "Catalyst temp high; ";
    }

    if (flowNoMoveFault) msg += "NO FLOW; ";
    if (flowRestrictFault) msg += "Flow LOW; ";
    if (flowUnexpectedFault) msg += "Unexpected FLOW; ";
    if (flowCalMismatchFault) msg += "Flow vs cal mismatch; ";

    if (calWarningLevel1) msg += "Calibration needs attention; ";

    if (msg.endsWith("; ")) msg.remove(msg.length() - 2);
    status = (alarmLevel >= 3 ? "CRITICAL: " : "WARNING: ") + msg;
  } else if (calWarningLevel1) {
    status = "WARNING: Calibration needs attention";
    statusClass = "warn";
  }

  int tMixLevel = 0;
  if (!isNum(tMix)) tMixLevel = 3;
  else if (tMix >= MIX_CRIT_C) tMixLevel = 3;
  else if (tMix >= MIX_HIGH_C) tMixLevel = 2;
  else if (tMix >= MIX_WARN_C) tMixLevel = 1;

  int tCatLevel = 0;
  if (!isNum(tCat)) tCatLevel = 3;
  else if (tCat >= CAT_CRIT_C) tCatLevel = 3;
  else if (tCat >= CAT_HIGH_C) tCatLevel = 2;

  int flowLevel = 0;
  String flowFaultType = "";
  if (flowNoMoveFault)        { flowLevel = 3; flowFaultType = "no_flow"; }
  else if (flowUnexpectedFault) { flowLevel = 2; flowFaultType = "unexpected"; }
  else if (flowCalMismatchFault) { flowLevel = 2; flowFaultType = "cal_mismatch"; }
  else if (flowRestrictFault) { flowLevel = 1; flowFaultType = "restricted"; }

  // calibration message
  String calMsg;
  String calFailMsg;
  bool calOk = false;
  bool calError = false;
  if (calibFailLatched) {
    calFailMsg = calibLastMsg;
    calMsg = calibLastMsg;
    calError = true;
  } else if (calState == CAL_ERROR) {
    calMsg = "Calibration error – please recalibrate or restore defaults";
    calOk = false;
    calError = true;
  } else if (calState == CAL_DEFAULT_RECOMMENDED) {
    if (calUsingDefault) {
      calMsg = "OK (default)";
      calOk = true;
    } else {
      calMsg = "Calibration recommended";
      calOk = false;
    }
  } else {
    calMsg = calUsingDefault ? "OK (default)" : "OK";
    calOk = true;
  }
  bool calFail = calError || calibFailLatched;

  String json = "{";
  json += "\"fuel\":" + String(fuelActive ? "true" : "false") + ",";

  json += "\"rpm\":"    + String((int)(rpmValue + 0.5f)) + ",";
  json += "\"rpmHz\":"  + String(rHz, 2) + ",";
  json += "\"rpmDp\":"  + String((uint32_t)rpmDpLast) + ",";
  json += "\"hallFault\":" + String(hallSensorFault ? "true" : "false") + ",";

  json += "\"flowLmin\":" + String(flow_Lmin, 3) + ",";
  json += "\"flowLh\":"   + String(flow_Lh, 1) + ",";
  json += "\"flowHz\":"   + String(fHz, 2) + ",";
  json += "\"flowDp\":"   + String((uint32_t)flowDpLast) + ",";
  json += "\"flowTargetLmin\":" + String(manualFlowTargetLmin, 2) + ",";
  json += "\"flowCalScale\":" + String(flowCalScale, 3) + ",";

  json += "\"tCat\":" + (isNum(tCat) ? String(tCat, 1) : String("null")) + ",";
  json += "\"tMix\":" + (isNum(tMix) ? String(tMix, 1) : String("null")) + ",";

  json += "\"pwm\":"  + String(pwmCmd) + ",";
  json += "\"x9c\":"  + String(x9cPos) + ",";

  json += "\"alarm\":" + String(alarmActive ? "true" : "false") + ",";
  json += "\"alarmMuted\":" + String(alarmMuted ? "true" : "false") + ",";
  json += "\"alarmRelaySilenced\":" + String(alarmRelaySilenced ? "true" : "false") + ",";
  json += "\"autoMode\":" + String(autoMode ? "true" : "false") + ",";
  json += "\"calibrating\":" + String(calibrating ? "true" : "false") + ",";
  json += "\"alarmLevel\":" + String(alarmLevel) + ",";

  json += "\"tableOk\":" + String(calibrateComplete ? "true" : "false") + ",";
  json += "\"cal\":[";
  for (int i = 0; i < CAL_STEPS; i++) {
    json += String(calFlowLmin[i], 2);
    if (i < CAL_STEPS - 1) json += ",";
  }
  json += "],";

  json += "\"calOk\":" + String(calOk ? "true" : "false") + ",";
  json += "\"calMsg\":\"" + calMsg + "\",";
  json += "\"calFail\":" + String(calFail ? "true" : "false") + ",";
  json += "\"calFailMsg\":\"" + (calFailMsg.length() ? calFailMsg : calMsg) + "\",";
  json += "\"calFailNotice\":" + String(calFailNotice ? "true" : "false") + ",";
  json += "\"calFailSerial\":" + String(calibFailSerial) + ",";
  json += "\"calError\":" + String(calError ? "true" : "false") + ",";
  json += "\"calDefault\":" + String(calUsingDefault ? "true" : "false") + ",";
  json += "\"tMixLevel\":" + String(tMixLevel) + ",";
  json += "\"tCatLevel\":" + String(tCatLevel) + ",";
  json += "\"flowLevel\":" + String(flowLevel) + ",";
  json += "\"flowFaultType\":\"" + flowFaultType + "\",";

  json += "\"status\":\"" + status + "\",";
  json += "\"statusClass\":\"" + statusClass + "\"";

  json += "}";
  server.send(200, "application/json; charset=utf-8", json);
}

void handleSet() {
  if (server.hasArg("pwm")) {
    int v = constrain(server.arg("pwm").toInt(), 0, 99);
    if (!autoMode && !calibrating) {
      resetManualFlowControl(true); // direct PWM overrides flow target
      pwmCmd = v;
      x9cSet(pwmCmd);
    }
  }

  if (server.hasArg("mode")) {
    String m = server.arg("mode");
    if (m == "manual") {
      autoMode = false;
      manualAlarmOverride = false;
    } else if (m == "auto") {
      autoMode = true;
      manualAlarmOverride = false;
      resetManualFlowControl(true);
    }
  }

  if (server.hasArg("flow")) {
    float target = server.arg("flow").toFloat();
    if (target < 0.0f) target = 0.0f;
    if (target > FLOW_TARGET_MAX_LMIN) target = FLOW_TARGET_MAX_LMIN;
    if (!autoMode && !calibrating) {
      manualFlowTargetLmin = target;
      if (target <= FLOW_CONTROL_HYST) {
        resetManualFlowControl(false);
        pwmCmd = 0;
        x9cSet(pwmCmd);
      } else {
        manualFlowControlActive = true;
        manualFlowPriming = true;
        manualFlowTargetChanged = true;
        manualFlowPrimeStart = millis();
      }
    }
  }

  if (server.hasArg("alarm")) {
    String val = server.arg("alarm");
    if (!autoMode) {
      manualAlarmOverride = true;
      manualAlarmShouldSound = (val == "on");
      setAlarmSilenced(!manualAlarmShouldSound);
    }
  }

  if (server.hasArg("flowCalScale")) {
    float s = server.arg("flowCalScale").toFloat();
    if (s > 0.0f && s <= 10.0f) {
      flowCalScale = s;
      flowCalScaleNeedsPersist = true;
      saveCalToNVS(calFlowLmin);
    }
  }

  if (server.hasArg("mute")) {
    int m = server.arg("mute").toInt();
    if (m == 1) {
      alarmMuted = true;
      alarmMutedUntil = millis() + MUTE_MS;
    } else {
      alarmMuted = false;
      alarmMutedUntil = 0;
    }
  }

  server.send(200, "text/plain; charset=utf-8", "OK");
}

void handleLogs() {
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Pragma", "no-cache");

  if (server.hasArg("clear")) {
    clearEventLog(true);
    server.send(200, "application/json; charset=utf-8", "{\"events\":[]}");
    return;
  }

  String json = "{\"events\":[";
  for (int i = 0; i < eventLog.count; i++) {
    int idx = (EVENT_LOG_CAPACITY + eventLog.head - 1 - i) % EVENT_LOG_CAPACITY; // newest first
    const EventLogEntry &e = eventLog.entries[idx];
    json += "{\"code\":" + String((uint32_t)e.code) + ",";
    json += "\"label\":\"" + eventLabel(e.code) + "\",";
    json += "\"value\":";
    json += isLogValueNum(e.value) ? String(e.value, 2) : String("null");
    json += ",";
    json += "\"ts\":" + String((uint32_t)e.tsMs) + "}";
    if (i < eventLog.count - 1) json += ",";
  }
  json += "]}";
  server.send(200, "application/json; charset=utf-8", json);
}

// ================= CALIBRATE / RESTORE HANDLERS =================
void handleCalibrate() {
  if (autoMode) {
    server.send(400, "text/plain; charset=utf-8", "ERROR: switch to manual mode");
    return;
  }
  if (calibrating) {
    server.send(200, "text/plain; charset=utf-8", "ALREADY_RUNNING");
    return;
  }

  resetManualFlowControl(true);

  // snapshot current calibration in RAM
  for (int i = 0; i < CAL_STEPS; i++) calFlowLmin_lastGood[i] = calFlowLmin[i];
  hasLastGoodInRam = true;

  calibrating = true;
  calStepIndex = -1;
  calStepStart = 0;
  calStepAccumFlow = 0.0f;
  calStepAccumCount = 0;

  calibLastOk = true;
  calibLastMsg = "Calibrating...";
  calibFailLatched = false;
  calibFailOneShot = false;
  setAlarmSilenced(true);

  server.send(200, "text/plain; charset=utf-8", "CALIBRATION_STARTED");
}

void handleRestore() {
  // Only manual mode
  if (autoMode) {
    server.send(400, "text/plain; charset=utf-8", "ERROR: switch to manual mode");
    return;
  }
  if (calibrating) {
    server.send(400, "text/plain; charset=utf-8", "ERROR: calibrating");
    return;
  }

  resetManualFlowControl(true);

  // Write defaults to NVS ON PURPOSE
  saveCalToNVS(DEFAULT_CAL_FLOW_LMIN);

  // Load defaults into RAM
  for (int i = 0; i < CAL_STEPS; i++) calFlowLmin[i] = DEFAULT_CAL_FLOW_LMIN[i];

  // After restore: treat as “default in use” (recommended) unless you want to mark it as OK.
  // Your request: default should not be zero and shouldn’t be treated as an error.
  // We'll mark it as OK because it's valid non-zero calibration data now.
  calibrateComplete = true;
  calState = CAL_OK;
  calUsingDefault = true;
  calWarningLevel1 = false;

  // Clear faults that depend on calibration
  flowRestrictFault = false;
  flowNoMoveFault = false;
  flowUnexpectedFault = false;
  flowCalMismatchFault = false;
  flowRestrictSince = flowLowSince = flowUnexpectedSince = 0;
  flowCalSampleMs = 0;
  flowCalBadCount = 0;

  server.send(200, "text/plain; charset=utf-8", "RESTORED_DEFAULTS");
}

// ================= UPDATES =================
void updateFuelSense() {
  static uint32_t last = 0;
  if (millis() - last < 50) return;
  last = millis();
  fuelActive = (digitalRead(FUEL_SENSE) == LOW);
}

void updateFlow() {
  static uint32_t lastMs = 0;
  static uint32_t lastPulses = 0;

  uint32_t now = millis();
  if (now - lastMs < 1000) return;
  uint32_t dtMs = now - lastMs;

  uint32_t p;
  noInterrupts(); p = flowPulses; interrupts();

  uint32_t dp = p - lastPulses;
  lastPulses = p;
  lastMs = now;

  flowDpLast = dp;

  float hz = (dtMs > 0) ? (dp * 1000.0f / dtMs) : 0.0f;
  flowHz_x100 = (uint32_t)(hz * 100.0f + 0.5f);

  float qLmin = (hz / FLOW_HZ_PER_LMIN) * flowCalScale;
  if (qLmin < 0) qLmin = 0;
  if (FLOW_CLAMP_LMIN > 0 && qLmin > FLOW_CLAMP_LMIN) qLmin = FLOW_CLAMP_LMIN;

  flow_Lmin = qLmin;
  flow_Lh   = qLmin * 60.0f;
}

void updateRPM() {
  static uint32_t lastMs = 0;
  static uint32_t lastPulses = 0;

  uint32_t now = millis();
  if (now - lastMs < 1000) return;
  uint32_t dtMs = now - lastMs;

  uint32_t p;
  noInterrupts(); p = rpmPulses; interrupts();

  uint32_t dp = p - lastPulses;
  lastPulses = p;
  lastMs = now;

  rpmDpLast = dp;

  float hz = (dtMs > 0) ? (dp * 1000.0f / dtMs) : 0.0f;
  rpmHz_x100 = (uint32_t)(hz * 100.0f + 0.5f);

  float r = (RPM_PULSES_PER_REV > 0.0f) ? (hz * 60.0f / RPM_PULSES_PER_REV) : 0.0f;
  if (r < 0) r = 0;
  rpmValue = r;

  // Hall sensor fault detection:
  // - Fuel pump active AND RPM < ENGINE_RPM_MIN for > HALL_RPM_LOW_HOLD_MS
  // - RPM > HALL_RPM_HIGH_THRESHOLD immediately
  bool lowRpmWhileFuel = (fuelActive && rpmValue < ENGINE_RPM_MIN);

  if (lowRpmWhileFuel) {
    if (rpmLowFaultSince == 0) rpmLowFaultSince = now;
    if (now - rpmLowFaultSince >= HALL_RPM_LOW_HOLD_MS) hallSensorFault = true;
  } else {
    rpmLowFaultSince = 0;
  }

  if (rpmValue > HALL_RPM_HIGH_THRESHOLD) {
    hallSensorFault = true;
  } else if (!lowRpmWhileFuel) {
    hallSensorFault = false;
  }
}

void updateCalibration() {
  if (!calibrating) return;

  if (calStepIndex < 0) {
    calStepIndex = 0;
    x9cHomeMin();
    pwmCmd = 0;
    x9cSet(0);
    delay(500);

    int pwmVal = calPwmStep[calStepIndex];
    pwmCmd = pwmVal;
    x9cSet(pwmVal);

    calStepStart = millis();
    calStepAccumFlow = 0.0f;
    calStepAccumCount = 0;
    return;
  }

  uint32_t now = millis();

  if (now - calStepStart < 10000) {
    calStepAccumFlow += flow_Lmin;
    calStepAccumCount++;
    return;
  }

  float avgFlow = (calStepAccumCount > 0) ? (calStepAccumFlow / calStepAccumCount) : flow_Lmin;
  calFlowLmin[calStepIndex] = avgFlow;

  calStepIndex++;

  if (calStepIndex < CAL_STEPS) {
    int pwmVal = calPwmStep[calStepIndex];
    pwmCmd = pwmVal;
    x9cSet(pwmVal);

    calStepStart = millis();
    calStepAccumFlow = 0.0f;
    calStepAccumCount = 0;
    return;
  }

  // done: stop pump
  pwmCmd = 0;
  x9cSet(0);

  // validate before saving
  String why;
  bool ok = validateCalibrationTable(calFlowLmin, why);

  if (ok) {
    saveCalToNVS(calFlowLmin);

    calibrateComplete = true;
    calState = CAL_OK;
    calUsingDefault = tableMatchesDefault(calFlowLmin);
    calWarningLevel1 = false;

    calibLastOk = true;
    calibLastMsg = "Calibration OK (saved)";
    calibFailLatched = false;
    calibFailOneShot = false;

    for (int i = 0; i < CAL_STEPS; i++) calFlowLmin_lastGood[i] = calFlowLmin[i];
    hasLastGoodInRam = true;
  } else {
    // revert to defaults to avoid persisting invalid calibration
    for (int i = 0; i < CAL_STEPS; i++) calFlowLmin[i] = DEFAULT_CAL_FLOW_LMIN[i];
    for (int i = 0; i < CAL_STEPS; i++) calFlowLmin_lastGood[i] = calFlowLmin[i];
    hasLastGoodInRam = true;

    calibrateComplete = true;
    calState = CAL_DEFAULT_RECOMMENDED;
    calUsingDefault = true;
    calWarningLevel1 = true;

    calibLastOk = false;
    calibLastMsg = "Calibration FAILED (" + why + ") — defaults restored";
    calibFailLatched = true;
    calibFailOneShot = true;
    calibFailSerial++;
  }

  calibrating = false;
}

void updateTemps() {
  static uint32_t last = 0;
  if (millis() - last < 300) return;
  last = millis();

  tCat = max6675ReadC(MAX_CS_CAT);
  tMix = max6675ReadC(MAX_CS_MIX);
}

void updatePumpControlAndFlowChecks() {
  if (calibrating) return;

  uint32_t now = millis();
  int targetPwm = 0;
  bool engineRunning = (fuelActive && rpmValue > ENGINE_RPM_MIN);

  if (autoMode) {
    if (engineRunning) {
      float baseFlow = autoFlowTargetFromRpm(rpmValue);
      float bias = tempFlowBiasFromMix(tMix);

      float targetFlow;
      if (bias > 1e8f) {
        targetFlow = FLOW_TARGET_MAX_LMIN;
      } else {
        targetFlow = baseFlow + bias;
        targetFlow = constrain(targetFlow, 0.0f, FLOW_TARGET_MAX_LMIN);
      }

      if (!manualFlowControlActive || fabsf(targetFlow - manualFlowTargetLmin) > FLOW_CONTROL_HYST) {
        manualFlowTargetChanged = true;
      }
      manualFlowControlActive = true;
      manualFlowTargetLmin = targetFlow;

      pwmCmd = updateManualFlowControl(now);
      targetPwm = pwmCmd;
    } else {
      targetPwm = 0;
      resetManualFlowControl(true);
    }
  } else {
    if (manualFlowControlActive && manualFlowTargetLmin > FLOW_CONTROL_HYST) {
      pwmCmd = updateManualFlowControl(now);
    }
    targetPwm = pwmCmd;
  }

  int effectivePwm = autoMode ? targetPwm : pwmCmd;
  bool manualPumpShouldBeOn = (!autoMode) && manualFlowControlActive && (manualFlowTargetLmin > FLOW_CONTROL_HYST);
  bool manualDirectPwmActive = (!autoMode) && (!manualFlowControlActive) && (pwmCmd > 0);
  bool pumpShouldBeOn = autoMode ? (effectivePwm >= PUMP_PWM_MIN) : (manualPumpShouldBeOn || manualDirectPwmActive);

  // NO FLOW (pump ON but flow below threshold for long enough)
  if (pumpShouldBeOn) {
    if (pumpNonZeroSince == 0) pumpNonZeroSince = now;

    if (now - pumpNonZeroSince >= FLOW_STARTUP_GRACE_MS) {
      if (flow_Lmin < FLOW_MIN_VALID_LMIN) {
        if (flowLowSince == 0) flowLowSince = now;
        if (now - flowLowSince >= FLOW_FAULT_HOLD_MS) flowNoMoveFault = true;
      } else {
        flowLowSince = 0;
        flowNoMoveFault = false;
      }
    }
  } else {
    pumpNonZeroSince = 0;
    flowLowSince = 0;
    flowNoMoveFault = false;
  }

  // Unexpected flow (flow when pump should be OFF)
  if (!pumpShouldBeOn) {
    if (flow_Lmin > FLOW_UNEXPECTED_LMIN) {
      if (flowUnexpectedSince == 0) flowUnexpectedSince = now;
      if (now - flowUnexpectedSince >= FLOW_UNEXPECTED_HOLD_MS) flowUnexpectedFault = true;
    } else {
      flowUnexpectedSince = 0;
      flowUnexpectedFault = false;
    }
  } else {
    flowUnexpectedSince = 0;
    flowUnexpectedFault = false;
  }

  // Calibration mismatch detection (reference-only map check)
  bool calCheckEnabled = calibrateComplete && !calibrating && pumpShouldBeOn && (pwmCmd > 0);
  if (!calCheckEnabled) {
    flowCalAccum = 0.0f;
    flowCalCount = 0;
    flowCalBadCount = 0;
    flowCalSampleMs = 0;
    flowCalMismatchFault = false;
  } else {
    if (flowCalSampleMs == 0) flowCalSampleMs = now;
    // accumulate flow samples continuously
    flowCalAccum += flow_Lmin;
    flowCalCount++;

    if (now - flowCalSampleMs >= FLOW_CAL_CHECK_INTERVAL_MS) {
      float avgFlow = (flowCalCount > 0) ? (flowCalAccum / float(flowCalCount)) : flow_Lmin;
      float expFlow = expectedFlowFromCal(effectivePwm);
      bool bad = false;
      if (isNum(expFlow) && expFlow > 0.1f) {
        float diffFrac = fabsf(avgFlow - expFlow) / expFlow;
        bad = (diffFrac > FLOW_CAL_MISMATCH_TOL_FRAC);
      }
      if (bad) flowCalBadCount++; else flowCalBadCount = 0;
      flowCalMismatchFault = (flowCalBadCount >= FLOW_CAL_MISMATCH_CONSEC);

      // reset window
      flowCalAccum = 0.0f;
      flowCalCount = 0;
      flowCalSampleMs = now;
    }
  }

  // Restriction (only if calibration complete)
  if (calibrateComplete && pumpShouldBeOn && !flowNoMoveFault) {
    float expF = expectedFlowFromCal(effectivePwm);
    if (isNum(expF) && expF > 0.1f) {
      if (flow_Lmin < expF * FLOW_RESTRICT_RATIO) {
        if (flowRestrictSince == 0) flowRestrictSince = now;
        if (now - flowRestrictSince >= FLOW_RESTRICT_HOLD_MS) flowRestrictFault = true;
      } else {
        flowRestrictSince = 0;
        flowRestrictFault = false;
      }
    } else {
      flowRestrictSince = 0;
      flowRestrictFault = false;
    }
  } else {
    flowRestrictSince = 0;
    flowRestrictFault = false;
  }

  if (autoMode) {
    pwmCmd = targetPwm;
    x9cSet(pwmCmd);
  } else if (manualFlowControlActive && manualFlowTargetLmin > FLOW_CONTROL_HYST) {
    x9cSet(pwmCmd);
  }
}

void updateAlarmLogic() {
  // Calibration attention forces Level 1 if nothing else is worse
  int newLevel = 0;
  bool any = false;

  static bool prevMixWarn = false;
  static bool prevMixHigh = false;
  static bool prevMixCrit = false;
  static bool prevMixFault = false;
  static bool prevCatHigh = false;
  static bool prevCatCrit = false;
  static bool prevCatFault = false;
  static bool prevFlowNoMove = false;
  static bool prevFlowRestrict = false;
  static bool prevFlowUnexpected = false;
  static bool prevFlowCalMismatch = false;
  static bool prevCalAttention = false;
  static bool prevHallFault = false;

  bool mixWarn = isNum(tMix) && tMix >= MIX_WARN_C && tMix < MIX_HIGH_C;
  bool mixHigh = isNum(tMix) && tMix >= MIX_HIGH_C && tMix < MIX_CRIT_C;
  bool mixCrit = isNum(tMix) && tMix >= MIX_CRIT_C;
  bool mixFault = !isNum(tMix);

  bool catHigh = isNum(tCat) && tCat >= CAT_HIGH_C && tCat < CAT_CRIT_C;
  bool catCrit = isNum(tCat) && tCat >= CAT_CRIT_C;
  bool catFault = !isNum(tCat);

  // sensor faults => critical
  if (!isNum(tMix) || !isNum(tCat)) { newLevel = 3; any = true; }

  if (isNum(tMix)) {
    if (tMix >= MIX_CRIT_C) { newLevel = max(newLevel, 3); any = true; }
    else if (tMix >= MIX_HIGH_C) { newLevel = max(newLevel, 2); any = true; }
    else if (tMix >= MIX_WARN_C) { newLevel = max(newLevel, 1); any = true; }
  }

  if (isNum(tCat)) {
    if (tCat >= CAT_CRIT_C) { newLevel = max(newLevel, 3); any = true; }
    else if (tCat >= CAT_HIGH_C) { newLevel = max(newLevel, 2); any = true; }
  }

  if (flowNoMoveFault)        { newLevel = max(newLevel, 3); any = true; }
  if (flowUnexpectedFault)    { newLevel = max(newLevel, 3); any = true; }
  if (flowRestrictFault)      { newLevel = max(newLevel, 2); any = true; }
  if (flowCalMismatchFault)   { newLevel = max(newLevel, 2); any = true; }
  if (hallSensorFault)        { newLevel = max(newLevel, 3); any = true; }

  if (calWarningLevel1)       { newLevel = max(newLevel, 1); any = true; }

  alarmActive = any;
  alarmLevel = newLevel;

  recordEventEdge(mixWarn, prevMixWarn, EVENT_MIX_WARN, tMix);
  recordEventEdge(mixHigh, prevMixHigh, EVENT_MIX_HIGH, tMix);
  recordEventEdge(mixCrit, prevMixCrit, EVENT_MIX_CRIT, tMix);
  recordEventEdge(mixFault, prevMixFault, EVENT_MIX_SENSOR_FAULT, tMix);
  recordEventEdge(catHigh, prevCatHigh, EVENT_CAT_HIGH, tCat);
  recordEventEdge(catCrit, prevCatCrit, EVENT_CAT_CRIT, tCat);
  recordEventEdge(catFault, prevCatFault, EVENT_CAT_SENSOR_FAULT, tCat);

  recordEventEdge(flowNoMoveFault, prevFlowNoMove, EVENT_FLOW_NO_MOVE, flow_Lmin);
  recordEventEdge(flowRestrictFault, prevFlowRestrict, EVENT_FLOW_RESTRICT, flow_Lmin);
  recordEventEdge(flowUnexpectedFault, prevFlowUnexpected, EVENT_FLOW_UNEXPECTED, flow_Lmin);
  recordEventEdge(flowCalMismatchFault, prevFlowCalMismatch, EVENT_FLOW_CAL_MISMATCH, flow_Lmin);
  recordEventEdge(calWarningLevel1, prevCalAttention, EVENT_CAL_ATTENTION, 0.0f);
  recordEventEdge(hallSensorFault, prevHallFault, EVENT_HALL_FAULT, rpmValue);

  if (autoMode && manualAlarmOverride) {
    manualAlarmOverride = false;
  }

  if (alarmMuted && millis() > alarmMutedUntil) {
    alarmMuted = false;
    alarmMutedUntil = 0;
  }

  bool shouldSound = false;
  static uint32_t levelStartTime = 0;
  static int prevLevel = -1;

  if (alarmLevel != prevLevel) {
    prevLevel = alarmLevel;
    levelStartTime = millis();
  }

  if (alarmActive) {
    if (alarmLevel >= 3) {
      shouldSound = true; // continuous
    } else {
      uint32_t elapsed = millis() - levelStartTime;
      uint32_t cycle = elapsed % 30000UL;
      if (alarmLevel == 2) shouldSound = (cycle < 3000UL);
      if (alarmLevel == 1) shouldSound = (cycle < 1000UL);
    }
  }

  if (alarmMuted && !manualAlarmOverride) shouldSound = false;

  if (!autoMode && manualAlarmOverride) {
    shouldSound = manualAlarmShouldSound;
  }

  setAlarmSilenced(!shouldSound);
}

// ================= SETUP / LOOP =================
void setup() {
  // ---- X9C init ----
  pinMode(X9C_CS, OUTPUT);
  pinMode(X9C_UD, OUTPUT);
  pinMode(X9C_INC, OUTPUT);
  digitalWrite(X9C_CS, HIGH);
  digitalWrite(X9C_INC, HIGH);
  digitalWrite(X9C_UD, LOW);
  delay(200);
  x9cHomeMin();
  pwmCmd = 0;
  x9cSet(0);

  // ---- MAX6675 init ----
  pinMode(MAX_SCK, OUTPUT);
  pinMode(MAX_SO, INPUT);
  pinMode(MAX_CS_CAT, OUTPUT);
  pinMode(MAX_CS_MIX, OUTPUT);
  digitalWrite(MAX_SCK, LOW);
  digitalWrite(MAX_CS_CAT, HIGH);
  digitalWrite(MAX_CS_MIX, HIGH);

  // ---- Alarm relay init ----
  pinMode(RELAY_ALARM, OUTPUT);
  setAlarmSilenced(false);
  delay(600);
  setAlarmSilenced(true);

  // ---- Fuel sense ----
  pinMode(FUEL_SENSE, INPUT_PULLUP);

  // ---- Flow input ----
  pinMode(FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), isrFlow, FALLING);

  // ---- RPM Hall input ----
  pinMode(RPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), isrRpm, FALLING);

  // ---- Load stored event log ----
  loadEventLogFromNVS();

  // ---- Load calibration (READ ONLY at startup - NO WRITES) ----
  float tmp[CAL_STEPS];
  CalState st;
  bool loaded = loadCalFromNVS(tmp, st);

  if (!loaded) {
    // No valid stored calibration -> use defaults in RAM only
    for (int i = 0; i < CAL_STEPS; i++) calFlowLmin[i] = DEFAULT_CAL_FLOW_LMIN[i];

    calUsingDefault = true;
    if (st == CAL_ERROR) {
      calState = CAL_ERROR;
      calibrateComplete = false;
      calWarningLevel1 = true;
    } else {
      calState = CAL_OK;
      calibrateComplete = true;
      calWarningLevel1 = false;
    }
  } else {
    // We loaded something. It might still be “all zero” table.
    if (tableAllZero(tmp)) {
      for (int i = 0; i < CAL_STEPS; i++) calFlowLmin[i] = DEFAULT_CAL_FLOW_LMIN[i];
      calUsingDefault = true;
      calState = CAL_DEFAULT_RECOMMENDED;
      calibrateComplete = true;
      calWarningLevel1 = true;   // beep 1s/30s
    } else {
      for (int i = 0; i < CAL_STEPS; i++) calFlowLmin[i] = tmp[i];
      calUsingDefault = tableMatchesDefault(calFlowLmin);
      calState = CAL_OK;
      calibrateComplete = true;
      calWarningLevel1 = false;
    }
  }

  if (calUsingDefault && calState != CAL_ERROR && !calibFailLatched) {
    calWarningLevel1 = false;
  }

  // snapshot lastGood in RAM (so bad calibration won't wipe it)
  for (int i = 0; i < CAL_STEPS; i++) calFlowLmin_lastGood[i] = calFlowLmin[i];
  hasLastGoodInRam = true;

  if (flowCalScaleNeedsPersist) {
    saveCalToNVS(calFlowLmin);
  }

  // ---- Start AP ----
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_pass);

  // ---- Web routes ----
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/set", handleSet);
  server.on("/logs", handleLogs);
  server.on("/calibrate", handleCalibrate);
  server.on("/restore", handleRestore);
  server.begin();
}

void loop() {
  server.handleClient();

  updateFuelSense();
  updateFlow();
  updateRPM();
  updateTemps();

  updateCalibration();

  if (!calibrating) {
    updatePumpControlAndFlowChecks();
    updateAlarmLogic();
  }
}

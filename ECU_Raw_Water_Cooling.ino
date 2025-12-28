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
// Flow sensor label: F(Hz) = 5.5 * Q(L/min)  => Q = Hz / 5.5
const float FLOW_K_HZ_PER_LMIN = 5.5f;

// Hall RPM: 2 magnets on crank => 2 pulses per revolution (ONE edge counted)
const float RPM_PULSES_PER_REV = 2.0f;

// Prius engine running threshold (edit here if needed)
const int ENGINE_RPM_MIN = 800;

// ===== Pump command mapping =====
const int PUMP_PWM_MIN = 35;      // minimum command when pump should be on
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

// ===== Real "NO FLOW" detection (pump ON but flow ~0) =====
const float FLOW_MIN_VALID_LMIN = 1.0f;        // ignore tiny wobble/backflow/air
const uint32_t FLOW_STARTUP_GRACE_MS = 5000;   // wait after pump turns on
const uint32_t FLOW_FAULT_HOLD_MS   = 5000;    // must be low this long => fault

// ===== "Unexpected FLOW" detection (flow when pump/conditions say OFF) =====
const float FLOW_UNEXPECTED_LMIN = 1.5f;
const uint32_t FLOW_UNEXPECTED_HOLD_MS = 5000;

// ===== Mute time =====
const uint32_t MUTE_MS = 5UL * 60UL * 1000UL;  // 5 minutes

// ===== Calibration validation thresholds =====
const float CAL_MIN_REAL_FLOW_LMIN = 2.0f;     // at least some steps must exceed this
const float CAL_MIN_SPREAD_LMIN    = 3.0f;     // last - first must be >= this (rough sanity)
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

// Alarm
bool alarmActive = false;
int  alarmLevel  = 0;            // 0 none, 1 warn, 2 high, 3 crit
bool alarmMuted  = false;
uint32_t alarmMutedUntil = 0;
bool alarmRelaySilenced = true;  // energized = silent

// Flow faults
bool flowRestrictFault = false;
bool flowNoMoveFault   = false;
bool flowUnexpectedFault = false;

uint32_t pumpNonZeroSince = 0;
uint32_t flowLowSince = 0;
uint32_t flowRestrictSince = 0;
uint32_t flowUnexpectedSince = 0;

// Pump boost
bool boostActive = false;

// Calibration table
const int CAL_STEPS = 9;
int   calPwmStep[CAL_STEPS] = {20,30,40,50,60,70,80,90,100};
float calFlowLmin[CAL_STEPS];
bool  calibrateComplete = false;
bool  calibrating = false;
int   calStepIndex = -1;
uint32_t calStepStart = 0;
float  calStepAccumFlow = 0.0f;
uint32_t calStepAccumCount = 0;

// Keep last known-good calibration so bad calibration doesn't overwrite it
float calFlowLmin_lastGood[CAL_STEPS];
bool  hasLastGoodInRam = false;

// Calibration result reporting
bool calibLastOk = true;
String calibLastMsg = "OK";

// Default calibration table in firmware (PLACEHOLDER for now; fill tomorrow)
float DEFAULT_CAL_FLOW_LMIN[CAL_STEPS] = {
  0,0,0,0,0,0,0,0,0
};

// ================= INTERRUPTS =================
void IRAM_ATTR isrFlow() { flowPulses++; }
void IRAM_ATTR isrRpm()  { rpmPulses++; }  // FALLING only

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

  if (v & 0x0004) return NAN;
  int tempCounts = (v >> 3) & 0x0FFF;
  return tempCounts * 0.25f;
}

// ================= NC ALARM RELAY =================
void setAlarmSilenced(bool silenced) {
  digitalWrite(RELAY_ALARM, silenced ? HIGH : LOW);
  alarmRelaySilenced = silenced;
}

// ================= HELPERS =================
static inline bool isNum(float x) { return !isnan(x) && isfinite(x); }

// Interpolate expected flow from calibration
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

// ===== Calibration validation =====
bool validateCalibrationTable(const float* t, String &reason) {
  // Basic sanity
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

  // All zeros / basically no flow
  if (maxV < CAL_MIN_REAL_FLOW_LMIN) {
    reason = "No real flow measured";
    return false;
  }

  // Not enough points above "real flow" threshold
  if (goodPts < CAL_MIN_GOOD_POINTS) {
    reason = "Too few valid points";
    return false;
  }

  // Must have some spread (pump not responding / sensor dead gives flat line)
  if ((maxV - minV) < CAL_MIN_SPREAD_LMIN) {
    reason = "Too flat (pump/sensor?)";
    return false;
  }

  // Mostly increasing (allow small noise)
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

// ===== NVS Calibration Handling =====
void saveCalToNVS(const float* table) {
  prefs.begin("ECU", false);
  prefs.putUInt("hasCal", 1);
  for (int i = 0; i < CAL_STEPS; i++) {
    char key[8];
    sprintf(key, "cal%d", calPwmStep[i]);
    prefs.putFloat(key, table[i]);
  }
  prefs.end();
}

bool loadCalFromNVS(float* outTable) {
  prefs.begin("ECU", true);
  uint32_t has = prefs.getUInt("hasCal", 0);
  if (has != 1) {
    prefs.end();
    return false;
  }
  for (int i = 0; i < CAL_STEPS; i++) {
    char key[8];
    sprintf(key, "cal%d", calPwmStep[i]);
    outTable[i] = prefs.getFloat(key, 0.0f);
  }
  prefs.end();
  return true;
}

void applyDefaultCalAndSave() {
  for (int i = 0; i < CAL_STEPS; i++) calFlowLmin[i] = DEFAULT_CAL_FLOW_LMIN[i];
  saveCalToNVS(DEFAULT_CAL_FLOW_LMIN);

  // If defaults are all zeros (like now), don't treat as valid for restriction logic.
  // After tomorrow, when defaults are real, you can set this to true if you want.
  calibrateComplete = false;

  calibLastOk = true;
  calibLastMsg = "Restored defaults";
}

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
    button { padding: 8px 12px; border-radius: 10px; border: 1px solid #aaa; background: #fafafa; }
    button:disabled { opacity: 0.55; }
    input[type=range] { height: 32px; }
  </style>
</head>
<body>
  <h2>ECU Debug (AP)</h2>

  <div class="card">
    <div class="row big">
      <div>Status</div>
      <div id="statusMsg" class="ok">OK</div>
    </div>
    <div class="row small">
      <div>Mode</div>
      <div>
        <label>
          Auto <input type="checkbox" id="modeSwitch"> Manual
        </label>
      </div>
    </div>
    <div class="row small">
      <div>Calibration</div>
      <div id="calStatus" class="mono">OK</div>
    </div>
  </div>

  <div class="card">
    <div class="row big"><div>Fuel</div><div id="fuel">?</div></div>

    <div class="row big">
      <div>RPM</div>
      <div class="mono">
        <span id="rpm">?</span>
        <span class="small">(<span id="rpmhz">?</span> Hz, dp=<span id="rpmdp">?</span>)</span>
      </div>
    </div>

    <div class="row big">
      <div>Flow</div>
      <div class="mono">
        <span id="flow">?</span> L/min (<span id="flowh">?</span> L/h)
        <span class="small">- <span id="flowhz">?</span> Hz, dp=<span id="flowdp">?</span></span>
      </div>
    </div>

    <div class="row big"><div>Tcat</div><div class="mono"><span id="tcat">?</span> &deg;C</div></div>
    <div class="row big"><div>Tmix</div><div class="mono"><span id="tmix">?</span> &deg;C</div></div>

    <div class="row big"><div>PWM (cmd)</div><div class="mono" id="pwm">?</div></div>
    <div class="row big"><div>X9C (pos)</div><div class="mono" id="x9c">?</div></div>

    <div class="row big"><div>Alarm</div><div class="mono" id="alarm">?</div></div>

    <div class="row small">
      <div>Cal Table</div>
      <div class="mono" id="caltable">Not calibrated</div>
    </div>
  </div>

  <div class="card">
    <div class="big">Manual PWM: <span id="sval">0</span></div>
    <input type="range" min="0" max="99" value="0" id="slider" style="width:100%;" oninput="setPWM(this.value)">

    <div style="margin: 12px 0 16px 0;">
      <button id="calibBtn" onclick="startCalibration()">Calibrate</button>
    </div>

    <div style="margin: 8px 0 16px 0;">
      <button id="restoreBtn" onclick="restoreDefaults()">Restore Defaults</button>
    </div>

    <div style="margin: 8px 0;">
      <button id="muteBtn" onclick="toggleMute()">Mute Alarm</button>
      <button id="alarmToggleBtn" onclick="toggleAlarm()" style="display:none;">Toggle Alarm</button>
    </div>

    <div class="small" style="margin-top:8px;">IP: <code>192.168.4.1</code></div>
  </div>

<script>
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
    alarm.textContent = d.alarm ? "ON" : "off";

    statusMsg.textContent = d.status || "OK";
    statusMsg.className = "";
    statusMsg.classList.add(d.statusClass || "ok");

    calStatus.textContent = d.calMsg || "OK";
    calStatus.className = "mono " + (d.calOk ? "ok" : "error");

    if (d.errMix) tmix.classList.add("error"); else tmix.classList.remove("error");
    if (d.errCat) tcat.classList.add("error"); else tcat.classList.remove("error");

    if (d.errFlow || d.errNoMove || d.errUnexpected) {
      flow.classList.add("error");
      flowh.classList.add("error");
    } else {
      flow.classList.remove("error");
      flowh.classList.remove("error");
    }

    modeSwitch.checked = d.autoMode ? false : true;

    const manual = !d.autoMode;

    slider.disabled = !manual;
    calibBtn.disabled = !manual || d.calibrating;
    restoreBtn.disabled = d.calibrating;

    if (!manual) {
      slider.classList.add("muted");
      calibBtn.classList.add("muted");
    } else {
      slider.classList.remove("muted");
      calibBtn.classList.remove("muted");
    }

    if (manual) {
      alarmToggleBtn.style.display = "inline";
      alarmToggleBtn.textContent = (d.alarmRelaySilenced ? "Sound Alarm" : "Silence Alarm");
    } else {
      alarmToggleBtn.style.display = "none";
    }

    sval.textContent  = d.pwm;
    slider.value      = d.pwm;

    if (d.tableOk && Array.isArray(d.cal) && d.cal.length === 9) {
      const steps = [20,30,40,50,60,70,80,90,100];
      let s = "";
      for (let i=0;i<steps.length;i++){
        s += steps[i] + "%=" + Number(d.cal[i]).toFixed(1) + " ";
      }
      caltable.textContent = s.trim();
    } else {
      caltable.textContent = "Not calibrated";
    }

    muteBtn.disabled = !d.alarm;
    muteBtn.textContent = d.alarmMuted ? "Unmute Alarm" : "Mute Alarm";

    calibBtn.textContent = d.calibrating ? "Calibrating..." : "Calibrate";

  }catch(e){}
}

async function setPWM(v){
  sval.textContent = v;
  await fetch('/set?pwm=' + v);
}

modeSwitch.addEventListener('change', async function() {
  if (this.checked) await fetch('/set?mode=manual');
  else              await fetch('/set?mode=auto');
});

async function startCalibration(){
  if (!confirm("Start calibration?\nMake sure discharge hose is in a bucket.")) return;
  if (!confirm("Are you sure? Pump will run through speeds.")) return;
  await fetch('/calibrate');
}

async function restoreDefaults(){
  if (!confirm("Restore default calibration table from firmware?\nThis overwrites saved calibration.")) return;
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

void handleData() {
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Pragma", "no-cache");

  float flm = flow_Lmin;
  float flh = flow_Lh;
  float rv  = rpmValue;

  float fHz = flowHz_x100 / 100.0f;
  float rHz = rpmHz_x100  / 100.0f;

  String status = "OK";
  String statusClass = "ok";

  if (alarmActive) {
    statusClass = (alarmLevel >= 3) ? "error" : "warn";
    String msg = "";

    if (!isNum(tMix)) msg += "Mixer sensor FAULT; ";
    if (!isNum(tCat)) msg += "Cat sensor FAULT; ";

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

    if (msg.endsWith("; ")) msg.remove(msg.length() - 2);
    status = (alarmLevel >= 3 ? "CRITICAL: " : "WARNING: ") + msg;
  }

  String json = "{";
  json += "\"fuel\":" + String(fuelActive ? "true" : "false") + ",";

  json += "\"rpm\":"    + String((int)(rv + 0.5f)) + ",";
  json += "\"rpmHz\":"  + String(rHz, 2) + ",";
  json += "\"rpmDp\":"  + String((uint32_t)rpmDpLast) + ",";

  json += "\"flowLmin\":" + String(flm, 3) + ",";
  json += "\"flowLh\":"   + String(flh, 1) + ",";
  json += "\"flowHz\":"   + String(fHz, 2) + ",";
  json += "\"flowDp\":"   + String((uint32_t)flowDpLast) + ",";

  json += "\"tCat\":" + (isNum(tCat) ? String(tCat, 1) : String("null")) + ",";
  json += "\"tMix\":" + (isNum(tMix) ? String(tMix, 1) : String("null")) + ",";

  json += "\"pwm\":"  + String(pwmCmd) + ",";
  json += "\"x9c\":"  + String(x9cPos) + ",";

  json += "\"alarm\":" + String(alarmActive ? "true" : "false") + ",";
  json += "\"alarmMuted\":" + String(alarmMuted ? "true" : "false") + ",";
  json += "\"alarmRelaySilenced\":" + String(alarmRelaySilenced ? "true" : "false") + ",";
  json += "\"autoMode\":" + String(autoMode ? "true" : "false") + ",";
  json += "\"calibrating\":" + String(calibrating ? "true" : "false") + ",";

  bool errMix = (!isNum(tMix) || (isNum(tMix) && tMix >= MIX_WARN_C));
  bool errCat = (!isNum(tCat) || (isNum(tCat) && tCat >= CAT_HIGH_C));
  json += "\"errMix\":" + String(errMix ? "true" : "false") + ",";
  json += "\"errCat\":" + String(errCat ? "true" : "false") + ",";
  json += "\"errFlow\":" + String(flowRestrictFault ? "true" : "false") + ",";
  json += "\"errNoMove\":" + String(flowNoMoveFault ? "true" : "false") + ",";
  json += "\"errUnexpected\":" + String(flowUnexpectedFault ? "true" : "false") + ",";

  json += "\"tableOk\":" + String(calibrateComplete ? "true" : "false") + ",";
  json += "\"cal\":[";
  for (int i = 0; i < CAL_STEPS; i++) {
    json += String(calFlowLmin[i], 2);
    if (i < CAL_STEPS - 1) json += ",";
  }
  json += "],";

  json += "\"calOk\":" + String(calibLastOk ? "true" : "false") + ",";
  json += "\"calMsg\":\"" + calibLastMsg + "\",";

  json += "\"status\":\"" + status + "\",";
  json += "\"statusClass\":\"" + statusClass + "\"";

  json += "}";
  server.send(200, "application/json; charset=utf-8", json);
}

void handleSet() {
  if (server.hasArg("pwm")) {
    int v = constrain(server.arg("pwm").toInt(), 0, 99);
    if (!autoMode && !calibrating) {
      pwmCmd = v;
      x9cSet(pwmCmd);
    }
  }

  if (server.hasArg("mode")) {
    String m = server.arg("mode");
    if (m == "manual") {
      autoMode = false;
      setAlarmSilenced(true);
    } else if (m == "auto") {
      autoMode = true;
    }
  }

  if (server.hasArg("alarm")) {
    String val = server.arg("alarm");
    if (!autoMode) {
      if (val == "on") setAlarmSilenced(false);
      else            setAlarmSilenced(true);
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

void handleCalibrate() {
  if (autoMode) {
    server.send(400, "text/plain; charset=utf-8", "ERROR: switch to manual mode");
    return;
  }
  if (calibrating) {
    server.send(200, "text/plain; charset=utf-8", "ALREADY_RUNNING");
    return;
  }

  // snapshot current known-good calibration into RAM
  for (int i = 0; i < CAL_STEPS; i++) calFlowLmin_lastGood[i] = calFlowLmin[i];
  hasLastGoodInRam = true;

  calibrating = true;
  calStepIndex = -1;
  calStepStart = 0;
  calStepAccumFlow = 0.0f;
  calStepAccumCount = 0;

  calibLastOk = true;
  calibLastMsg = "Calibrating...";

  setAlarmSilenced(true);

  server.send(200, "text/plain; charset=utf-8", "CALIBRATION_STARTED");
}

void handleRestore() {
  if (calibrating) {
    server.send(400, "text/plain; charset=utf-8", "ERROR: calibrating");
    return;
  }
  applyDefaultCalAndSave();

  flowRestrictFault = false;
  flowNoMoveFault = false;
  flowUnexpectedFault = false;
  flowRestrictSince = 0;
  flowLowSince = 0;
  flowUnexpectedSince = 0;

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

  float qLmin = hz / FLOW_K_HZ_PER_LMIN;
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

  // done: stop pump first
  pwmCmd = 0;
  x9cSet(0);

  // validate before saving
  String why;
  bool ok = validateCalibrationTable(calFlowLmin, why);

  if (ok) {
    calibrateComplete = true;
    saveCalToNVS(calFlowLmin);
    calibLastOk = true;
    calibLastMsg = "Calibration OK (saved)";
    // refresh lastGood snapshot
    for (int i = 0; i < CAL_STEPS; i++) calFlowLmin_lastGood[i] = calFlowLmin[i];
    hasLastGoodInRam = true;
  } else {
    // revert to last good table in RAM (if we have it)
    if (hasLastGoodInRam) {
      for (int i = 0; i < CAL_STEPS; i++) calFlowLmin[i] = calFlowLmin_lastGood[i];
    }
    calibLastOk = false;
    calibLastMsg = "Calibration FAILED (" + why + ") kept old";
    // keep calibrateComplete as-is (do NOT force true)
  }

  calibrating = false;
}

void updateTempsAndAlarm() {
  static uint32_t last = 0;
  if (millis() - last < 300) return;
  last = millis();

  tCat = max6675ReadC(MAX_CS_CAT);
  tMix = max6675ReadC(MAX_CS_MIX);

  int newLevel = 0;
  bool any = false;

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

  if (flowNoMoveFault) { newLevel = max(newLevel, 3); any = true; }
  if (flowUnexpectedFault) { newLevel = max(newLevel, 2); any = true; }
  if (flowRestrictFault) { newLevel = max(newLevel, 1); any = true; }

  alarmActive = any;
  alarmLevel = newLevel;

  if (!autoMode) return;

  if (alarmMuted && millis() > alarmMutedUntil) {
    alarmMuted = false;
    alarmMutedUntil = 0;
  }

  if (!alarmActive) {
    setAlarmSilenced(true);
    return;
  }

  bool shouldSound = false;
  static uint32_t levelStartTime = 0;
  static int prevLevel = -1;
  if (alarmLevel != prevLevel) {
    prevLevel = alarmLevel;
    levelStartTime = millis();
  }

  if (alarmLevel >= 3) {
    shouldSound = true;
  } else {
    uint32_t elapsed = millis() - levelStartTime;
    uint32_t cycle = elapsed % 30000UL;
    if (alarmLevel == 2) shouldSound = (cycle < 3000UL);
    if (alarmLevel == 1) shouldSound = (cycle < 1000UL);
  }

  if (alarmMuted) shouldSound = false;

  setAlarmSilenced(!shouldSound);
}

void updatePumpControlAndFlowChecks() {
  if (!autoMode || calibrating) return;

  int targetPwm = 0;

  bool engineRunning = (fuelActive && rpmValue > ENGINE_RPM_MIN);

  if (engineRunning) {
    float rpmClamped = rpmValue;
    if (rpmClamped > PUMP_RPM_MAX) rpmClamped = PUMP_RPM_MAX;
    if (rpmClamped < ENGINE_RPM_MIN) rpmClamped = ENGINE_RPM_MIN;

    float fraction = (rpmClamped - ENGINE_RPM_MIN) / float(PUMP_RPM_MAX - ENGINE_RPM_MIN);
    float basePwmF = PUMP_PWM_MIN + fraction * float(PUMP_PWM_MAX - PUMP_PWM_MIN);
    if (basePwmF > PUMP_PWM_MAX) basePwmF = PUMP_PWM_MAX;
    if (basePwmF < 0) basePwmF = 0;
    targetPwm = int(basePwmF + 0.5f);

    if (!boostActive && isNum(tMix) && tMix >= MIX_HIGH_C) boostActive = true;
    if (boostActive && isNum(tMix) && tMix <= (MIX_HIGH_C - 5.0f)) boostActive = false;

    if (isNum(tMix) && tMix >= MIX_CRIT_C) targetPwm = 99;
    else if (boostActive) targetPwm = min(99, targetPwm + 10);
  } else {
    targetPwm = 0;
    boostActive = false;
  }

  uint32_t now = millis();
  bool pumpShouldBeOn = (targetPwm >= PUMP_PWM_MIN);

  // NO FLOW
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

  // Unexpected flow
  bool shouldHaveFlow = pumpShouldBeOn;
  if (!shouldHaveFlow) {
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

  // Restriction (only when we have a valid table)
  if (calibrateComplete && pumpShouldBeOn && !flowNoMoveFault) {
    float expF = expectedFlowFromCal(targetPwm);
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

  pwmCmd = targetPwm;
  x9cSet(pwmCmd);
}

// ================= SETUP / LOOP =================
void setup() {
  // X9C
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

  // MAX6675
  pinMode(MAX_SCK, OUTPUT);
  pinMode(MAX_SO, INPUT);
  pinMode(MAX_CS_CAT, OUTPUT);
  pinMode(MAX_CS_MIX, OUTPUT);
  digitalWrite(MAX_SCK, LOW);
  digitalWrite(MAX_CS_CAT, HIGH);
  digitalWrite(MAX_CS_MIX, HIGH);

  // Alarm relay
  pinMode(RELAY_ALARM, OUTPUT);
  setAlarmSilenced(false);
  delay(600);
  setAlarmSilenced(true);

  // Fuel
  pinMode(FUEL_SENSE, INPUT_PULLUP);

  // Flow
  pinMode(FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), isrFlow, FALLING);

  // RPM (fix: only one edge)
  pinMode(RPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), isrRpm, FALLING);

  // Load calibration (or seed defaults)
  float tmp[CAL_STEPS];
  bool hasNvs = loadCalFromNVS(tmp);

  if (!hasNvs) {
    applyDefaultCalAndSave(); // seeds defaults to NVS
  } else {
    for (int i = 0; i < CAL_STEPS; i++) calFlowLmin[i] = tmp[i];
    calibrateComplete = true;

    // set lastGood snapshot
    for (int i = 0; i < CAL_STEPS; i++) calFlowLmin_lastGood[i] = calFlowLmin[i];
    hasLastGoodInRam = true;
  }

  // Start AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_pass);

  // Web routes
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/set", handleSet);
  server.on("/calibrate", handleCalibrate);
  server.on("/restore", handleRestore);
  server.begin();
}

void loop() {
  server.handleClient();

  updateFuelSense();
  updateFlow();
  updateRPM();
  updateCalibration();

  if (!calibrating) {
    updatePumpControlAndFlowChecks();
    updateTempsAndAlarm();
  }
}

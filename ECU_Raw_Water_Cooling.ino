#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// ================= PIN MAP (user's map) =================
#define FLOW_PIN     0    // GPIO0  -> Flow sensor
#define RPM_PIN      1    // GPIO1  -> RPM Hall sensor
#define FUEL_SENSE   2    // GPIO2  -> Fuel pump active (PC817), INPUT_PULLUP, LOW = ON

#define MAX_CS_CAT   3    // GPIO3  -> MAX6675 #1 CS (CAT)
#define MAX_CS_MIX   4    // GPIO4  -> MAX6675 #2 CS (MIXER)
#define MAX_SO       5    // GPIO5  -> MAX6675 SO (shared)
#define MAX_SCK      6    // GPIO6  -> MAX6675 SCK (shared)

#define X9C_UD       7    // GPIO7  -> X9C U/D (digital potentiometer control)
#define X9C_CS       8    // GPIO8  -> X9C CS  (chip select)
#define X9C_INC      9    // GPIO9  -> X9C INC (increment)

#define RELAY_ALARM  10   // GPIO10 -> Alarm relay control (NC relay strategy)

// ================= USER CALIBRATION =================
// Flow sensor calibration: F(Hz) = 5.5 * Q(L/min) => Q = Hz / 5.5
const float FLOW_K_HZ_PER_LMIN = 5.5f; 

// Hall RPM: 2 magnets on crank => 2 pulses per revolution
const float RPM_PULSES_PER_REV = 2.0f;

// Temperature alarm thresholds (Mixer in °C)
const float MIX_ALARM_ON_C   = 50.0f;  // Warning threshold for mixer temperature
const float MIX_ALARM_HIGH_C = 60.0f;  // High temperature threshold
const float MIX_ALARM_CRIT_C = 70.0f;  // Critical temperature threshold

// Temperature alarm threshold (Catalytic Converter in °C)
const float CAT_ALARM_HIGH_C = 150.0f; // High temperature threshold for cat
const float CAT_ALARM_CRIT_C = 180.0f; // Critical temperature threshold

// Flow reading clamp (to ignore unrealistic spikes from air or noise)
const float FLOW_CLAMP_LMIN = 120.0f;  // If flow reading exceeds this L/min, clamp it (0 to disable)

// Engine RPM threshold to consider engine "running" (for pumping water). 
// Based on Prius idle ~960-1000 RPM:contentReference[oaicite:3]{index=3}:contentReference[oaicite:4]{index=4}, we choose a safe threshold slightly lower.
const int ENGINE_RPM_MIN = 800;  

// ================= WIFI AP (Debug) =================
const char* ap_ssid = "ECU_DEBUG";
const char* ap_pass = "12345678";

WebServer server(80);
Preferences prefs; // for storing calibration data

// ================= ECU STATE (global variables) =================
volatile bool  fuelActive  = false;   // True if fuel pump is on (engine likely running)
volatile bool  alarmActive = false;   // True if any alarm condition is active (not necessarily sounding if muted)

 // Mode and control
bool autoMode = true;          // true = automatic control mode, false = manual mode
bool alarmMuted = false;       // true if alarm has been muted by user
uint32_t alarmMutedUntil = 0;  // timestamp when mute period ends (ms)

volatile int   pwmCmd = 0;     // Pump speed command (0..99)
int x9cPos = 0;                // Current position of X9C digital pot (0..99)

// Temperature readings
float tCat = NAN;
float tMix = NAN;

// Flow measurement accumulators
volatile uint32_t flowPulses = 0;
volatile float    flow_Lmin  = 0.0f;
volatile float    flow_Lh    = 0.0f;
volatile uint32_t flowHz_x100 = 0;   // Flow frequency (Hz * 100) for display
volatile uint32_t flowDpLast = 0;    // Pulses counted in last measurement window

// RPM measurement accumulators
volatile uint32_t rpmPulses = 0;
volatile float    rpmValue  = 0.0f;
volatile uint32_t rpmHz_x100 = 0;    // RPM hall frequency (Hz * 100) for display
volatile uint32_t rpmDpLast = 0;     // Pulses counted in last measurement window

// Calibration data for flow vs PWM
const int CAL_STEPS = 9;              // number of calibration points (20%,30%,...100%)
int calPwmStep[CAL_STEPS] = {20,30,40,50,60,70,80,90,100}; // PWM values to calibrate at
float calFlowLmin[CAL_STEPS];        // measured flow (L/min) at those PWM values
bool calibrateComplete = false;      // whether calibration data is available
bool flowLowCondition = false;      // flag for low flow condition detection
bool calibrating = false;            // whether calibration process is active
int calStepIndex = -1;               // current step index during calibration
uint32_t calStepStart = 0;           // timestamp when current calibration step started
float calStepAccumFlow = 0.0f;       // accumulate flow for averaging (if needed)
uint32_t calStepAccumCount = 0;      // count of flow measurements in step (optional)

// Alarm and control state
int alarmLevel = 0;          // 0 = no alarm, 1 = warning, 2 = high, 3 = critical (used for patterns)
bool alarmRelayState = true; // current relay state (true = silenced/energized, false = sounding/de-energized)
bool boostActive = false;    // whether we have applied extra pump boost due to high temperature

// ================= INTERRUPT SERVICE ROUTINES =================
void IRAM_ATTR isrFlow() { 
  // Increment flow pulse count on falling edge
  flowPulses++; 
}
void IRAM_ATTR isrRpm() { 
  // Increment RPM pulse count on falling edge
  rpmPulses++; 
}

// ================= DIGITAL POT (X9C) CONTROL =================
// Functions to control X9C digital potentiometer (for PWM output control)
void x9cPulse() {
  digitalWrite(X9C_INC, LOW);  delayMicroseconds(2);
  digitalWrite(X9C_INC, HIGH); delayMicroseconds(2);
}
void x9cStep(bool up, int steps) {
  if (steps <= 0) return;
  digitalWrite(X9C_CS, LOW);
  digitalWrite(X9C_UD, up ? HIGH : LOW);
  delayMicroseconds(2);
  for (int i = 0; i < steps; i++) {
    x9cPulse();
  }
  digitalWrite(X9C_CS, HIGH);
  delay(5); // latch/store new value
}
void x9cHomeMin() {
  // Drive the pot to the minimum position (0)
  digitalWrite(X9C_CS, LOW);
  digitalWrite(X9C_UD, LOW);
  delayMicroseconds(2);
  for (int i = 0; i < 140; i++) {
    x9cPulse(); // overshoot pulses to ensure reaching min
  }
  digitalWrite(X9C_CS, HIGH);
  delay(10);
  x9cPos = 0;
}
void x9cSet(int target) {
  // Set the digital pot to a target position 0-99
  target = constrain(target, 0, 99);
  if (target == x9cPos) return;
  if (target > x9cPos) {
    x9cStep(true, target - x9cPos);
  } else {
    x9cStep(false, x9cPos - target);
  }
  x9cPos = target;
}

// ================= MAX6675 THERMOCOUPLE READ =================
float max6675ReadC(uint8_t csPin) {
  // Read 16-bit data from MAX6675 and convert to temperature in °C
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

  if (v & 0x0004) {
    // Bit 2 indicates thermocouple open circuit
    return NAN;
  }
  int tempCounts = (v >> 3) & 0x0FFF;   // get the 12-bit temperature data
  return tempCounts * 0.25f;
}

// ================= ALARM RELAY CONTROL (NC STRATEGY) =================
// NC (normally-closed) relay: alarm sounds when relay is NOT energized. 
// To silence alarm, we energize the relay.
void setAlarmSilenced(bool silenced) {
  // For most relay modules HIGH = energized; if yours is active-LOW, invert this logic.
  digitalWrite(RELAY_ALARM, silenced ? HIGH : LOW);
  alarmRelayState = silenced;
}

// ================= WEB UI PAGES =================
String pageHTML() {
  // Returns the HTML string for the web interface page
  return R"HTML(
<!doctype html><html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ECU Debug</title>
  <style>
    body { font-family: sans-serif; margin: 16px; }
    .card { padding: 12px; border: 1px solid #ccc; border-radius: 10px; margin: 10px 0; }
    .big { font-size: 1.1em; }
    .row { display:flex; justify-content:space-between; padding: 6px 0; gap: 12px; }
    code { background:#f3f3f3; padding:2px 6px; border-radius:6px; }
    .small { font-size: 0.9em; opacity: 0.75; }
    .mono { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace; }
    .error { color: red; font-weight: bold; }
    .disabled { opacity: 0.5; }
  </style>
</head>
<body>
  <h2>ECU Debug (AP)</h2>

  <!-- Status / alerts card -->
  <div class="card">
    <div class="big" id="statusMsg" style="color: green;">Status: OK</div>
  </div>

  <!-- Sensor readings card -->
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

    <div class="row big">
      <div>Tcat</div><div class="mono"><span id="tcat">?</span> &deg;C</div>
    </div>
    <div class="row big">
      <div>Tmix</div><div class="mono"><span id="tmix">?</span> &deg;C</div>
    </div>

    <div class="row big"><div>PWM (cmd)</div><div class="mono" id="pwm">?</div></div>
    <div class="row big"><div>X9C (pos)</div><div class="mono" id="x9c">?</div></div>
    <div class="row big"><div>Alarm</div><div class="mono" id="alarm">?</div></div>
  </div>

  <!-- Controls card -->
  <div class="card">
    <div class="row big">
      <div>Mode</div>
      <div>
        <label style="font-weight:normal;">
          Auto <input type="checkbox" id="modeSwitch"> Manual
        </label>
      </div>
    </div>
    <div class="big">Pump PWM: <span id="sval">0</span></div>
    <input type="range" min="0" max="99" value="0" id="slider" style="width:100%;" oninput="setPWM(this.value)">
    <div style="margin: 8px 0;">
      <button id="calibBtn" onclick="startCalibration()">Calibrate</button>
      <button id="muteBtn" onclick="toggleMute()">Mute Alarm</button>
      <button id="alarmToggleBtn" onclick="toggleAlarm()" style="display:none;">Toggle Alarm</button>
    </div>
    <div class="small" style="margin-top:4px;">IP: <code>192.168.4.1</code></div>
  </div>

<script>
async function refresh(){
  try {
    const r = await fetch('/data', {cache: "no-store"});
    const d = await r.json();
    // Update sensor and status fields
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

    // Temperature fields: if null (sensor fault), show "FAULT"
    tcat.textContent  = (typeof d.tCat === "number" ? d.tCat.toFixed(1) : "FAULT");
    tmix.textContent  = (typeof d.tMix === "number" ? d.tMix.toFixed(1) : "FAULT");

    pwm.textContent   = d.pwm;
    x9c.textContent   = d.x9c;
    alarm.textContent = d.alarm ? "ON" : "off";

    // Update status message and styling
    statusMsg.textContent = d.status;
    if (d.status && d.status.startsWith("Status: OK")) {
      statusMsg.style.color = "green";
    } else {
      statusMsg.style.color = "red";
    }

    // Highlight or clear parameter fields if they are in error
    if (d.errMix) { tmix.classList.add("error"); } else { tmix.classList.remove("error"); }
    if (d.errCat) { tcat.classList.add("error"); } else { tcat.classList.remove("error"); }
    if (d.errFlow) {
      flow.classList.add("error");
      flowh.classList.add("error");
    } else {
      flow.classList.remove("error");
      flowh.classList.remove("error");
    }
    // (RPM and Fuel are not highlighted in current logic)

    // Update mode switch UI
    modeSwitch.checked = d.autoMode ? false : true; // checked means Manual mode
    // Gray out controls when in auto mode
    if (d.autoMode) {
      slider.disabled = true;
      slider.classList.add("disabled");
      calibBtn.disabled = true;
      calibBtn.classList.add("disabled");
      // In auto mode, manual alarm toggle is hidden (use mute instead)
      alarmToggleBtn.style.display = "none";
    } else {
      slider.disabled = false;
      slider.classList.remove("disabled");
      calibBtn.disabled = false;
      calibBtn.classList.remove("disabled");
      // In manual mode, show alarm toggle button
      alarmToggleBtn.style.display = "inline";
      alarmToggleBtn.textContent = (d.alarmRelay ? "Sound Alarm" : "Silence Alarm");
    }

    // Update slider and manual value display to current PWM command
    sval.textContent  = d.pwm;
    slider.value      = d.pwm;

    // Mute button state update
    if (d.alarmMuted) {
      // If currently muted, show "Unmute Alarm"
      muteBtn.textContent = "Unmute Alarm";
    } else {
      muteBtn.textContent = "Mute Alarm";
    }
    // Enable mute button only if an alarm condition is present
    muteBtn.disabled = !d.alarm;
    if (muteBtn.disabled) {
      muteBtn.classList.add("disabled");
    } else {
      muteBtn.classList.remove("disabled");
    }

  } catch(e) {
    console.log("Refresh error:", e);
  }
}

async function setPWM(v){
  // Send manual PWM command (only works in manual mode)
  sval.textContent = v;
  await fetch('/set?pwm=' + v);
}

// Toggle between auto and manual mode
modeSwitch.addEventListener('change', async function() {
  if (this.checked) {
    // switched to Manual mode
    await fetch('/set?mode=manual');
  } else {
    // switched to Auto mode
    await fetch('/set?mode=auto');
  }
});

// Trigger calibration process (with confirmation)
async function startCalibration(){
  if (!confirm("Start calibration?\nMake sure discharge hose is in a bucket.")) {
    return;
  }
  if (!confirm("Are you sure? The pump will run at various speeds.")) {
    return;
  }
  // Send calibration command
  await fetch('/calibrate');
  // After sending, immediately disable calibrate button and inform user
  calibBtn.disabled = true;
  calibBtn.textContent = "Calibrating...";
}

// Mute/unmute alarm
async function toggleMute(){
  // If currently muted, unmute; if not muted, mute for 5 minutes.
  const action = (muteBtn.textContent.indexOf("Unmute") !== -1) ? 0 : 1;
  await fetch('/set?mute=' + action);
}

// Manual alarm relay toggle (only in manual mode)
async function toggleAlarm(){
  // Toggles the alarm relay state in manual mode
  const currentText = alarmToggleBtn.textContent;
  const activate = currentText.indexOf("Sound") !== -1; 
  // If button says "Sound Alarm", we want to activate alarm (relay off)
  await fetch('/set?alarm=' + (activate ? 'on' : 'off'));
}

setInterval(refresh, 500);
refresh();
</script>
</body></html>
)HTML";
}

// Handle root URL: serve the main page
void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Pragma", "no-cache");
  server.send(200, "text/html; charset=utf-8", pageHTML());
}

// Handle /data request: send JSON with current sensor readings and status
void handleData() {
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Pragma", "no-cache");

  // Copy volatile counts with interrupts off for consistency
  noInterrupts();
  uint32_t flowPulsesCopy = flowPulses;
  uint32_t rpmPulsesCopy = rpmPulses;
  interrupts();

  // We use the computed values which are updated in updateFlow/updateRPM
  float flm = flow_Lmin;
  float flh = flow_Lh;
  float rv  = rpmValue;
  float fHz = flowHz_x100 / 100.0f;
  float rHz = rpmHz_x100  / 100.0f;

  String json = "{";
  // Fuel status
  json += "\"fuel\":" + String(fuelActive ? "true" : "false") + ",";

  // RPM values
  json += "\"rpm\":"    + String((int)(rv + 0.5f)) + ",";  // integer RPM (rounded)
  json += "\"rpmHz\":"  + String(rHz, 2) + ","; 
  json += "\"rpmDp\":"  + String((uint32_t)rpmDpLast) + ",";

  // Flow values
  json += "\"flowLmin\":" + String(flm, 3) + ","; 
  json += "\"flowLh\":"   + String(flh, 1) + ","; 
  json += "\"flowHz\":"   + String(fHz, 2) + ","; 
  json += "\"flowDp\":"   + String((uint32_t)flowDpLast) + ",";

  // Temperatures (null if fault)
  if (isnan(tCat)) json += "\"tCat\":null,";
  else json += "\"tCat\":" + String(tCat, 1) + ",";
  if (isnan(tMix)) json += "\"tMix\":null,";
  else json += "\"tMix\":" + String(tMix, 1) + ",";

  // PWM, X9C positions
  json += "\"pwm\":"  + String(pwmCmd) + ",";
  json += "\"x9c\":"  + String(x9cPos) + ",";

  // Alarm status flags
  json += "\"alarm\":" + String(alarmActive ? "true" : "false") + ",";
  json += "\"alarmRelay\":" + String(alarmRelayState ? "true" : "false") + ",";  // true if relay energized (silenced)
  json += "\"autoMode\":" + String(autoMode ? "true" : "false") + ",";
  json += "\"alarmMuted\":" + String(alarmMuted ? "true" : "false") + ",";

  // Error flags for highlighting fields
  bool errMixFlag = false;
  bool errCatFlag = false;
  bool errFlowFlag = false;
  if (isnan(tMix) || tMix >= MIX_ALARM_ON_C) errMixFlag = true;
  if (isnan(tCat) || tCat >= CAT_ALARM_HIGH_C) errCatFlag = true;
  errFlowFlag = flowLowCondition;

  json += "\"errMix\":" + String(errMixFlag ? "true" : "false") + ",";
  json += "\"errCat\":" + String(errCatFlag ? "true" : "false") + ",";
  json += "\"errFlow\":" + String(errFlowFlag ? "true" : "false") + ",";

  // Compose status message
  String statusMsg = "";
  if (!alarmActive) {
    statusMsg = "Status: OK";
  } else {
    // Alarm(s) active – build message list
    String msgList = "";
    if (isnan(tMix)) msgList += "Mixer sensor FAULT; ";
    if (isnan(tCat)) msgList += "Cat sensor FAULT; ";
    if (tMix >= MIX_ALARM_CRIT_C) msgList += "Mixer OVERHEAT; ";
    else if (tMix >= MIX_ALARM_HIGH_C) msgList += "Mixer temp high; ";
    else if (tMix >= MIX_ALARM_ON_C) msgList += "Mixer temp warning; ";
    if (tCat >= CAT_ALARM_CRIT_C) msgList += "Catalyst OVERHEAT; ";
    else if (tCat >= CAT_ALARM_HIGH_C) msgList += "Catalyst temp high; ";
    if (errFlowFlag) msgList += "Flow LOW; ";
    if (autoMode && !fuelActive && pwmCmd > 0) msgList += "Pump running with engine off; ";

    if (msgList.endsWith("; ")) {
      msgList.remove(msgList.length() - 2);  // remove trailing "; "
    }
    // Choose prefix based on highest severity level
    String prefix = "";
    if (alarmLevel == 3) prefix = "CRITICAL: ";
    else if (alarmLevel >= 1) prefix = "WARNING: ";
    statusMsg = prefix + msgList;
  }
  json += "\"status\":\"" + statusMsg + "\"";
  json += "}";
  server.send(200, "application/json; charset=utf-8", json);
}

// Handle /set for setting parameters (pwm, mode, mute, alarm toggle)
void handleSet() {
  if (server.hasArg("pwm")) {
    // Manual PWM control (only effective in Manual mode)
    int v = constrain(server.arg("pwm").toInt(), 0, 99);
    if (!autoMode) {
      pwmCmd = v;
      x9cSet(pwmCmd);
    }
  }
  if (server.hasArg("mode")) {
    String m = server.arg("mode");
    if (m == "manual") {
      // Switch to Manual mode
      autoMode = false;
      // Upon entering manual, silence alarm (give user control without noise)
      setAlarmSilenced(true);
    } else if (m == "auto") {
      // Switch to Auto mode
      autoMode = true;
      // (Alarm and pump control will take over in next loop cycle)
    }
  }
  if (server.hasArg("alarm")) {
    // Manual alarm relay control (Manual mode only)
    String val = server.arg("alarm");
    if (!autoMode) {
      if (val == "on") {
        // Sound alarm (de-energize relay)
        setAlarmSilenced(false);
      } else {
        // Silence alarm (energize relay)
        setAlarmSilenced(true);
      }
    }
  }
  if (server.hasArg("mute")) {
    int m = server.arg("mute").toInt();
    if (m == 1) {
      // Mute alarm for 5 minutes
      alarmMuted = true;
      alarmMutedUntil = millis() + 5*60*1000;
    } else {
      // Unmute alarm immediately
      alarmMuted = false;
      alarmMutedUntil = 0;
    }
  }
  server.send(200, "text/plain; charset=utf-8", "OK");
}

// Handle /calibrate endpoint to initiate calibration
void handleCalibrate() {
  if (autoMode) {
    server.send(400, "text/plain; charset=utf-8", "ERROR: Switch to manual mode first");
    return;
  }
  // Only allow calibration in manual mode (autoMode is false here)
  calibrating = true;
  calStepIndex = -1;
  // (calibration state will be advanced in main loop)
  server.send(200, "text/plain; charset=utf-8", "CALIBRATION_STARTED");
}

// ================= PERIODIC UPDATE FUNCTIONS =================

// Update fuelActive status (called frequently)
void updateFuelSense() {
  static uint32_t lastCheck = 0;
  if (millis() - lastCheck < 50) return;  // limit check frequency to reduce noise
  lastCheck = millis();
  fuelActive = (digitalRead(FUEL_SENSE) == LOW);
}

// Update flow measurement (calculate flow rate once per second)
void updateFlow() {
  static uint32_t lastMs = 0;
  static uint32_t lastPulses = 0;
  uint32_t now = millis();
  if (now - lastMs < 1000) {
    return; // only update ~ once per second
  }
  uint32_t dtMs = now - lastMs;
  lastMs = now;
  // Copy pulse count atomically
  uint32_t p;
  noInterrupts();
  p = flowPulses;
  interrupts();
  uint32_t dp = p - lastPulses;
  lastPulses = p;
  flowDpLast = dp;
  // Frequency in Hz
  float hz = (dtMs > 0) ? (dp * 1000.0f / dtMs) : 0.0f;
  flowHz_x100 = (uint32_t)(hz * 100.0f + 0.5f);
  // Flow rate in L/min
  float qLmin = hz / FLOW_K_HZ_PER_LMIN;
  if (qLmin < 0) qLmin = 0;
  if (FLOW_CLAMP_LMIN > 0 && qLmin > FLOW_CLAMP_LMIN) {
    qLmin = FLOW_CLAMP_LMIN;
  }
  flow_Lmin = qLmin;
  flow_Lh   = qLmin * 60.0f;
}

// Update RPM measurement (calculate RPM once per second)
void updateRPM() {
  static uint32_t lastMs = 0;
  static uint32_t lastPulses = 0;
  uint32_t now = millis();
  if (now - lastMs < 1000) {
    return;
  }
  uint32_t dtMs = now - lastMs;
  lastMs = now;
  // Copy pulse count safely
  uint32_t p;
  noInterrupts();
  p = rpmPulses;
  interrupts();
  uint32_t dp = p - lastPulses;
  lastPulses = p;
  rpmDpLast = dp;
  // Frequency in Hz
  float hz = (dtMs > 0) ? (dp * 1000.0f / dtMs) : 0.0f;
  rpmHz_x100 = (uint32_t)(hz * 100.0f + 0.5f);
  // Calculate RPM from frequency
  float r = (RPM_PULSES_PER_REV > 0.0f) ? (hz * 60.0f / RPM_PULSES_PER_REV) : 0.0f;
  if (r < 0) r = 0;
  rpmValue = r;
}

// Update temperature readings and handle alarm conditions/patterns
void updateTempsAndAlarm() {
  static uint32_t last = 0;
  if (millis() - last < 300) return;  // update ~3-4 times per second
  last = millis();
  // Read thermocouples
  tCat = max6675ReadC(MAX_CS_CAT);
  tMix = max6675ReadC(MAX_CS_MIX);

  int newAlarmLevel = 0;
  bool anyAlarm = false;
  // Mixer temperature thresholds
  if (!isnan(tMix)) {
    if (tMix >= MIX_ALARM_CRIT_C) {
      newAlarmLevel = max(newAlarmLevel, 3);
      anyAlarm = true;
    } else if (tMix >= MIX_ALARM_HIGH_C) {
      newAlarmLevel = max(newAlarmLevel, 2);
      anyAlarm = true;
    } else if (tMix >= MIX_ALARM_ON_C) {
      newAlarmLevel = max(newAlarmLevel, 1);
      anyAlarm = true;
    }
  }
  // Catalyst temperature thresholds
  if (!isnan(tCat)) {
    if (tCat >= CAT_ALARM_CRIT_C) {
      newAlarmLevel = max(newAlarmLevel, 3);
      anyAlarm = true;
    } else if (tCat >= CAT_ALARM_HIGH_C) {
      newAlarmLevel = max(newAlarmLevel, 2);
      anyAlarm = true;
    }
  }
  // Sensor faults
  if (isnan(tMix) || isnan(tCat)) {
    newAlarmLevel = max(newAlarmLevel, 3);
    anyAlarm = true;
  }
  // Flow low condition
  if (flowLowCondition) {
    newAlarmLevel = max(newAlarmLevel, 1);
    anyAlarm = true;
  }
  // Pump running without engine (potential flood)
  if (autoMode && !fuelActive && pwmCmd > 0) {
    newAlarmLevel = max(newAlarmLevel, 3);
    anyAlarm = true;
  }

  alarmActive = anyAlarm;
  alarmLevel = newAlarmLevel;
  // Handle alarm relay patterns in Auto mode
  if (autoMode) {
    if (!alarmActive) {
      // No alarms: keep relay energized (silent)
      setAlarmSilenced(true);
    } else {
      // Alarm active
      // Auto-reset mute if time expired
      if (alarmMuted && millis() > alarmMutedUntil) {
        alarmMuted = false;
      }
      bool shouldSound = false;
      // Pattern based on level (with ~30s cycle)
      static uint32_t levelStartTime = 0;
      static int prevLevel = 0;
      if (alarmLevel != prevLevel) {
        levelStartTime = millis();
        prevLevel = alarmLevel;
      }
      uint32_t elapsed = millis() - levelStartTime;
      if (alarmLevel >= 3) {
        // Level 3: continuous
        shouldSound = true;
      } else if (alarmLevel == 2) {
        // Level 2: 3s ON, 27s OFF
        uint32_t cycle = elapsed % 30000;
        if (cycle < 3000) shouldSound = true;
        else shouldSound = false;
      } else if (alarmLevel == 1) {
        // Level 1: 1s ON, 29s OFF
        uint32_t cycle = elapsed % 30000;
        if (cycle < 1000) shouldSound = true;
        else shouldSound = false;
      }
      if (alarmMuted) {
        shouldSound = false;
      }
      setAlarmSilenced(!shouldSound);
    }
  }
}

// Update pump control (automatic mode): adjust pwmCmd based on RPM and temperature
void updatePumpControl() {
  if (!autoMode || calibrating) {
    // No automatic control in manual mode or during calibration
    return;
  }
  int targetPwm = 0;
  if (fuelActive && rpmValue > ENGINE_RPM_MIN) {
    // Engine running above threshold -> compute base pump command from RPM
    float rpmClamped = rpmValue;
    if (rpmClamped > 5000) rpmClamped = 5000;
    if (rpmClamped < ENGINE_RPM_MIN) rpmClamped = ENGINE_RPM_MIN;
    float fraction = (rpmClamped - ENGINE_RPM_MIN) / (5000 - ENGINE_RPM_MIN);
    float basePwmF = 20.0f + fraction * (100.0f - 20.0f);
    if (basePwmF > 100.0f) basePwmF = 100.0f;
    targetPwm = (int)(basePwmF + 0.5f);
    // Temperature-based boost control
    if (!boostActive && tMix >= MIX_ALARM_HIGH_C) {
      boostActive = true;
    } else if (boostActive && tMix <= (MIX_ALARM_HIGH_C - 5.0f)) {
      boostActive = false;
    }
    if (tMix >= MIX_ALARM_CRIT_C) {
      targetPwm = 99;
    } else if (boostActive) {
      targetPwm = min(99, targetPwm + 10);
    }
  } else {
    // Engine off or below threshold -> pump off
    targetPwm = 0;
    boostActive = false;
  }
  // Flow monitoring against calibration data
  static int lowFlowCount = 0;
  if (calibrateComplete && targetPwm >= 20) {
    // Expected flow from calibration table (interpolate)
    float expectedFlow = 0.0f;
    if (targetPwm <= calPwmStep[0]) {
      expectedFlow = calFlowLmin[0];
    } else if (targetPwm >= calPwmStep[CAL_STEPS-1]) {
      expectedFlow = calFlowLmin[CAL_STEPS-1];
    } else {
      for (int i = 0; i < CAL_STEPS-1; ++i) {
        if (targetPwm >= calPwmStep[i] && targetPwm <= calPwmStep[i+1]) {
          // Linear interpolation between points
          float ratio = (targetPwm - calPwmStep[i]) / float(calPwmStep[i+1] - calPwmStep[i]);
          expectedFlow = calFlowLmin[i] + ratio * (calFlowLmin[i+1] - calFlowLmin[i]);
          break;
        }
      }
    }
    // If measured flow < 80% of expected, increment low-flow counter
    if (expectedFlow > 0 && flow_Lmin < expectedFlow * 0.8f) {
      lowFlowCount++;
      if (lowFlowCount >= 3) {
        flowLowCondition = true;
      }
    } else {
      // Flow normal or not in range -> reset
      flowLowCondition = false;
      lowFlowCount = 0;
    }
  } else {
    // No calibration data or pump off -> reset flag
    flowLowCondition = false;
    lowFlowCount = 0;
  }
  // Apply the new pump command
  pwmCmd = targetPwm;
  x9cSet(pwmCmd);
}

// Manage calibration process if active (non-blocking state machine)
void updateCalibration() {
  if (!calibrating) return;
  if (calStepIndex < 0) {
    // Start calibration at first step
    calStepIndex = 0;
    x9cHomeMin();
    pwmCmd = 0;
    x9cSet(0);
    delay(500);  // brief pause after homing
    int pwmVal = calPwmStep[calStepIndex];
    pwmCmd = pwmVal;
    x9cSet(pwmVal);
    // Reset flow pulse count to start clean
    noInterrupts();
    flowPulses = 0;
    interrupts();
    calStepStart = millis();
    calStepAccumFlow = 0.0f;
    calStepAccumCount = 0;
    return;
  }
  uint32_t now = millis();
  if (now - calStepStart < 10000) {
    // Still in current step (10 seconds)
    calStepAccumFlow += flow_Lmin;
    calStepAccumCount++;
  } else {
    // Step time completed
    float avgFlow = 0.0f;
    if (calStepAccumCount > 0) {
      avgFlow = calStepAccumFlow / calStepAccumCount;
    } else {
      avgFlow = flow_Lmin;
    }
    calFlowLmin[calStepIndex] = avgFlow;
    calStepIndex++;
    if (calStepIndex < CAL_STEPS) {
      // Move to next calibration step
      int pwmVal = calPwmStep[calStepIndex];
      pwmCmd = pwmVal;
      x9cSet(pwmVal);
      noInterrupts();
      flowPulses = 0;
      interrupts();
      calStepStart = millis();
      calStepAccumFlow = 0.0f;
      calStepAccumCount = 0;
    } else {
      // Calibration finished
      calibrating = false;
      calibrateComplete = true;
      // Save calibration data to NVS
      prefs.begin("ECU", false);
      for (int i = 0; i < CAL_STEPS; ++i) {
        char key[8];
        sprintf(key, "cal%d", calPwmStep[i]);
        prefs.putFloat(key, calFlowLmin[i]);
      }
      prefs.end();
      // Reset pump to 0
      pwmCmd = 0;
      x9cSet(0);
      // (Optional: could alert completion via buzzer or UI message)
    }
  }
}

// ================= SETUP AND LOOP =================
void setup() {
  // Initialize X9C digital pot pins
  pinMode(X9C_CS, OUTPUT);
  pinMode(X9C_UD, OUTPUT);
  pinMode(X9C_INC, OUTPUT);
  digitalWrite(X9C_CS, HIGH);
  digitalWrite(X9C_INC, HIGH);
  digitalWrite(X9C_UD, LOW);
  delay(100);
  x9cHomeMin();
  pwmCmd = 0;
  x9cSet(0);

  // Initialize MAX6675 pins
  pinMode(MAX_SCK, OUTPUT);
  pinMode(MAX_SO, INPUT);
  pinMode(MAX_CS_CAT, OUTPUT);
  pinMode(MAX_CS_MIX, OUTPUT);
  digitalWrite(MAX_SCK, LOW);
  digitalWrite(MAX_CS_CAT, HIGH);
  digitalWrite(MAX_CS_MIX, HIGH);

  // Initialize alarm relay (NC strategy)
  pinMode(RELAY_ALARM, OUTPUT);
  // Boot-up beep: brief alarm on startup
  setAlarmSilenced(false);
  delay(500);
  setAlarmSilenced(true);
  alarmActive = false;

  // Fuel sense input
  pinMode(FUEL_SENSE, INPUT_PULLUP);

  // Flow sensor input
  pinMode(FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), isrFlow, FALLING);

  // RPM Hall sensor input
  pinMode(RPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), isrRpm, FALLING);
  // Using FALLING edge so each magnet produces one pulse (was CHANGE before)

  // Load calibration data from NVS (if exists)
  prefs.begin("ECU", true);
  float testVal = prefs.getFloat("cal20", -1.0f);
  if (testVal >= 0) {
    calibrateComplete = true;
    calFlowLmin[0] = testVal;
    for (int i = 1; i < CAL_STEPS; ++i) {
      char key[8];
      sprintf(key, "cal%d", calPwmStep[i]);
      calFlowLmin[i] = prefs.getFloat(key, 0.0f);
    }
  }
  prefs.end();

  // Start WiFi Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_pass);

  // Set up web server routes
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/set", handleSet);
  server.on("/calibrate", handleCalibrate);
  server.begin();
}

void loop() {
  server.handleClient();
  updateFuelSense();
  updateFlow();
  updateRPM();
  updateCalibration();
  if (!calibrating) {  // Skip normal control during calibration
    updatePumpControl();
    updateTempsAndAlarm();
  }
}

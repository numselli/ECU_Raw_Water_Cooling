# Water Mixer ECU (ESP32) — Cooling/Exhaust Water Control + Safety Alarms

This project is an **ESP32-based ECU** for controlling a dedicated **water mixer pump** (exhaust cooling / water injection) with:
- **Auto mode** (RPM-based target flow + temperature bias)
- **Manual mode** (direct PWM control OR target flow closed-loop control)
- **Web UI** over ESP32 Wi-Fi Access Point (no router needed)
- **Event log** stored in NVS (last 10 events)
- **Calibration routine** to build a PWM→Flow table
- **Safety alarms** (temps, no flow, restriction, sensor faults, hall fault, etc.)

Originally built for a DIY hybrid boat project (Prius drivetrain), but should be adaptable to any setup with a flow sensor + pump.

---

## Features

### Control
- **Auto mode**
  - Requires engine running: `fuelActive && RPM > ENGINE_RPM_MIN`
  - Target flow is derived from RPM (starter curve)  
  - Adds extra flow when **mixer temperature rises above 60°C**
- **Manual mode**
  - **Direct PWM** slider (0–99)
  - **Target Flow (L/min)** slider enables semi-automatic control:
    - Prime at 100% until flow is detected
    - Then closed-loop adjusts PWM to match target

### Sensors
- **Flow sensor** (pulse input) → L/min + L/h  
  - Adjustable scaling factor `flowCalScale` stored in NVS (editable in Web UI)
- **RPM Hall sensor** (pulse input)  
  - Assumes 2 pulses per revolution (2 magnets) by default
- **2× MAX6675 thermocouples**
  - `Tcat` and `Tmix`

### Alarms (NC relay strategy)
Alarm relay is **Normally Closed strategy**:
- Relay energized = **silent**
- Relay de-energized = **alarm sounds**

Alarm levels:
- **Level 1**: 1s beep every 30s  
- **Level 2**: 3s beep every 30s  
- **Level 3**: continuous alarm  

### Fault detection included
- Mixer temp warning/high/critical + sensor fault
- Catalyst temp high/critical + sensor fault
- **No Flow** (pump ON but flow near zero)
- **Unexpected Flow** (flow when pump should be OFF)
- **Flow Restriction** (measured < expected * ratio using calibration table)
- **Flow vs Calibration mismatch** (avg flow deviates > tolerance)
- **Hall sensor fault**
  - Fuel pump active but RPM stays below threshold for too long
  - RPM too high immediately (sanity check)

---

## Web UI (built-in)

The ESP32 hosts a web interface over Wi-Fi AP:

- SSID: `ECU_DEBUG`
- Password: `12345678`
- Open: `http://192.168.4.1`

UI shows:
- Fuel status, RPM, Flow, Tcat, Tmix
- PWM command and X9C position
- Alarm state + level
- Calibration status + calibration table
- Event log modal (last 10 events)
- Manual controls (manual mode only)

---

## Hardware

### MCU
- ESP32 (Arduino framework)

### Outputs
- Pump command via **X9C digital potentiometer** (0–99 “PWM position”)

### Inputs
- Flow sensor pulse input (GPIO interrupt)
- RPM Hall pulse input (GPIO interrupt)
- Fuel pump active sense (PC817 / optocoupler input)

### Temperature
- MAX6675 #1 = CAT thermocouple
- MAX6675 #2 = MIXER thermocouple  
(shared SCK/SO, separate CS pins)

### Alarm
- One output driving an alarm relay (NC strategy)

---

## Pin map (current sketch)

| Function | GPIO | Notes |
|---|---:|---|
| Flow sensor | 0 | interrupt, `INPUT_PULLUP`, falling edge |
| RPM Hall | 1 | interrupt, `INPUT_PULLUP`, falling edge |
| Fuel sense | 2 | `INPUT_PULLUP`, **LOW = ON** (via PC817) |
| MAX6675 CS (CAT) | 3 | |
| MAX6675 CS (MIX) | 4 | |
| MAX6675 SO | 5 | shared |
| MAX6675 SCK | 6 | shared |
| X9C U/D | 7 | |
| X9C CS | 8 | |
| X9C INC | 9 | |
| Alarm relay | 10 | NC strategy |

> ⚠️ Note: GPIO0/1 can be special on some ESP32 boards (boot/serial). If your board behaves weirdly, consider remapping to safer GPIOs.

---

## Firmware logic overview

### Engine running condition (Auto mode)
Auto control only runs when:
- `fuelActive == true` (fuel sense input is LOW)
- `rpmValue > ENGINE_RPM_MIN` (default 800 RPM)

### Auto target flow curve
A simple starter curve in code:
- ~6 L/min at ~1000 RPM
- up to ~45 L/min max (clamped)

### Mixer temperature boost
When `Tmix` exceeds **60°C**, additional flow bias ramps up until **70°C**:
- At/above 70°C → forced full flow target

---

## Calibration

Calibration builds a PWM→Flow table using fixed PWM steps:

`20,30,40,50,60,70,80,90,100`

Process:
1. Switch to **Manual mode**
2. Press **Calibrate**
3. Pump runs each step for ~10 seconds and averages flow
4. Table is validated (sanity checks):
   - must have real flow
   - must not be too flat
   - mostly monotonic

If calibration fails:
- defaults are restored
- a warning is shown (and logged)

### Restore Default Calibration
Manual mode only.
Writes a safe default calibration table to NVS.

### Flow calibration scale
You can adjust flow scaling without reflashing firmware:
- Manual mode → set **Flow cal scale** and Save

---

## Event log

Stored in ESP32 NVS, keeps **last 10 events** (newest first):
- Mixer temp warn/high/crit
- Cat temp high/crit
- Sensor faults
- No flow / restriction / unexpected flow / mismatch
- Calibration attention
- Hall fault

Open **Logs** in the web UI to view or clear.

---

## Build / Flash

### Requirements
- Arduino IDE (or PlatformIO)
- ESP32 board support installed
- Libraries used:
  - `WiFi.h`
  - `WebServer.h`
  - `Preferences.h`

### Flash
1. Select your ESP32 board + correct COM port
2. Paste the sketch into Arduino IDE
3. Upload
4. Connect to Wi-Fi AP `ECU_DEBUG`
5. Open `http://192.168.4.1`

---

## Safety notes (read this)
This controller can run pumps and activate alarms — use common sense:
- Test with **hose in a bucket** during calibration.
- Verify your pump drive method (X9C + motor controller) is stable.
- Treat temperature sensors as critical. Sensor faults are escalated to **Level 3** by design.
- Use proper fusing, marine wiring practice, strain relief, and waterproofing.

---

## Customization quick list

Edit these in the sketch:
- `ENGINE_RPM_MIN` (default 800)
- Mixer thresholds: `MIX_WARN_C`, `MIX_HIGH_C`, `MIX_CRIT_C`
- Catalyst thresholds: `CAT_HIGH_C`, `CAT_CRIT_C`
- Flow sensor conversion / scaling
- Auto target flow function: `autoFlowTargetFromRpm()`
- Temp bias function: `tempFlowBiasFromMix()`
- AP SSID/password: `ap_ssid`, `ap_pass`

---

## Credits / Contact
Built by **Fox (YachtCityLife)** for a DIY hybrid boat project.
PRs and issues are welcome — especially if you adapt it to different pumps/sensors or improve the control loop.

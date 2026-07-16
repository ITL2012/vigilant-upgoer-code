# ISAAC L Flight Controller — Complete System Documentation

---

# Table of Contents
1. [Project Overview](#1-project-overview)
2. [Hardware Pin Mapping](#2-hardware-pin-mapping)
3. [Global State & Configuration (`globals.h`)](#3-global-state--configuration-globalsh)
4. [Boot Sequence (`main.cpp setup()`)](#4-boot-sequence-maincpp-setup)
5. [Main Control Loop (`main.cpp loop()`)](#5-main-control-loop-maincpp-loop)
6. [Instrument Drivers (`instruments.h`)](#6-instrument-drivers-instrumentsh)
7. [Guidance, Navigation & Control (`guidance_flight_control.h`)](#7-guidance-navigation--control-guidance_flight_controlh)
8. [Flight Profile System (`flight_profile.h`)](#8-flight-profile-system-flight_profileh)
9. [Background Tasks (`background_tasks.h`)](#9-background-tasks-background_tasksh)
10. [Launch Sequence & Safety (`Launchsequence.h`)](#10-launch-sequence--safety-launchsequenceh)
11. [Web Server & Ground Control (`web_server.h`)](#11-web-server--ground-control-web_serverh)
12. [Web UI Dashboard (`index.html`)](#12-web-ui-dashboard-indexhtml)
13. [Debug CLI (`debug_cli.h`)](#13-debug-cli-debug_clih)
14. [Audio Feedback (`buzzers.h`)](#14-audio-feedback-buzzersh)
15. [Complete Data Flow Summary](#15-complete-data-flow-summary)

---

## 1. Project Overview

This is an ESP32-S3 based flight controller for an actively controlled rocket. It uses:

- **BNO085 IMU** (SPI) for attitude (quaternion) and linear acceleration at 100Hz
- **BMP580 barometer** (SPI) for pressure altitude at 50Hz
- **Ublox GPS** (UART2) for position and ground speed at 10Hz
- **PCA9685 PWM driver** (I2C) for 8 servo control surfaces
- **SD card** (SPI/HSPI) for flight data logging
- **WiFi AP** for ground-control dashboard
- **Buzzer** (GPIO 38, LEDC) for audible feedback

The rocket has 8 control surfaces (4 forward canards + 4 aft flaps), each with an independent PID controller. A weighted blending matrix maps roll/pitch/yaw errors to individual surface deflections. A gain-scheduling lookup table automatically adjusts PID gains based on estimated airspeed. Programmable flight profiles (quintic S-curve interpolation) allow pre-programmed maneuvers triggered by time, apogee, altitude, or velocity events.

### File Inventory (12 files, ~4480 lines)

| File | Lines | Purpose |
|------|-------|---------|
| `globals.h` | 255 | Global types, enums, pin definitions, extern declarations |
| `main.cpp` | 419 | Hardware init, main loop (sensor read → fusion → guidance → stabilization → logging) |
| `instruments.h` | 124 | BNO085 IMU + BMP580 barometer SPI drivers |
| `guidance_flight_control.h` | 524 | Sensor fusion, state machine, PID stabilization, gain scheduling, Kalman airspeed |
| `flight_profile.h` | 542 | Programmable flight profile engine, JSON parsing, SD storage manager |
| `background_tasks.h` | 358 | RTOS tasks (GPS telemetry, SD writer), log system, PSRAM-backed queue |
| `Launchsequence.h` | 184 | Pyro firing, instrument checks, arm/disarm logic |
| `web_server.h` | 867 | WiFi AP, HTTP handlers, OTA updates, phase-aware route switching |
| `index.html` | 686 | Full GCS dashboard HTML/CSS/JS |
| `debug_cli.h` | 351 | Serial debug CLI for testing and diagnostics |
| `buzzers.h` | 85 | LEDC buzzer tones and chimes |
| `FLIGHT_CONTROL_DOCS.md` |  | This file |

---

## 2. Hardware Pin Mapping

Defined in `globals.h`.

### SPI Buses

**VSPI (primary, for sensors):**
| Signal | GPIO | Device |
|--------|------|--------|
| MOSI | 11 | Shared |
| MISO | 13 | Shared |
| CLK  | 12 | Shared |
| CS (BMP580) | 10 | Barometer |
| CS (BNO085) | 9 | IMU |
| INT (BNO) | 8 | IMU data-ready |
| RST (BNO) | 14 | IMU reset |
| INT (BMP) | 18 | Baro data-ready |

**HSPI (secondary, for SD card):**
| Signal | GPIO | Device |
|--------|------|--------|
| MOSI | 35 | SD card |
| MISO | 37 | SD card |
| CLK  | 36 | SD card |
| CS   | 21 | SD card chip select |

### I2C (PCA9685 servo driver)
| Signal | GPIO | Device |
|--------|------|--------|
| SDA | 1 | PCA9685 (addr 0x40) |
| SCL | 3 | PCA9685 |
| Speed | 400kHz | - |

### UART
| Port | GPIO | Baud | Device |
|------|------|------|--------|
| UART2 RX | 16 | 9600 | GPS |
| UART2 TX | 17 | 9600 | GPS |

### Other
| GPIO | Function |
|------|----------|
| 3 | 5V Enable (output) |
| 7 | Pyro arm pin (output) |
| 38 | Buzzer (LEDC channel 0) |
| 999 | Pyro 1, 2, 3 (placeholder — change to real pins) |

---

## 3. Global State & Configuration (`globals.h`)

### Enumerations

**`SystemMode`** — two major operational states:
```cpp
MODE_TRANSPORT     // Ground handling: sensors inactive, servos centered
MODE_ACTIVE_PAD    // On the pad: sensors live, ready for flight
```

**`FlightPhase`** — 7-phase state machine:
```cpp
TRANSPORT  // Software lockout (no flight logic)
PAD        // On pad, monitoring sensors
READY      // Armed, awaiting liftoff
BOOST      // Motor burning (accel > 3g)
COAST      // Motor burnout (accel < 0.5g)
DESCENT    // Apogee passed, parachute deployed
LANDED     // (not used in state machine — goes to TRANSPORT)
```

**`LogDestination`** — for the `write()` function:
```cpp
LOG_SERIAL  // Serial console only
LOG_SD      // SD card only
LOG_BOTH    // Both
```

**`LogSeverity`**:
```cpp
LOG_INFO
LOG_WARN
LOG_ERROR
```

### Core Atomic State Variables

These are `std::atomic` for thread-safe access across RTOS tasks:
```cpp
currentSystemMode  // MODE_TRANSPORT or MODE_ACTIVE_PAD
currentPhase       // Current flight phase (TRANSPORT..DESCENT)
systemArmed        // Pyro/servo safety arm state
wifiActive         // Whether WiFi is still transmitting
systemBaseEpochMs  // Wall clock seed from browser (ms since epoch)
systemBaseMillis   // Corresponding millis() capture
lastIMUReport_ms   // Timestamp of last IMU data (for timeout detection)
```

### Navigation State (global floats, accessed from main loop + fusion)
```cpp
V_z              // Vertical velocity from complementary filter (m/s)
filter_alt       // Filtered altitude from complementary filter (m)
baseline_altitude// Ground-level baro altitude offset (m)
previous_altitude// Previous raw altitude (for baro Vz derivative)
qnh_pressure     // Calibrated sea-level pressure (hPa)
```

### Telemetry Snapshot (written each loop, read by web server)
```cpp
latestServoAngles[8]   // Current servo deflection angles
latestPIDOutputs[8]    // Current PID output values
latestActiveGains[3]   // Kp, Ki, Kd being used
latestAirspeed         // Kalman-estimated airspeed (m/s)
latestKalmanP          // Kalman covariance
latestBaroAlpha        // Barometer weighting factor
latestBaroPressure     // hPa
latestBaroTemp         // °C
latestDt               // Loop dt (seconds)
latestQx..latestQw     // Quaternion
latestAx..latestAz     // Linear acceleration (m/s²)
```

### Servo Override
```cpp
servoOverrideActive     // Flag: manual override active
servoOverrideAngles[8]  // Manual angle targets
```

### Log Buffers
```cpp
serialLogBuffer[4096]  // Circular buffer for web serial log
logCache[8192]         // Pre-SD circular buffer (before SD init)
LOG_QUEUE_LEN = 500    // RTOS queue depth for flight data packets
```

### Important Externals
```cpp
pwm              // Adafruit_PWMServoDriver (PCA9685)
gps              // TinyGPSPlus parser
sharedTelemetry  // Thread-safe telemetry struct
telemetryMutex   // Guards sharedTelemetry
sdLogQueue       // RTOS queue for log packets
profileEngine    // FlightProfileEngine instance
```

### Configuration Constants
```cpp
debugMode = true                     // Enables Serial debug CLI
enableBuzzer = true                  // Enables buzzer tones
stabilizationMode = true             // Enables PID stabilization
enforceGPSLock = false               // GPS not required for arming
enforceSDCard = true                 // SD card required
enableWaypointGuidance = false       // Waypoint guidance disabled
LAUNCH_ACCEL_THRESHOLD = 25.0f       // (but actual liftoff check uses 3.0g)
APOGEE_VEL_THRESHOLD = -0.5f         // m/s
DEPLOYMENT_ALTITUDE = 50.0f          // meters
WDT_TIMEOUT_S = 5                    // Watchdog timeout
```

### `convertQuaternionToEuler()`
A utility function that converts quaternion (r,i,j,k) to Euler angles and writes to the globals `current_roll`, `current_pitch`, `current_yaw`. Uses standard aerospace formulas. Note: this function exists but the main loop calculates Euler angles inline rather than calling it.

---

## 4. Boot Sequence (`main.cpp setup()`)

On power-up, `setup()` runs once:

### Step 1: Serial & Log System
- `Serial.begin(115200)` — allow 1 second for USB enumeration
- `initLogCache()` — initializes the 8KB pre-SD ring buffer and its mutex

### Step 2: Watchdog
- `esp_task_wdt_init(5s, true)` — 5-second hardware watchdog
- `esp_task_wdt_add(NULL)` — add current task (Core 1 loop) to watchdog

### Step 3: RTOS Primitives
- `telemetryMutex = xSemaphoreCreateMutex()` — guards `sharedTelemetry`
- `webServerMutex = xSemaphoreCreateMutex()` — guards web server state
- `sdLogQueue = createPSRAMBackedQueue(500, sizeof(LogPacket))` — flight data queue

If any of these fail, the system `while(1)` halts.

### Step 4: I2C Bus (PCA9685 servo driver)
- `Wire.begin(SDA=1, SCL=3)` at 400kHz
- `pwm.begin()` + `pwm.setPWMFreq(50)` — standard servo frequency
- `UserSpace::initServos()` — centers all 8 to 90°

### Step 5: PID Initialization
- `initStabilizationPIDs()` — sets all 8 PIDs to automatic mode, output limits ±30°
- `airspeedFilter.init()` — resets Kalman filter state

### Step 6: SPI Bus (HSPI for SD)
- Creates `hspi = new SPIClass(HSPI)` with pins 35(MOSI), 37(MISO), 36(CLK), 21(CS)

### Step 7: SD Card
- `SDInit()` — attempts to mount SD; if successful:
  - Creates `/flight_log.csv` with CSV header if not present
  - Creates `/system_log.txt` if not present
  - `flushLogCacheToSD()` — writes any pre-init log messages to system_log.txt
  - `loadAllProfilesFromSD()` — loads JSON profiles from `/profiles/` directory

### Step 8: GPS
- `gpsSerial.begin(9600, SERIAL_8N1, RX=16, TX=17)`

### Step 9: RTOS Task Creation
Three tasks are created on Core 0 (leaving Core 1 for `loop()`):

1. **TelemetryTask** (priority 2, 4KB stack) — GPS data collection
2. **SDWriterTask** (priority 1, 4KB stack) — SD card writing (only if SD ready)
3. **WifiServerTask** (priority 1, 8KB stack) — WiFi AP + HTTP server

### Step 10: Completion
- `startupChime()` — musical C-E-G-C tone sequence
- Enters `loop()` on Core 1

---

## 5. Main Control Loop (`main.cpp loop()`)

The `loop()` function runs on Core 1 at maximum speed. It does NOT use `delay()` (except in TRANSPORT mode where it yields 10ms). The sequence per iteration:

### 5.1 CLI & Watchdog
- If `debugMode == true`, `debugCLI_loop()` processes serial input
- `esp_task_wdt_reset()` — pet the watchdog

### 5.2 Compute dt
```cpp
dt = (micros() - lastMicros) / 1,000,000.0f;
lastMicros = currentMicros;
```

### 5.3 MODE_ACTIVE_PAD Path (sensors live)
Only executes if `currentSystemMode == MODE_ACTIVE_PAD`:

#### a. Barometer Read
```cpp
raw_altitude = readBaroAltitude();  // bmp.readAltitude(qnh) - baseline
```
Also reads BMP580 pressure (hPa) and temperature (°C) into `latestBaroPressure` and `latestBaroTemp`.

#### b. IMU Read
```cpp
readIMU(qx, qy, qz, qw, lin_ax, lin_ay, lin_az)
```
On success, updates `latestQx..latestQw`, `latestAx..latestAz`, and computes Euler angles:
```
roll  = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx² + qy²)) * 180/π
pitch = asin(2*(qw*qy - qz*qx)) * 180/π
yaw   = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy² + qz²)) * 180/π
```

#### c. Sensor Fusion
```cpp
updateRocketFusion(lin_ax, lin_ay, lin_az, qx, qy, qz, qw, raw_altitude, previous_altitude, dt);
```
This is the core state estimation function (see §7.1).

#### d. Flight Profile Engine
```cpp
apogeeDetected = (currentPhase == DESCENT);
profileEngine.update(dt, filter_alt, V_z, currentPhase, apogeeDetected);
```
Updates the active profile's S-curve interpolation and checks step triggers.

#### e. WiFi Shutdown on Liftoff
If `currentPhase == BOOST` and WiFi is still active, immediately:
- Sets `filter_alt = raw_altitude` (re-baseline after shock of launch)
- Calls `shutdownWiFiNetwork()` — powers off WiFi transceiver

#### f. Control Surface Actuation
**Phase PAD or READY:**
- Calls `userPreFlightLoop(dt)` (currently empty)
- If `servoOverrideActive`, writes override angles to servos

**Phase BOOST or COAST:**
- Calls `userFlightStabilizationLoop(roll, pitch, yaw, V_z, dt)` — the main PID control law

#### g. GPS Airspeed Update
```cpp
if (gps.location.isUpdated() && gps.speed.isValid())
    airspeedFilter.updateGPS(gps.speed.mps(), true);
```

### 5.4 MODE_TRANSPORT Path
- `userPreFlightLoop(dt)` (empty)
- `delay(10)` — yield to idle task

### 5.5 Shared Telemetry Update
```cpp
updateSharedTelemetry(roll, pitch, yaw, raw_altitude, filter_alt, V_z);
```
Writes current state into `sharedTelemetry` under mutex for the web server to read.

### 5.6 Flight Data Logging (every 10ms in ACTIVE_PAD)
Packs a `LogPacket` with all sensor, servo, PID, and estimation data and pushes to the RTOS queue via `pushLogPacket()`. The SDWriterTask picks this up from the other core and writes it to CSV.

---

## 6. Instrument Drivers (`instruments.h`)

### BNO085 IMU (SPI, CS=9, RST=14)
Initialization:
```cpp
bno08x.begin_SPI(BNO_CS, BNO_INT, &SPI);  // VSPI bus
```

Three reports enabled at 10ms intervals (100Hz):
- `SH2_ROTATION_VECTOR` — quaternion orientation
- `SH2_LINEAR_ACCELERATION` — acceleration minus gravity
- `SH2_ARVR_STABILIZED_RV` — stabilized rotation vector (not used in main loop)

Data read (`readIMU`):
- On `wasReset()`, re-enables reports
- `getSensorEvent()` dispatches:
  - `SH2_ROTATION_VECTOR` → fills qx,qy,qz,qw
  - `SH2_LINEAR_ACCELERATION` → fills lin_ax,lin_ay,lin_az
- If no data received for `IMU_TIMEOUT_MS` (500ms), returns false

### BMP580 Barometer (SPI, CS=10)
Initialization:
```cpp
bmp.begin(BMP_CS, &SPI)
```
Configuration:
- Temp oversampling: 8x
- Pressure oversampling: 4x
- IIR filter coefficient: 3
- Output data rate: 50 Hz

`readBaroAltitude()`:
```cpp
bmp.performReading() -> bmp.readAltitude(qnh_pressure) - baseline_altitude
```

`calibrateGroundAltitude()`:
- Takes 50 samples over ~2 seconds
- Averages to set `baseline_altitude`
- Captures QNH pressure from first valid reading
- Called when switching to `MODE_ACTIVE_PAD`

---

## 7. Guidance, Navigation & Control (`guidance_flight_control.h`)

This is the most complex file — the brain of the flight controller. It handles sensor fusion, state transitions, PID stabilization, gain scheduling, and airspeed estimation.

### 7.1 Sensor Fusion: `updateRocketFusion()`

**Purpose:** Estimate vertical velocity ($V_z$) and filtered altitude ($filter\_alt$) by fusing IMU acceleration and barometer altitude.

**Step 1 — Earth-frame acceleration:**
Projects the body-frame linear acceleration into the earth frame using the quaternion:
```cpp
a_earth_z = (2*(qx*qz - qw*qy))*lin_ax +
            (2*(qy*qz + qw*qx))*lin_ay +
            (1 - 2*(qx² + qy²))*lin_az;
```
This isolates the vertical component regardless of rocket orientation.

**Step 2 — Barometric velocity:**
```cpp
V_baro = (current_altitude - last_altitude) / dt;
```
A simple numerical derivative. Prone to noise, so it's blended with IMU.

**Step 3 — Complementary filter with phase-adaptive alpha:**
The alpha parameter determines how much to trust the IMU vs. the barometer:

| Phase | Alpha | Rationale |
|-------|-------|-----------|
| TRANSPORT/PAD | 0.80 | Heavy baro weight (stable, no acceleration) |
| BOOST | 0.995 | Heavy IMU weight (engine pressure spikes corrupt baro) |
| COAST | 0.96 + 0.0001·\|Vz\|, capped at 0.995 | Increase IMU trust as speed increases |
| DESCENT | 0.90 | Moderate balance |

Update:
```cpp
V_predicted = V_z + a_earth_z * dt;
V_z = alpha * V_predicted + (1 - alpha) * V_baro;
filter_alt += V_z * dt;
```

**Step 4 — State machine evaluation:**
```cpp
process_flight_state_machine(a_earth_z, filter_alt);
```

**Step 5 — Airspeed Kalman prediction:**
```cpp
airspeedFilter.predict(a_earth_z, dt);
airspeedFilter.updateIMU(a_earth_z, dt);
airspeedFilter.updateBaro(V_baro);
```

### 7.2 Flight State Machine: `process_flight_state_machine()`

**State transition logic:**

```
TRANSPORT ──→ PAD ──→ READY ──→ BOOST ──→ COAST ──→ DESCENT ──→ TRANSPORT
                (manual)  (arm)    (liftoff)  (burnout)  (apogee)    (landed)
```

| Transition | Condition | Action |
|------------|-----------|--------|
| READY → BOOST | `accel_z > LIFTOFF_ACCEL_G (3.0g) AND armed AND mode == ACTIVE_PAD` | Sets phase to BOOST, resets filter_alt to 0 |
| BOOST → COAST | `accel_z < BURNOUT_ACCEL_G (0.5g)` | Sets phase to COAST |
| COAST → DESCENT | `V_z <= -0.5 m/s AND (max_alt - filter_alt > 1.5m)` for `APOGEE_BUFFER_SIZE (10)` consecutive cycles | Sets phase to DESCENT, fires apogee pyro |
| DESCENT → TRANSPORT | `filter_alt < 5.0m` for 3 consecutive seconds | Sets phase to TRANSPORT |

The apogee detection uses a buffer counter that increments when conditions are met and decrements otherwise — a simple debounce filter to prevent false triggers from turbulence.

**Note:** There's a bug in the DESCENT case: `landing_start_ms` and `tracking_landing` are declared as local variables within the `case DESCENT:` block, shadowing the file-scope variables, so the 3-second landing timer will never work correctly.

### 7.3 PID Stabilization: `userFlightStabilizationLoop()`

This is the core control law, called every loop iteration during BOOST and COAST.

**Architecture: 8 independent PIDs, one per servo.**

#### Surface Weighting Matrix
Each servo $i$ has weights $[W_{roll}, W_{pitch}, W_{yaw}]$ that determine how much each axis error contributes:

```
Index  Surface            Weights [R, P, Y]    Role
  0    Canard 0 (Fwd-L)   [ 0.15,  1.00, 0.00]  Pitch primary, roll secondary
  1    Canard 1 (Fwd-R)   [ 0.15, -1.00, 0.00]  Pitch primary, roll secondary
  2    Canard 2 (Fwd-L)   [ 0.00,  0.00, 1.00]  Yaw primary
  3    Canard 3 (Fwd-R)   [ 0.00,  0.00,-1.00]  Yaw primary
  4    Flap 4  (Aft-L)    [ 1.00, -0.30, 0.00]  Roll primary, neg pitch assist
  5    Flap 5  (Aft-R)    [-1.00, -0.30, 0.00]  Roll primary, neg pitch assist
  6    Flap 6  (Aft-L)    [ 1.00, -0.30, 0.00]  Roll primary, neg pitch assist
  7    Flap 7  (Aft-R)    [-1.00, -0.30, 0.00]  Roll primary, neg pitch assist
```

Canards provide pitch authority; flaps provide roll authority. Sign conventions ensure opposite surfaces deflect in opposite directions.

#### Control Law (per servo $i$)
```cpp
rollErr  = roll  - spRoll;
pitchErr = pitch - spPitch;
yawErr   = yaw   - spYaw;

pidInput[i] = W[i].roll  * rollErr +
              W[i].pitch * pitchErr +
              W[i].yaw   * yawErr;
pidSetpoint[i] = 0;  // drive weighted error to zero
```
Then `pid[i].Compute()` produces `pidOutput[i]`, clamped to ±MAX_DEFLECTION_DEG (30°).

#### Servo Output
```cpp
angle = SERVO_CENTER_DEG (90°) + constrain(pidOutput[i], -30°, 30°);
// Final range: 60° to 120°
```

#### Setpoint Sources
During `PAD/TRANSPORT/READY` phases, the loop sets all PIDs to manual mode and centers servos at 90°.
During `BOOST/COAST` phases:
- If a flight profile is active (`profileEngine.isActive()`), `spRoll/spPitch/spYaw` come from the profile
- If no profile is active, setpoints are (0,0,0) = level flight

#### Gain Scheduling
Applied each loop before PID computation:
```cpp
applyAdvancedGainScheduling(estimated_speed);
```

### 7.4 Gain Scheduling: `applyAdvancedGainScheduling()`

A 5-entry lookup table with linear interpolation between entries:

| Velocity (m/s) | Kp | Ki | Kd |
|:--------------:|:--:|:--:|:--:|
| 15 | 4.80 | 0.60 | 0.24 |
| 30 | 3.00 | 0.40 | 0.15 |
| 50 | 2.00 | 0.30 | 0.10 |
| 80 | 1.30 | 0.20 | 0.07 |
| 120 | 0.90 | 0.12 | 0.04 |

Below 15 m/s, gains are clamped to the first row. Above 120 m/s, clamped to the last row. Between entries, linear interpolation is used. All 8 PIDs get the same gains.

**Rationale:** At low speeds, control surfaces are less effective, so high gains are needed. At high speeds, surfaces generate large forces quickly — gains must be reduced to prevent oscillation/flutter.

### 7.5 Kalman Airspeed Filter: `KalmanAirspeed`

A 1D Kalman filter that estimates speed by fusing three measurement sources:

**State:** $x$ = airspeed (m/s), $P$ = estimate covariance

**Process model:** $x_{k|k-1} = x_{k-1} + a_{along} \cdot dt$, $P_k = P_{k-1} + Q$

**Process noise:** $Q = 0.5$

**Measurement noise:**
- $R_{baro} = 1.0$ (most trusted)
- $R_{gps} = 2.0$
- $R_{imu} = 4.0$ (least trusted — acceleration drift)

**Adaptive weighting:**
- Barometer trust decreases with speed: `baro_alpha = 1 - (|v|/120) * 0.9`, range [0.1, 1.0]
- IMU trust increases with speed: `imu_alpha = 0.1 + (|v|/120) * 0.9`, range [0.1, 1.0]

**Update sources:**
- `predict()` — always called, integrates longitudinal acceleration
- `updateIMU()` — also from acceleration (redundant but provides adaptive weighting)
- `updateBaro()` — from barometric vertical speed $V_{baro}$ (absolute value, since baro measures vertical, not horizontal)
- `updateGPS()` — from GPS ground speed (only when fix available)

**Note:** The filter actually estimates *absolute* vertical speed from the barometer, not horizontal airspeed. The variable name "airspeed" is a misnomer — it's better understood as a fused vertical speed estimate.

---

## 8. Flight Profile System (`flight_profile.h`)

### 8.1 Data Structures

**`ProfileStep`** — one step in a profile:
```cpp
struct ProfileStep {
    float targetRoll;         // degrees
    float targetPitch;        // degrees
    float targetYaw;          // degrees
    float durationSec;        // seconds (for TRIG_TIME)
    uint8_t triggerType;      // TRIG_TIME, TRIG_APOGEE, TRIG_ALTITUDE, TRIG_VELOCITY, TRIG_MANUAL
    float triggerValue;       // altitude (m) or velocity (m/s) threshold
    float maxRateDegPerSec;   // rate limit (0 = unlimited)
};
```

**`FlightProfile`** — a named sequence of up to 16 steps:
```cpp
struct FlightProfile {
    const char* name;       // Profile name
    ProfileStep steps[16];  // Ordered step sequence
    uint8_t numSteps;       // Number of steps used
    bool loop;              // Repeat after last step
};
```

**`JsonProfile`** — runtime/heap version with owned name buffer:
```cpp
struct JsonProfile {
    char name[32];
    ProfileStep steps[16];
    uint8_t numSteps;
    bool loop;
    bool valid;
};
```

### 8.2 Trigger Types

| Trigger | Enum | Behavior |
|---------|------|----------|
| `time` | TRIG_TIME | Step ends after `durationSec` seconds |
| `apogee` | TRIG_APOGEE | Step ends when COAST→DESCENT transition occurs |
| `altitude` | TRIG_ALTITUDE | Step ends when `currentAltitude <= triggerValue` (descending) |
| `velocity` | TRIG_VELOCITY | Step ends when `|Vz| <= triggerValue` |
| `manual` | TRIG_MANUAL | Step ends only on explicit `next()` call |

### 8.3 The Engine: `FlightProfileEngine`

**State:**
```cpp
profile          // Pointer to active FlightProfile
currentStep      // Current step index
stepStartMs      // millis() when current step began
active           // Whether a profile is running
stepComplete     // Flag for manual trigger
startSetpoint[3] // Attitude at step start (for S-curve interpolation)
currentSetpoint[3] // Current output setpoint
targetSetpoint[3]  // Target for current step
```

**`startProfile(profile)`:**
Captures current setpoint as the start point, loads the first step's target, logs start.

**`update()` — called every loop:**
1. **Safety checks:** If phase is ground (TRANSPORT/PAD/READY) or system is not armed, aborts immediately and decays to level
2. **Trigger evaluation:** Checks if current step's trigger condition is met
3. **Watchdog:** If step runs > 30 seconds, force-advance
4. **`advanceStep()`** on trigger: increments step (or loops if `loop == true`, or completes if last step)
5. **`computeSetpoints(dt)`:**
   - Computes normalized progress `t = elapsed / durationSec`, clamped to [0,1]
   - Applies **quintic blend**: $s = 10t^3 - 15t^4 + 6t^5$
   - $setpoint = start + (target - start) \cdot s$
   - Applies per-step rate limit if set

**Quintic blend** ensures zero velocity and zero acceleration at both the start and end of each step — critical for preventing servo jerk and mechanical stress during attitude transitions.

**`decayToLevel(dt)`:**
When no profile is active or after abort, exponentially decays setpoints toward (0,0,0) at 30°/s for safe recovery.

**Accessors:**
```cpp
getSetpointRoll/Pitch/Yaw()  // Current attitude setpoints (read by stabilization loop)
isActive()                   // Whether a profile is running
getCurrentStep() / getNumSteps() / getStepProgress()  // UI status
getProfileName()             // "NONE" if inactive
next()                       // Manually advance step (for TRIG_MANUAL)
abortProfile()               // Stop and decay to level
```

### 8.4 Built-in Profiles (flash-stored, in `main.cpp`)

**MATH_TEST** (5 steps, no loop):
1. Roll 180° over 5s (rate 40°/s)
2. Roll back to 0° over 3s (rate 40°/s)
3. Pitch 20° over 4s (rate 30°/s)
4. Flatten at apogee (trigger: apogee, rate 25°/s)
5. Hold level to 50m (trigger: altitude 50m, rate 20°/s)

**ROLL_CAL** (4 steps, no loop):
1. Roll 90° over 4s (rate 25°/s)
2. Return to 0° over 2s (rate 25°/s)
3. Roll -90° over 4s (rate 25°/s)
4. Return to 0° over 2s (rate 25°/s)

**RECOVERY** (2 steps, no loop):
1. Flatten at apogee (rate 25°/s)
2. Hold level to 30m (trigger: altitude 30m, rate 15°/s)

### 8.5 SD Card Profile Storage

**Format:** JSON files in `/profiles/*.json`

**JSON Schema:**
```json
{
  "name": "MY_PROFILE",
  "loop": false,
  "steps": [
    { "roll": 180, "pitch": 0, "yaw": 0, "duration": 5.0,
      "trigger": "time", "value": 0, "rate": 40 },
    { "roll": 0, "pitch": 0, "yaw": 0, "trigger": "apogee", "rate": 25 }
  ]
}
```

**`loadAllProfilesFromSD()`:** Scans `/profiles/` for `.json` files, parses each, registers in the runtime store. Creates the directory if missing.

**`saveProfileToSD(name, json)`:** Writes a profile to `/profiles/{name}.json`.

**`parseProfileJSON()`:** Uses ArduinoJson v7 to deserialize. Validates structure (must have `steps` array). Returns `JsonProfile`.

**`startProfileByName(name)`:** Searches flash profiles first, then SD-loaded profiles. Returns true if found and started.

**`registerJsonProfile()`:** Adds/updates a profile in the `loadedProfiles[8]` store. Creates a `FlightProfile` wrapper pointing to the `JsonProfile`'s name.

### 8.6 SD Profile Manager Functions

```cpp
findLoadedProfile(name)       // → index in loadedProfiles[], or -1
jsonProfileToFlightProfile()  // Copies step data, points name to json.name
serializeProfileJSON()        // JsonProfile → JSON string
```

---

## 9. Background Tasks (`background_tasks.h`)

### 9.1 Log System

#### Pre-SD Cache (`logCache[8192]`)
A circular byte buffer with head/tail pointers. Used for system log messages that occur before the SD card is initialized. Thread-safe via `logCacheMutex`.

**Functions:**
- `initLogCache()` — zeros head/tail, creates mutex
- `appendToLogCache(msg)` — writes to ring buffer, sets overflow flag if full
- `flushLogCacheToSD()` — writes all cached messages to `/system_log.txt` on SD

#### Serial Log Buffer (`serialLogBuffer[4096]`)
A circular character buffer for the web UI's "Serial Log" display. Written to by `write()` for LOG_SERIAL and LOG_BOTH destinations. Read by the web server's `/serial_log` endpoint.

#### `write()` — Universal Log Function
```cpp
void write(int destination, int severity, const char *format, ...)
```
- Formats message with severity prefix: `[INFO]`, `[WARN]`, `[ERROR]`
- LOG_SERIAL: prints to Serial, appends to serialLogBuffer and logCache
- LOG_SD: writes to `/system_log.txt` (or logCache if SD not ready)
- LOG_BOTH: both of the above

#### `LogPacket` — Flight Data Packet
A ~100-byte struct packed every 10ms during ACTIVE_PAD mode containing:
- Timestamp, epoch_ms
- Roll, pitch, yaw (degrees)
- Raw and filtered altitude, vertical velocity
- Linear acceleration (x, y, z)
- Current flight phase
- All 8 servo angles and PID outputs
- Active PID gains (Kp, Ki, Kd)
- Kalman airspeed, covariance P, baro alpha
- Quaternion (qx, qy, qz, qw)
- Baro pressure, temperature, loop dt

#### `pushLogPacket()`
Sends a `LogPacket` to the RTOS queue. If the queue is full, increments `logDropCount` and logs a warning.

### 9.2 `createPSRAMBackedQueue()`
Creates an RTOS queue using PSRAM for storage and internal RAM for the control structure. Falls back to standard `xQueueCreate` if PSRAM allocation fails. Tracks allocation diagnostics (`psramAllocFailures`, `controlAllocFailures`, `queueCreateFailures`, `psramAllocatedBytes`).

### 9.3 SD Card: `SDInit()`
- Mounts SD via `SD.begin(SD_CS, *hspi)` (HSPI bus)
- Creates `/flight_log.csv` with CSV header if needed
- Creates `/system_log.txt` if needed
- Tracks `sdInitAttempts`, `sdInitFailures`

### 9.4 `TelemetryTask()` (Core 0, priority 2, runs every 50ms)
```cpp
while (gpsSerial.available()) gps.encode(gpsSerial.read());
// Update sharedTelemetry with lat/lng under mutex
```
Runs GPS data encoding continuously. Updates `sharedTelemetry.latitude/longitude/gpsUpdated` but NOT speed (speed is handled in main loop).

### 9.5 `SDWriterTask()` (Core 0, priority 1)
Blocks on `xQueueReceive(sdLogQueue, timeout=1000ms)`. On receiving a packet:
- Opens `/flight_log.csv` in append mode
- Writes all 40+ CSV fields using `file.printf()`
- If write fails, re-initializes SD card

### 9.6 `updateSharedTelemetry()`
Writes current flight data (roll, pitch, yaw, altitude, velocity, phase, mode, armed, sensor status) into `sharedTelemetry` under mutex. Called from main loop every iteration.

---

## 10. Launch Sequence & Safety (`Launchsequence.h`)

### 10.1 5V Enable
```cpp
enable5v(bool onOff)
```
Controls GPIO 3 to enable/disable 5V power rail. Tracks state in `Enabled5V`.

### 10.2 Pyro Firing
```cpp
firePyro(int pyroPin, int pulseDurationMs)
```
Static arrays track up to 70 pins. On first call with a given pin, sets it HIGH and records start time. On subsequent calls, checks if `pulseDurationMs` has elapsed and sets LOW. This means `firePyro()` must be called repeatedly from the loop to complete a pulse — it's a time-sliced non-blocking pyro controller.

### 10.3 Apogee Pyro (`fire_apogee_pyro()`)
Three safety checks before firing:
1. **System must be armed** (`systemArmed == true`)
2. **Altitude must be above safety floor** (`filter_alt > minAltitudeForParachuteMeters = 150m`)
3. **Must be in COAST phase** (prevents firing on pad or during boost)

If any check fails, logs an error and returns without firing. Has a `safetyOverride` parameter to bypass altitude check. On success, calls `firePyro(parachutePyroPin, 75ms)`.

### 10.4 Instrument Check (`instrumentCheck()`)
Returns 0 for success, -1 for failure. Checks:
- **Critical:** BMP580 (must read), BNO085 (must be initialized, no reset)
- **Critical:** Servo driver (PCA9685 `begin()` called)
- **Optional:** GPS (only if `enforceGPSLock == true`)
- **Warning:** SD card (only if `enforceSDCard == true`)

### 10.5 Arm/Disarm (`arm()`)
```cpp
arm(true)   // Arms: runs instrumentCheck(), enables 5V, sets pyroArmPin HIGH, sets systemArmed=true
arm(false)  // Disarms: sets systemArmed=false, optionally disables 5V
```

---

## 11. Web Server & Ground Control (`web_server.h`)

### 11.1 WiFi Access Point
- SSID: `ISAAC_AVIONICS`
- Password: `12345678`
- IP: 192.168.4.1 (default softAP IP)
- Web server on port 80
- Uses **ElegantOTA** for over-the-air firmware updates

### 11.2 `WifiServerTask()` (Core 0, priority 1)
Runs the web server. **Phase-aware route switching:**

| Flight Phase | Active Routes | Description |
|-------------|---------------|-------------|
| TRANSPORT | Full dashboard | All routes (see below) |
| PAD | Full dashboard | All routes |
| READY | Minimal dashboard | Abort, Launch, limited telemetry |
| BOOST | **WiFi shut down** | Task yields, no handling |
| COAST | Full dashboard | (but WiFi already off) |

When transitioning PAD→READY, the server is destroyed and replaced with a minimal one (lower overhead during critical phase). When READY→PAD (abort), full server is restored.

### 11.3 HTTP Endpoints

#### Telemetry & Data
| Route | Method | Handler | Description |
|-------|--------|---------|-------------|
| `/` | GET | `handleRoot` | Full GCS HTML page |
| `/data` | GET | `handleGetData` | Full telemetry JSON |
| `/ready_data` | GET | `handleReadyData` | Minimal telemetry (READY phase) |
| `/set_time` | GET | `handleSetTime` | Wall clock seed from browser |
| `/serial_log` | GET | `handleSerialLog` | Serial log buffer content |

**`handleGetData()`** returns a ~1.6KB JSON blob with:
- roll, pitch, yaw, raw_alt, filtered_alt, vel_z
- lat, lng, gps_updated
- phase, system_mode, armed, sensors_ok
- airspeed, kalman_P, baro_alpha
- All 8 PID outputs and servo angles
- Active gains Kp/Ki/Kd
- baro_pressure, baro_temp
- Quaternion and acceleration
- dt, log_drops
- IMU/baro/GPS/SD health booleans
- servo_override state
- Profile status (active, name, step, progress, setpoints)

#### System Control
| Route | Method | Handler | Description |
|-------|--------|---------|-------------|
| `/arm` | POST | `handleArm` | Arm system (ACTIVE_PAD only) |
| `/disarm` | POST | `handleDisarm` | Disarm system |
| `/launch` | POST | `handleLaunch` | PAD/READY → READY with armed |
| `/abort` | POST | `handleAbort` | READY → PAD (abort sequence) |
| `/set_mode` | POST | `handleSetSystemMode` | Switch TRANSPORT/ACTIVE_PAD |

**`handleLaunch()`:** Sets system armed, disables servo override, transitions to READY phase. Returns 403 if not in ACTIVE_PAD or already flying.

**`handleAbort()`:** Returns to PAD phase, disarms, releases servo override. Only works from READY phase.

**`handleSetSystemMode()`:** Switching to ACTIVE_PAD calls `initInstruments()` and `calibrateGroundAltitude()`.

#### Servo Control
| Route | Method | Handler | Description |
|-------|--------|---------|-------------|
| `/servo` | POST | `handleSetServo` | Set single servo (ch 0-7, angle 60-120°) |
| `/servo_all` | POST | `handleSetAllServos` | Set all servos to same angle |
| `/servo_release` | POST | `handleReleaseServos` | Release manual override |
| `/servo_test` | POST | `handleServoTest` | Sweep test 90→120→60→90° |

Servo override only works in PAD and TRANSPORT phases (returns 403 during flight).

#### Flight Profiles
| Route | Method | Handler | Description |
|-------|--------|---------|-------------|
| `/profile_list` | GET | `handleProfileListAll` | List all profiles (flash + SD) |
| `/profile_start` | POST | `handleProfileStart` | Start profile by name, index, or raw JSON |
| `/profile_stop` | POST | `handleProfileStop` | Abort running profile |
| `/profile_status` | GET | `handleProfileStatus` | Active profile state |
| `/profile_upload` | POST | `handleProfileUpload` | Upload JSON to SD + register |
| `/profile_delete` | POST | `handleProfileDelete` | Delete SD profile file |

**`handleProfileStart()`** supports three modes:
1. `name` parameter — looks up by name (flash then SD)
2. `index` parameter — flash profile index
3. Raw JSON body — parses, registers, and starts inline

#### Data Logs
| Route | Method | Handler | Description |
|-------|--------|---------|-------------|
| `/download_log` | GET | `handleDownloadLog` | Download `/flight_log.csv` |
| `/delete_log` | POST | `handleDeleteLog` | Erase and recreate with header |

#### OTA
| Route | Method | Handler |
|-------|--------|---------|
| `/update` | (from ElegantOTA) | Firmware update |

### 11.4 `shutdownWiFiNetwork()`
Called at BOOST phase onset:
1. Sets `wifiActive = false`
2. Stops and deletes the web server
3. Disconnects and powers off WiFi (`WiFi.mode(WIFI_OFF)`)
4. Plays a descending tone

### 11.5 READY Phase Minimal Page
During READY, a stripped-down HTML page (`READY_INDEX_HTML`) is served. It shows:
- Filtered/raw altitude, vertical velocity, acceleration
- Instrument status
- ABORT button (large, prominent)
- LAUNCH button
- Serial log feed

---

## 12. Web UI Dashboard (`index.html`)

A single-page GCS (Ground Control Station) dashboard served as a raw string literal (`INDEX_HTML`), included in `web_server.h` via `#include "index.html"`.

### Layout Sections
1. **Header** — "ISAAC-L GCS" with arm status pill and override badge
2. **Status Row** — System mode, flight phase, loop dt, log drops, instrument health
3. **Flight Telemetry** (collapsible) — altitude, Vz, airspeed, attitude (R/P/Y)
4. **Advanced Telemetry** (collapsible) — quaternion, acceleration, baro pressure/temp, Kalman P, baro alpha, active gains, GPS, all 8 PID outputs
5. **Servo Positions** (collapsible) — all 8 current angles
6. **Avionics Control** — Transport/Active Pad toggle, Arm/Disarm/Launch buttons
7. **Servo Control** (collapsible) — individual sliders and angle displays, center/full up/full down/sweep/release buttons
8. **Flight Profile** — profile selector dropdown, start/stop buttons, progress bar, status display, JSON upload form
9. **Serial Log** — scrollable log viewer with clear and auto-scroll toggle
10. **Data Logs** — download/erase flight_log.csv
11. **System Maintenance** — OTA firmware update button

### JavaScript Logic
- `updateTelemetry()` — fetches `/data` every 250ms, populates all display fields
- `fetchSerialLog()` — fetches `/serial_log` every 500ms, appends to log box
- `updateProfileStatus()` — fetches `/profile_status` every 500ms, updates status/progress bar
- `loadProfileList()` — fetches `/profile_list` to populate dropdown
- `setSystemMode(mode)` — POST to `/set_mode`
- `sendCommand(ep)` — generic POST for arm/disarm/launch
- `setServo(ch, angle)` — POST to `/servo`, updates angle display and PWM calculation
- `setAllServos(angle)` — POST to `/servo_all`
- `runSweepTest()` — confirmed POST to `/servo_test`
- `releaseServos()` — POST to `/servo_release`
- `triggerLaunchSequence()` — confirmed POST to `/launch`
- `confirmErase()` / `confirmOTA()` — confirmed destructive actions
- `uploadProfile()` — POST name+JSON to `/profile_upload`, refreshes list
- `sendClientTime()` — seeds system clock via `/set_time?ts=` on load
- Server-discovered phase transitions cause page reload if no longer in PAD
- Phase-specific CSS styling (pill colors for each flight phase)

### Phase Visual Styling
| Phase | CSS Pill Class | Color |
|-------|---------------|-------|
| TRANSPORT | pill-transport | Gray |
| PAD | pill-pad | Amber/warning |
| READY | pill-ready | Orange |
| BOOST | pill-boost | Red/danger |
| COAST | pill-coast | Purple |
| DESCENT | pill-descent | Green/success |

---

## 13. Debug CLI (`debug_cli.h`)

### Activation
Only runs when `debugMode == true`. Enabled via `#define` in `globals.h`. Called from `loop()`:
```cpp
if (debugMode == true) debugCLI_loop();
```

### Buffer
A `String debugCmdBuffer` accumulates characters. Backspace is handled. Commands are dispatched on newline.

### Commands

**General:**
| Command | Handler | Description |
|---------|---------|-------------|
| `help` | `debugCLI_printHelp()` | Show command list |
| `status` | `debugCLI_printStatus()` | System state + diagnostics |

**Sensor Read:**
| Command | Handler | Description |
|---------|---------|-------------|
| `read gps` | `debugCLI_readGPS()` | GPS lat/lng/speed/sats/HDOP |
| `read imu` | `debugCLI_readIMU()` | Quat, accel, Euler angles |
| `read baro` | `debugCLI_readBaro()` | Pressure (hPa), temp (°C), altitude (m) |

**Bus Test:**
| Command | Handler | Description |
|---------|---------|-------------|
| `test i2c` | `debugCLI_scanI2C()` | Scan I2C addresses 8-119 |
| `reset i2c` | `debugCLI_resetI2C()` | Wire.end() + Wire.begin() |
| `reset spi` | `debugCLI_resetSPI()` | hspi->end() + hspi->begin() |

**System:**
| Command | Handler | Description |
|---------|---------|-------------|
| `init instruments` | `debugCLI_initInstruments()` | Reinit IMU + baro |
| `sd info` / `read sd` | `debugCLI_sdInfo()` | Card size + file listing |
| `servo <ch> <ang>` | inline | Set servo (0-7, 60-120°) |
| `mode <transport\|active_pad>` | inline | Switch system mode |

**Profile:**
| Command | Handler |
|---------|---------|
| `profile list` | `debugCLI_printHelp` sub-block |
| `profile start <name>` | inline |
| `profile stop` | inline |
| `profile next` | inline |
| `profile reload` | inline |
| `profile status` | inline |

---

## 14. Audio Feedback (`buzzers.h`)

### LEDC Buzzer
- GPIO 38, LEDC channel 0
- Initialized with 12-bit resolution at 5kHz base frequency
- Fallback frequencies: 4kHz/12-bit → 2kHz/11-bit
- Controlled by `enableBuzzer` flag

### Functions

**`playTone(freq, duration_ms)`:** If buzzer enabled, initializes LEDC and calls `tone()`.

**`startupChime()`:** C-E-G-C arpeggio (523→659→784→1047 Hz)

**`errorBeep()`:** 200Hz for 1 second

**`armChime()`:** A-A-A' triple tone (880→880→1760 Hz)

**`disarmChime()`:** Descending A'-A (1760→880 Hz)

**`modeChangeChime()`:** D-A-D' arpeggio (587→880→1175 Hz)

**`armedAlarm()`:** A pulsing alarm at 1760Hz (100ms on, 200ms off). Intended to run continuously during armed state. Uses static local variables for state tracking.

---

## 15. Complete Data Flow Summary

```
┌─────────────────────────────────────────────────────────────────────┐
│                        ESP32-S3 Core 1 (loop)                       │
│                                                                     │
│  ┌──────────┐   ┌──────────────┐   ┌──────────────┐                │
│  │ BNO085   │   │ BMP580       │   │ GPS (UART2)  │                │
│  │ (SPI)    │   │ (SPI)        │   │ (9600 baud)  │                │
│  │ 100Hz    │   │ 50Hz         │   │ 10Hz         │                │
│  │ Quat+Acc │   │ Pressure+Temp│   │ Pos+Speed    │                │
│  └────┬─────┘   └──────┬───────┘   └──────┬────────┘               │
│       │                │                  │                         │
│       ▼                ▼                  ▼                         │
│  ┌───────────────────────────────────────────────────┐              │
│  │               Sensor Fusion Engine                │              │
│  │  updateRocketFusion()                             │              │
│  │  • Earth-frame accel projection                   │              │
│  │  • Complementary filter (phase-adaptive alpha)    │              │
│  │  • Vertical velocity + filtered altitude          │              │
│  │  • Kalman airspeed prediction                    │              │
│  │  • State machine evaluation                      │              │
│  └─────────────────────┬─────────────────────────────┘              │
│                        │                                             │
│                        ▼                                             │
│  ┌───────────────────────────────────────────────────┐              │
│  │            Flight State Machine                   │              │
│  │  process_flight_state_machine()                    │              │
│  │  PAD → READY → BOOST → COAST → DESCENT            │              │
│  │  (accelerometer & altitude triggered)              │              │
│  └─────────────────────┬─────────────────────────────┘              │
│                        │                                             │
│                        ▼                                             │
│  ┌───────────────────────────────────────────────────┐              │
│  │         Flight Profile Engine                     │              │
│  │  profileEngine.update()                            │              │
│  │  • Evaluates step triggers                        │              │
│  │  • Computes quintic S-curve interpolation         │              │
│  │  • Output: spRoll, spPitch, spYaw                 │              │
│  └─────────────────────┬─────────────────────────────┘              │
│                        │                                             │
│                        ▼                                             │
│  ┌───────────────────────────────────────────────────┐              │
│  │          PID Stabilization Loop                   │              │
│  │  userFlightStabilizationLoop()                    │              │
│  │  • Compute attitude errors                        │              │
│  │  • Weight by surface matrix                       │              │
│  │  • Apply gain scheduling (speed-based)            │              │
│  │  • 8 QuickPID controllers compute outputs         │              │
│  │  • Map to 8 servo angles (center ±30°)            │              │
│  └─────────────────────┬─────────────────────────────┘              │
│                        │                                             │
│                        ▼                                             │
│  ┌───────────────────────────────────────────────────┐              │
│  │              PCA9685 PWM Driver                   │              │
│  │  UserSpace::writeServoAngle(ch, angle)             │              │
│  │  • Map 0-180° to 150-600μs pulse                  │              │
│  │  • I2C to 8 servo channels                        │              │
│  └───────────────────────────────────────────────────┘              │
│                                                                     │
│  ┌───────────────────────────────────────────────────┐              │
│  │         Logging (every 10ms via RTOS Queue)        │              │
│  │  pushLogPacket() → sdLogQueue (PSRAM-backed)       │              │
│  └───────────────────────────────────────────────────┘              │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│                        ESP32-S3 Core 0 (RTOS)                       │
│                                                                     │
│  ┌──────────────────────┐  ┌──────────────────────┐                 │
│  │  TelemetryTask       │  │  SDWriterTask        │                 │
│  │  (priority 2, 50ms)  │  │  (priority 1)        │                 │
│  │  • GPS encode        │  │  • Dequeue LogPacket │                 │
│  │  • Update sharedTele │  │  • Write CSV to SD   │                 │
│  └──────────────────────┘  └──────────────────────┘                 │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │  WifiServerTask (priority 1)                                 │   │
│  │  • WiFi AP "ISAAC_AVIONICS"                                  │   │
│  │  • WebServer on port 80                                      │   │
│  │  • Phase-aware route switching (PAD→READY minimal)           │   │
│  │  • Telemetry: /data JSON, /serial_log                        │   │
│  │  • Control: /arm, /disarm, /launch, /abort, /set_mode        │   │
│  │  • Servos: /servo, /servo_all, /servo_release, /servo_test   │   │
│  │  • Profiles: /profile_*                                      │   │
│  │  • Logs: /download_log, /delete_log                          │   │
│  │  • OTA: /update (ElegantOTA)                                 │   │
│  └──────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
```

### Key Equations Reference

**Earth-frame vertical acceleration:**
```
a_z_earth = 2(qx·qz - qw·qy)·ax + 2(qy·qz + qw·qx)·ay + (1 - 2(qx² + qy²))·az
```

**Complementary filter:**
```
V_z = alpha · (V_z + a_z·dt) + (1 - alpha) · V_baro
filter_alt += V_z · dt
```

**PID input (per servo i):**
```
input[i] = W_roll[i] · (roll - spRoll) + W_pitch[i] · (pitch - spPitch) + W_yaw[i] · (yaw - spYaw)
```

**Servo output:**
```
angle[i] = 90° + clamp(PID_output[i], -30°, 30°)
```

**Quintic blend:**
```
s(t) = 10t³ - 15t⁴ + 6t⁵  for t ∈ [0, 1]
setpoint = start + (target - start) · s(t)
```

**Gain interpolation (at velocity v):**
```
t = (v - v_i) / (v_{i+1} - v_i)
Kp = Kp_i + t · (Kp_{i+1} - Kp_i)
```
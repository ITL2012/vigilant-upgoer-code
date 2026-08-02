#ifndef GUIDANCE_GLOBALS_H
#define GUIDANCE_GLOBALS_H

#include <Arduino.h>
#include <atomic>
#include <Adafruit_PWMServoDriver.h>
#include <TinyGPSPlus.h>

// ============================================================================
// BROWNOUT RECOVERY — BUILD-TIME FLAGS (immutable; static constexpr)
// ============================================================================
// These four flags are READ-ONLY constants baked in at compile time so nothing
// (not the dashboard, not CLI, not boot recovery) can accidentally mutate
// them at runtime. To change them: edit this file and reflash.
//
//   enableBrownoutRecovery — on reset mid-flight, apply recent (<60s) snapshot
//   enableDualWriteCache  — also mirror SD log packets to NVS (flash wear!)
//   enableBackupChute     — descent-rate backup chute auto-fire
//   flightCacheValid      — RUNTIME bool (defined in main.cpp); set true after
//                           a successful in-flight snapshot load at boot

int BMP580good;
int BNO080good;
int GPSgood;
int SDgood;
int wifiGood;
int servoGood;

float current_pitch;
float current_roll;
float current_yaw;

static constexpr bool debugMode = false;

// Brownout recovery / pyro backup flags — see top-of-file comment.
static constexpr bool enableBackupChute      = false;
static constexpr bool enableBrownoutRecovery = true;   // resume in-flight state on reboot/reset
static constexpr bool enableDualWriteCache   = false;  // dual-write log packets to NVS (flash wear!)



static constexpr bool enableBuzzer = true;
static constexpr bool stabilizationMode = true;
static constexpr bool enforceGPSLock = false;
static constexpr bool enforceSDCard = true;
static constexpr bool enableWaypointGuidance = false;

static constexpr int pyro1Pin = 999;  // 999 = no channel wired
static constexpr int pyro2Pin = 999;
static constexpr int pyro3Pin = 999;

static constexpr int Enable5VPin = 3;
bool Enabled5V = false;

// ============================================================================
// PYRO CHANNEL CONFIGURATION
// ============================================================================
// Per-pin unified configuration. Each of the 3 hardware pyro channels can be
// independently assigned a role, pulse duration, enabled/disabled, and an
// optional trigger parameter used by aux events (e.g. staging delay or main
// deploy altitude). Roles are code-configured here at build time (NOT web
// reassignable) — to change which pin does what job, edit the alias lines
// below. A pin value of 999 means "not connected".

enum PyroRole {
    ROLE_NONE = 0,          // Channel disabled / unassigned
    ROLE_PRIMARY_CHUTE = 1, // Fires automatically at apogee
    ROLE_BACKUP_CHUTE = 2,  // Fires automatically if primary fails (descent-rate heuristic)
    ROLE_AUX_STAGING = 3,   // Fires automatically after auxTriggerValue_ms post-liftoff (extra staging / air-start)
    ROLE_AUX_MAIN    = 4,   // Fires automatically when filter_alt <= auxTriggerValue_m (low-alt main chute)
    ROLE_MANUAL      = 5    // Only operator-fires it (web/CLI); never automatic
};

struct PyroChannelConfig {
    int pin;                       // physical GPIO (999 = not connected)
    unsigned long pulseMs;         // firing pulse duration
    PyroRole role;                 // assigned role (build-time configurable)
    bool enabled;                  // disable a channel without re-wiring
    unsigned long auxTriggerValue; // ms for STAGING, m (×1000 int) for MAIN; 0 = use default
};

// Default per-channel config. Edit these aliases (and the helpers below) to
// rebind which physical pin handles which role. Set pin=999 to omit a channel.
static constexpr int        parachutePyroPin        = pyro1Pin; // backwards-compat alias
static constexpr int        backupChutePyroPin     = pyro2Pin; // alias for backup chute role
static constexpr int        auxiliaryPyroPin        = pyro3Pin; // alias for aux role
static constexpr unsigned long parachutePulseDurationMs = 75;
static constexpr unsigned long backupPulseDurationMs    = 75;
static constexpr unsigned long auxPulseDurationMs        = 100;
static constexpr unsigned long minAltitudeForParachuteMeters = 150;

// runtime-toggleable: turn OFF backup-charge logic — NOTE: enableBackupChute is
// now static constexpr (read-only). Edit globals.h to change.
// (no extern here; defined as static constexpr at top of file)

// Backup-charge descent-rate heuristic constants
static constexpr float    DESCENT_FAIL_VZ_THRESHOLD       = -15.0f; // m/s; if descending faster after eval window -> backup
static constexpr unsigned long POST_FIRE_EVAL_WINDOW_MS   = 1500;   // observe V_z this long after primary fire
static constexpr float    EXPECTED_CANOPY_DESCENT_MPS     = 5.0f;   // nominal chute descent (reference only)

// Aux trigger default values (overridable per-channel via auxTriggerValue)
static constexpr unsigned long DEFAULT_AUX_STAGING_DELAY_MS = 2000;  // post-liftoff delay for ROLE_AUX_STAGING
static constexpr float        DEFAULT_AUX_MAIN_ALT_M       = 50.0f; // deploy altitude for ROLE_AUX_MAIN (== DEPLOYMENT_ALTITUDE)

// Channel registry (defined in main.cpp). Index 0..2 == pyro1..3.
extern PyroChannelConfig pyroChannels[3];

// Find first channel index currently assigned to a given role, or -1 if none.
int pyroChannelForRole(PyroRole r);

// ============================================================================
// SHARED FLIGHT DIAGNOSTICS (for brownout cache and telemetry)
// ============================================================================
extern float max_altitude;

// ============================================================================
// BROWNOUT RECOVERY — FLIGHT STATE SNAPSHOT
// ============================================================================
// Three-tier persistence:
//   - RTC slow memory (RTC_DATA_ATTR): updated every loop, no wear. Fastest.
//   - NVS / Preferences (flash): updated on phase transitions + pyro events.
//   - SD card (/flight_state.bin): same triggers as NVS, large file.
// At boot, if a snapshot is <BROWNOUT_CACHE_VALIDITY_S seconds old AND shows
// the controller was armed/in-flight, the snapshot is re-applied and the
// state machine resumes from the cached phase. This survives a brownout /
// panic reset mid-flight (NOT a full power-cycle).
//
// PID accumulators are stored in the snapshot, but written only on phase
// transitions to avoid excessive flash wear (they ride RTC per-loop instead).

static constexpr unsigned long BROWNOUT_CACHE_VALIDITY_S = 60;  // seconds
static constexpr uint32_t       BROWNOUT_CACHE_MAGIC        = 0xB04C1A57;  // "BO-CAST" magic
static constexpr uint32_t       BROWNOUT_CACHE_VERSION      = 1;
static constexpr size_t         BROWNOUT_NUM_PID             = 8;

struct FlightCache {
    uint32_t magic;             // BROWNOUT_CACHE_MAGIC if valid
    uint32_t version;           // schema version for forward-compat
    uint64_t snapshotEpoch_ms;  // wall-clock of snapshot (or 0 if no RTC)

    // Mode + phase + armed
    uint8_t  systemMode;        // SystemMode
    uint8_t  flightPhase;       // FlightPhase
    uint8_t  armed;             // 0/1

    // Timings
    uint64_t liftoffEpoch_ms;   // epoch at liftoff (liftoff_time_ms preserved across reboot)
    uint32_t liftoffAgeMs;      // fallback: age at snapshot if no epoch

    // Flight state
    float    filter_alt;
    float    V_z;
    float    max_altitude;
    float    baseline_altitude; // ground ref restored
    float    qnh_pressure;       // sea-level pressure reference (was missing — bug fix)

    // Pyro
    uint8_t  pyroStateArr[3];   // PyroState per channel
    uint32_t pyroFiredAtMs[3];  // ms timestamps of fire
    uint8_t  pyroAttempted[3];  // 0/1
    uint8_t  backupEnabled;     // 0/1 copy of enableBackupChute

    // PID integrators (resume stabilization continuity)
    float    pid_iTerm[BROWNOUT_NUM_PID];

    // Diagnostics
    uint32_t bootCount;
    uint32_t lastResetReason;   // esp_reset_reason_t
};

extern bool        flightCacheValid;     // set true after a successful loadFromAny
extern FlightCache flightCacheRestored; // copy restored at boot (for logging)
extern uint32_t    bootCount;
extern uint32_t    rtcLastResetReason;   // NOINIT RTC var — survives reboot only

// Save current flight state into all three stores (RTC always; NVS+SD optional
// via writeFlash == true). Called on phase transitions & pyro events.
void flightCacheSave(bool writeFlash);

// ============================================================================
// BAROMETER CALIBRATION (always restored on boot from NVS)
// ============================================================================
struct BaroCalibration;
extern BaroCalibration baroCal;
extern const char *baroCalSource;
void baroCalibrationSampleAndCompute(BaroCalibration &out);
void baroCalibrationSave(const BaroCalibration &c);
bool baroCalibrationLoad(BaroCalibration &out);
void baroCalibrationInvalidate();
bool baroCalibrationRestoreOnBoot();
void calibrateGroundAltitude();

// Per-loop RTC-only snapshot (no flash wear).
void flightCacheSnapshotRTC();

// Load the most-recent snapshot from NVS (preferred) or SD (fallback). Returns
// true if a valid in-flight snapshot was loaded and applied.
bool flightCacheLoadAndApply();

// Invalidate the snapshot (called when transitioning to TRANSPORT/RECOVERY so
// the next cold boot doesn't resume a long-finished flight).
void flightCacheInvalidate();

// Restore PID integrators / pyroState from a loaded FlightCache.
void flightCacheRestorePyros(const FlightCache &c);
void flightCacheRestorePID(const FlightCache &c);

static constexpr bool enableAutoMotorIgnition = false;
static constexpr float motorPyroPin = 999;
static constexpr unsigned long motorIgnitionPulseDurationMs = 100;

static constexpr bool pyroArmPin = 7;

static constexpr bool doFinTestOnCountdown = true;

// VSPI does BNO and BMP
static constexpr int VSPI_MOSI = 11;
static constexpr int VSPI_MISO = 13;
static constexpr int VSPI_CLK  = 12;
static constexpr int BMP_CS  = 10;
static constexpr int BMP_INT = 18;
static constexpr int BNO_CS  = 9;
static constexpr int BNO_INT = 8;
static constexpr int BNO_RST = 14;


// SD card in SDMMC mode (DAT0, CMD, CLK, DAT3 per PCB schematic)
static constexpr int SDMMC_CLK  = 42;
static constexpr int SDMMC_CMD  = 15;
static constexpr int SDMMC_D0   = 47;
static constexpr int SDMMC_D3   = 23;
// HSPI pins (no longer used for SD — SD uses SDMMC now)
static constexpr int HSPI_MISO = 37;
static constexpr int HSPI_MOSI = 35;
static constexpr int HSPI_CLK  = 36;

static constexpr int I2C_SDA      = 1;
static constexpr int I2C_SCL      = 2;
static constexpr unsigned long I2C_SPEED = 400000UL;
static constexpr uint8_t PCA9685_ADDR = 0x40;

static constexpr int GPS_BAUD = 9600;
static constexpr int GPS_RX2 = 16;
static constexpr int GPS_TX2 = 17;
static constexpr int BUZZER_PIN = 38;

static const char AP_SSID[] = "ISAAC_AVIONICS";
static const char AP_PASSWORD[] = "12345678";

static constexpr int PWM_NEUTRAL = 307;
static constexpr int PWM_RANGE   = 100;
static const char LOG_FILE_PATH[] = "/flight_log.csv";
static const char SYSTEM_LOG_FILE_PATH[] = "/system_log.txt";
static constexpr int LOG_QUEUE_LEN = 5000;
static constexpr size_t LOG_MESSAGE_BUFFER_SIZE = 256;
static constexpr size_t SERIAL_LOG_BUF_SIZE = 4096;
static constexpr size_t LOG_CACHE_SIZE = 8192;
static constexpr float LAUNCH_ACCEL_THRESHOLD = 25.0f;
static constexpr float APOGEE_VEL_THRESHOLD = -0.5f;
static constexpr unsigned long IMU_TIMEOUT_MS = 500;
static constexpr float DEPLOYMENT_ALTITUDE = 50.0f;
static constexpr unsigned long WDT_TIMEOUT_S = 5;
static constexpr unsigned long COAST_LOCKOUT_MS = 1500;
static constexpr unsigned long APOGEE_BACKUP_TIMEOUT_MS = 10000;

static constexpr double METERS_TO_FEET = 3.280839895013123;

enum SystemMode {
    MODE_TRANSPORT,
    MODE_PAD,
    MODE_ACTIVE_PAD
};

enum FlightPhase {
    TRANSPORT,
    PAD,
    READY,
    BOOST,
    COAST,
    DESCENT,
    RECOVERY
};

enum LogDestination {
    LOG_SERIAL = 0,
    LOG_SD = 1,
    LOG_BOTH = 2
};

enum LogSeverity {
    LOG_INFO = 0,
    LOG_WARN = 1,
    LOG_ERROR = 3
};

struct TelemetryData {
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float raw_altitude = 0.0f;
    float filtered_altitude = 0.0f;
    float velocity_z = 0.0f;
    double latitude = 0.0;
    double longitude = 0.0;
    bool gpsUpdated = false;
    uint8_t phase = 0;
    uint8_t system_mode = 0;
    bool armed = false;
    bool sensors_ok = false;
};

struct LogPacket {
    uint32_t timestamp_ms;
    uint64_t epoch_ms;
    float roll;
    float pitch;
    float yaw;
    float raw_alt;
    float filtered_alt;
    float vel_z;
    float accel_x;
    float accel_y;
    float accel_z;
    uint8_t current_phase;
    float servo0;
    float servo1;
    float servo2;
    float servo3;
    float servo4;
    float servo5;
    float servo6;
    float servo7;
    float pid0;
    float pid1;
    float pid2;
    float pid3;
    float pid4;
    float pid5;
    float pid6;
    float pid7;
    float gain_kp;
    float gain_ki;
    float gain_kd;
    float airspeed;
    float kalman_P;
    float baro_alpha;
    float qx;
    float qy;
    float qz;
    float qw;
    float baro_pressure;
    float baro_temp;
    float dt;
};

extern float latestServoAngles[8];
extern float latestPIDOutputs[8];
extern float latestActiveGains[3];
extern float latestAirspeed;
extern float latestKalmanP;
extern float latestBaroAlpha;
extern float latestBaroPressure;
extern float latestBaroTemp;
extern float latestDt;

extern float latestQx, latestQy, latestQz, latestQw;
extern float latestAx, latestAy, latestAz;

extern volatile bool servoOverrideActive;
extern volatile float servoOverrideAngles[8];

extern std::atomic<SystemMode> currentSystemMode;
extern std::atomic<FlightPhase> currentPhase;
extern std::atomic<bool> systemArmed;
extern std::atomic<bool> wifiActive;

// Hardware/global peripheral instances (defined in one .cpp only)
extern Adafruit_PWMServoDriver pwm;
extern TinyGPSPlus gps;

extern SemaphoreHandle_t telemetryMutex;
extern volatile TelemetryData sharedTelemetry;

extern float V_z;
extern float filter_alt;
extern float baseline_altitude;
extern float previous_altitude;
extern float qnh_pressure;

extern unsigned long lastMicros;
extern unsigned long lastLogTime;
extern std::atomic<unsigned long> lastIMUReport_ms;
extern std::atomic<uint8_t> imuCalStatus; // BNO085 dynamic-cal accuracy 0-3 (3 = fully calibrated)
extern std::atomic<unsigned long> liftoff_time_ms;
extern std::atomic<uint32_t> logDropCount;

extern bool sdReady;

// Pre-SD-init log cache
extern char logCache[LOG_CACHE_SIZE];
extern volatile size_t logCacheHead;
extern volatile size_t logCacheTail;
extern volatile bool logCacheOverflow;

extern SemaphoreHandle_t logCacheMutex;

void write(int destination, int severity, const char *format, ...);
void flushLogCacheToSD();
void initLogCache();

void userCustomSetup();

void convertQuaternionToEuler(float r, float i, float j, float k) {
    float ysqr = j * j;

    float t0 = +2.0f * (r * i + j * k);
    float t1 = +1.0f - 2.0f * (i * i + ysqr);
    current_pitch = atan2(t0, t1) * RAD_TO_DEG;

    float t2 = +2.0f * (r * j - k * i);
    t2 = t2 > 1.0f ? 1.0f : t2;
    t2 = t2 < -1.0f ? -1.0f : t2;
    current_roll = asin(t2) * RAD_TO_DEG;

    float t3 = +2.0f * (r * k + i * j);
    float t4 = +1.0f - 2.0f * (ysqr + k * k);
    current_yaw = atan2(t3, t4) * RAD_TO_DEG;
}

#endif // GUIDANCE_GLOBALS_H

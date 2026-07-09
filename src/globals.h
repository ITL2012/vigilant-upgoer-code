#ifndef GUIDANCE_GLOBALS_H
#define GUIDANCE_GLOBALS_H

#include <Arduino.h>
#include <atomic>

int BMP580good;
int BNO080good;
int GPSgood;
int SDgood;
int wifiGood;
int servoGood;

float current_pitch;
float current_roll;
float current_yaw;

static constexpr bool debugMode = true;

static constexpr bool enableBuzzer = true;
static constexpr bool stabilizationMode = true;
static constexpr bool enforceGPSLock = false;
static constexpr bool enforceSDCard = true;
static constexpr bool enableWaypointGuidance = false;

static constexpr int pyro1Pin = 999;
static constexpr int pyro2Pin = 999;
static constexpr int pyro3Pin = 999;

static constexpr int Enable5VPin = 3;
bool Enabled5V = false;

static constexpr int parachutePyroPin = pyro1Pin; // CHANGE TO REAL PIN
static constexpr unsigned long parachutePulseDurationMs = 75;
static constexpr unsigned long minAltitudeForParachuteMeters = 150;

static constexpr bool enableAutoMotorIgnition = false;
static constexpr float motorPyroPin = 999;
static constexpr unsigned long motorIgnitionPulseDurationMs = 100;



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


static constexpr int SD_CS     = 21;
static constexpr int HSPI_MISO = 37;
static constexpr int HSPI_MOSI = 35;
static constexpr int HSPI_CLK  = 36;

static constexpr int I2C_SDA      = 1;
static constexpr int I2C_SCL      = 3;
static constexpr unsigned long I2C_SPEED = 400000UL;
static constexpr uint8_t PCA9685_ADDR = 0x40;

static constexpr int GPS_RX2 = 16;
static constexpr int GPS_TX2 = 17;
static constexpr int BUZZER_PIN = 38;

static const char AP_SSID[] = "ISAAC_AVIONICS";
static const char AP_PASSWORD[] = "12345678";

static constexpr int PWM_NEUTRAL = 307;
static constexpr int PWM_RANGE   = 100;
static const char LOG_FILE_PATH[] = "/flight_log.csv";
static const char SYSTEM_LOG_FILE_PATH[] = "/system_log.txt";
static constexpr int LOG_QUEUE_LEN = 500;
static constexpr size_t LOG_MESSAGE_BUFFER_SIZE = 256;
static constexpr float LAUNCH_ACCEL_THRESHOLD = 25.0f;
static constexpr float APOGEE_VEL_THRESHOLD = -0.5f;
static constexpr unsigned long IMU_TIMEOUT_MS = 500;
static constexpr float DEPLOYMENT_ALTITUDE = 50.0f;
static constexpr unsigned long WDT_TIMEOUT_S = 5;

static constexpr double METERS_TO_FEET = 3.280839895013123;

enum SystemMode {
    MODE_TRANSPORT,
    MODE_ACTIVE_PAD
};

enum FlightPhase {
    TRANSPORT,
    PAD,
    READY,
    BOOST,
    COAST,
    DESCENT
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
extern volatile uint32_t logDropCount;

extern bool sdReady;

// Buzzer functions
void playTone(unsigned int freq, unsigned long duration_ms);
void initBuzzerLEDC();

// System epoch bookkeeping (ms since 1970-01-01 UTC)
extern std::atomic<unsigned long long> systemBaseEpochMs;
extern std::atomic<unsigned long> systemBaseMillis;

void write(int destination, int severity, const char *format, ...);

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

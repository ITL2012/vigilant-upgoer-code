#include "globals.h"
#include "instruments.h"
#include "guidance_flight_control.h"
#include "background_tasks.h"
#include "web_server.h"
#include "Launchsequence.h"
#include "debug_cli.h"
#include "buzzers.h"
#include "brownout_recovery.h"

#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <SPI.h>
#include <esp_task_wdt.h>
#include <TinyGPSPlus.h>

std::atomic<SystemMode> currentSystemMode(MODE_TRANSPORT);
std::atomic<FlightPhase> currentPhase(PAD);
std::atomic<bool> systemArmed(false);
std::atomic<bool> wifiActive(true);
std::atomic<unsigned long long> systemBaseEpochMs(0);
std::atomic<unsigned long> systemBaseMillis(0);

std::atomic<unsigned long> liftoff_time_ms(0);

float V_z = 0.0f;
float filter_alt = 0.0f;
float baseline_altitude = 0.0f;
float previous_altitude = 0.0f;
float qnh_pressure = 1013.25f;
float max_altitude = 0.0f;

float latestServoAngles[8] = {90.0f};
float latestPIDOutputs[8]  = {0.0f};
float latestActiveGains[3] = {0.0f};
float latestAirspeed    = 0.0f;
float latestKalmanP     = 0.0f;
float latestBaroAlpha   = 1.0f;
float latestBaroPressure = 0.0f;
float latestBaroTemp     = 0.0f;
float latestDt           = 0.0f;
float latestQx = 0.0f, latestQy = 0.0f, latestQz = 0.0f, latestQw = 1.0f;
float latestAx = 0.0f, latestAy = 0.0f, latestAz = 0.0f;

volatile bool servoOverrideActive = false;
volatile float servoOverrideAngles[8] = {90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f};

char serialLogBuffer[SERIAL_LOG_BUF_SIZE];
volatile uint16_t serialLogHead = 0;
volatile uint16_t serialLogTail = 0;

char logCache[LOG_CACHE_SIZE];
volatile size_t logCacheHead = 0;
volatile size_t logCacheTail = 0;
volatile bool logCacheOverflow = false;

SemaphoreHandle_t logCacheMutex = NULL;

unsigned long lastMicros = 0;
unsigned long lastLogTime = 0;
std::atomic<unsigned long> lastIMUReport_ms(0);
std::atomic<uint8_t> imuCalStatus(0);
std::atomic<uint32_t> logDropCount(0);

HardwareSerial gpsSerial(2);
TinyGPSPlus gps;
Adafruit_PWMServoDriver pwm(PCA9685_ADDR);

volatile TelemetryData sharedTelemetry;
SemaphoreHandle_t telemetryMutex = NULL;
QueueHandle_t sdLogQueue = NULL;
TaskHandle_t telemetryTaskHandle = NULL;
TaskHandle_t sdWriterTaskHandle = NULL;

// ============================================================================
// PYRO CHANNEL REGISTRY (config) & STATE
// ============================================================================
// Edit assignments below to rebind a physical pin to a role. Set pin=999 to
// effectively disable a channel. The order is pyro1/2/3 -> index 0/1/2.

PyroChannelConfig pyroChannels[3] = {
    // [0] pyro1 — primary chute
    { parachutePyroPin, parachutePulseDurationMs, ROLE_PRIMARY_CHUTE, true, 0 },
    // [1] pyro2 — backup chute
    { backupChutePyroPin, backupPulseDurationMs, ROLE_BACKUP_CHUTE, true, 0 },
    // [2] pyro3 — aux role. Default is MANUAL (operator assigns staging / main via code)
    { auxiliaryPyroPin, auxPulseDurationMs, ROLE_MANUAL, true, 0 },
};

PyroChannelState pyroState[3] = {
    { PYRO_IDLE, 0, false },
    { PYRO_IDLE, 0, false },
    { PYRO_IDLE, 0, false },
};

// ============================================================================
// BROWNOUT RECOVERY — RUNTIME STATE + DIAGNOSTICS
// ============================================================================
// enableBrownoutRecovery / enableDualWriteCache / enableBackupChute are
// static constexpr in globals.h (read-only). Only flightCacheValid + the
// diagnostic counters are defined here, since they mutate at boot.

FlightCache flightCacheRestored;
uint32_t bootCount = 0;
bool flightCacheValid = false;         // set true after successful cache load

// Barometer calibration (always-on restore from NVS at boot)
BaroCalibration baroCal;
const char *baroCalSource = "none";



// ---- Setup & Loop ----

void setup() {
    Serial.begin(115200);

    // Fast mid-flight-resume detector — RTC slow-memory only (microseconds,
    // no flash/SD/wire traffic). If a brownout / panic reset happened during
    // flight, skip the 1s power-settle delay so the critical sensors and the
    // restored flight state come up almost immediately.
    const bool expedited = flightCacheExpediteBoot();
    if (!expedited) delay(1000);

    initLogCache();

    // ---- BOOT DIAGNOSTICS & BROWNOUT RECOVERY ----
    // Track ESP reset reason +_INCREMENT boot counter (NOINIT survives reboot only)
    {
        // Determine reset reason. esp_reset_reason() survives across MOST reboots
        // (it's based on a hardware register). RTC_NOINIT's rtcBootCount is used as
        // a soft survival check — if it retained its value we likely had a soft reset.
        uint32_t prevRtcBoot = rtcBootCount;
        (void)prevRtcBoot;
        rtcBootCount++;
        rtcLastResetReason = (uint32_t)esp_reset_reason();
        bootCount = rtcBootCount;

        write(LOG_SERIAL, LOG_INFO, "\n====================================");
        write(LOG_SERIAL, LOG_INFO, "ISAAC L FLIGHT CONTROLLER - BOOTING");
        write(LOG_SERIAL, LOG_INFO, "====================================");
        write(LOG_BOTH, LOG_INFO,
              "[BOOT] bootCount=%u resetReason=%u (1=PowerOn 4=Brownout 14=Panic 15=SW 16=WDT)",
              (unsigned)bootCount, (unsigned)rtcLastResetReason);
    }
    if (debugMode) write(LOG_SERIAL, LOG_INFO, "[DEBUG] Type 'help' for debug commands\n");





    #pragma region Initialization





    esp_task_wdt_init(WDT_TIMEOUT_S, true);
    // loopTask was auto-registered by the Arduino core; feed it during long ops
    lastMicros = micros();

    // Create RTOS synchronization primitives
    telemetryMutex = xSemaphoreCreateMutex();
    webServerMutex = xSemaphoreCreateMutex();
    sdLogQueue = createPSRAMBackedQueue(LOG_QUEUE_LEN, sizeof(LogPacket));

    if (telemetryMutex == NULL || sdLogQueue == NULL || webServerMutex == NULL) {
        write(LOG_SERIAL, LOG_ERROR, "[CRITICAL ERROR] Failed to create RTOS structures!");
        while (1) {
            delay(100);
        }
    }

    // True if a cached in-flight snapshot was applied during this boot. Used in
    // the shared boot footer below for both the expedited and normal paths.
    bool resumedInFlight = false;

    if (expedited) {
        // ===================== EXPEDITED MID-FLIGHT RESUME =====================
        // Skipped vs. normal boot: 1s power settle, 100ms SPI settle, 100ms I2C
        // settle, 200ms GPS settle, 500ms task-bootstrap delay, SD mount, GPS
        // serial + WiFi bring-up. Critical path is only: SPI -> IMU/baro ->
        // baro-cal restore -> PWM/servos -> PIDs. The cached flight state is
        // applied by the shared BROWNOUT RECOVERY block just below, before the
        // first loop() iteration. Everything else finishes in DeferredInitTask
        // at low priority in the background.
        write(LOG_BOTH, LOG_WARN, "[BOOT] EXPEDITED — mid-flight resume detected, fast init");
        esp_task_wdt_reset();

        // ---- SPI + critical instruments (BNO085 IMU, BMP580 baro) ----
        write(LOG_BOTH, LOG_INFO, "[BOOT] Initializing sensors...");
        SPI.begin(VSPI_CLK, VSPI_MISO, VSPI_MOSI);
        initInstruments();
        checkInstruments();

        // ---- BAROMETER CALIBRATION RESTORE (always-on) ----
        // Must run before any telemetry/fusion so filter_alt / QNH are valid.
        baroCalibrationRestoreOnBoot();

        // ---- I2C + PWM/servo init (needed for active control) ----
        Wire.begin(I2C_SDA, I2C_SCL);
        Wire.setClock(I2C_SPEED);

        pwm.begin();
        pwm.setPWMFreq(50);
        // Mid-flight resume: seed the servos with the last-commanded angles
        // from the RTC cache (no 90° "death jerk"), if we have a valid
        // in-flight snapshot to restore from.
        float seedServo[8];
        if (peekCachedServoAngles(seedServo)) {
            UserSpace::initServos(true, seedServo);
        } else {
            UserSpace::initServos(true, nullptr);   // cache stale/missing — center neutrally
        }

        // ---- Flight-control PIDs ----
        write(LOG_BOTH, LOG_INFO, "[BOOT] Initializing stabilization PIDs...");
        initStabilizationPIDs(true);
        airspeedFilter.init();

        // ---- Tasks: telemetry now, the rest deferred ----
        if (xTaskCreatePinnedToCore(
            TelemetryTask, "GPS_Telemetry_Task", 4096, NULL, 2,
            &telemetryTaskHandle, 0) != pdPASS) {
            write(LOG_BOTH, LOG_ERROR, "[FAIL] Telemetry task create");
        }
        if (xTaskCreatePinnedToCore(
            DeferredInitTask, "Deferred_Init_Task", 4096, NULL, 1,
            NULL, 0) != pdPASS) {
            write(LOG_BOTH, LOG_ERROR, "[FAIL] Deferred init task create");
        }
        esp_task_wdt_reset();

    } else {
    #pragma region Initialization

    // ---- SPI Bus Initialization (VSPI for BNO085, BMP580) ----
    write(LOG_BOTH, LOG_INFO, "[BOOT] Initializing SPI bus...");
    SPI.begin(VSPI_CLK, VSPI_MISO, VSPI_MOSI);
    delay(100);
    write(LOG_BOTH, LOG_INFO, "[OK] SPI bus ready");
    esp_task_wdt_reset();

    // ---- Instrument Initialization (BNO085 IMU, BMP580 Baro) ----
    write(LOG_BOTH, LOG_INFO, "[BOOT] Initializing sensors...");
    esp_task_wdt_reset();
    initInstruments();
    esp_task_wdt_reset();
    checkInstruments();

    // ---- BAROMETER CALIBRATION RESTORE (always-on, from NVS) ----
    // Done right after instruments init so filter_alt / QNH have valid values
    // before any telemetry/fusion runs. If no stored calibration, baseline stays
    // 0 and QNH stays ISA default; operator can calibrate via web/CLI on the pad.
    baroCalibrationRestoreOnBoot();

    // ---- I2C Bus Initialization ----
    write(LOG_BOTH, LOG_INFO, "[BOOT] Initializing I2C bus...");
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_SPEED);
    delay(100);
    write(LOG_BOTH, LOG_INFO, "[OK] I2C bus ready");
    esp_task_wdt_reset();

    // ---- PWM/Servo Initialization ----
    write(LOG_BOTH, LOG_INFO, "[BOOT] Initializing PWM controller...");
    pwm.begin();
    pwm.setPWMFreq(50);
    UserSpace::initServos();
    write(LOG_BOTH, LOG_INFO, "[OK] PWM controller initialized");
    esp_task_wdt_reset();

    // ---- Flight Control PID Initialization ----
    write(LOG_BOTH, LOG_INFO, "[BOOT] Initializing stabilization PIDs...");
    initStabilizationPIDs();
    airspeedFilter.init();
    write(LOG_BOTH, LOG_INFO, "[OK] PIDs initialized");

    // ---- SD Card Initialization ----
    esp_task_wdt_reset();  // feed before potentially blocking SD.begin()
    SDInit();
    esp_task_wdt_reset();  // feed after
    if (sdReady) {
        write(LOG_BOTH, LOG_INFO, "[OK] SD card ready for logging");
        write(LOG_BOTH, LOG_INFO, "SD card ready for logging.");
        flushLogCacheToSD();
    } else {
        write(LOG_BOTH, LOG_WARN, "[WARN] SD card not available - logging disabled");
    }

    // ---- GPS Initialization ----
    write(LOG_BOTH, LOG_INFO, "[BOOT] Initializing GPS module...");
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX2, GPS_TX2);
    delay(200);
    if (!gpsSerial) {
        write(LOG_BOTH, LOG_WARN, "[WARN] GPS serial port not available");
    } else {
        write(LOG_BOTH, LOG_INFO, "[OK] GPS communication open at %d baud", GPS_BAUD);
    }
    esp_task_wdt_reset();


    write(LOG_BOTH, LOG_INFO, "[BOOT] Creating RTOS tasks...");
    
    // Create Telemetry/GPS Task
    if (xTaskCreatePinnedToCore(
        TelemetryTask,
        "GPS_Telemetry_Task",
        4096,
        NULL,
        2,
        &telemetryTaskHandle,
        0
    ) != pdPASS) {
        write(LOG_BOTH, LOG_ERROR, "[ERROR] Failed to create telemetry task!");
    } else {
        write(LOG_BOTH, LOG_INFO, "[OK] Telemetry task created");
    }

    // Create SD Writer Task (only if SD is ready)
    if (sdReady) {
        if (xTaskCreatePinnedToCore(
            SDWriterTask,
            "SD_Logging_Task",
            4096,
            NULL,
            1,
            &sdWriterTaskHandle,
            0
        ) != pdPASS) {
            write(LOG_BOTH, LOG_ERROR, "[ERROR] Failed to create SD task!");
        } else {
            write(LOG_BOTH, LOG_INFO, "[OK] SD logging task created");
        }
    } else {
        write(LOG_BOTH, LOG_INFO, "[SKIP] SD task skipped (SD not ready)");
        sdWriterTaskHandle = NULL;
    }

    // Create WiFi Server Task
    if (xTaskCreatePinnedToCore(
        WifiServerTask,
        "GCS_Wifi_Server",
        8192,
        NULL,
        1,
        &wifiServerTaskHandle,
        0
    ) != pdPASS) {
        write(LOG_BOTH, LOG_ERROR, "[ERROR] Failed to create WiFi task!");
    } else {
        write(LOG_BOTH, LOG_INFO, "[OK] WiFi server task created");
    }
    esp_task_wdt_reset();

    delay(500); // Give tasks time to start

    #pragma endregion
    } // else (normal boot) — expedited boot ended above

    // ---- BROWNOUT RECOVERY — apply cached in-flight state (if recent) ----
    // Done after all subsystems are up so telemetry / servos / SD logging
    // resume immediately alongside the restored phase. On an expedited boot
    // this restores the cached flight before the first loop() iteration. If
    // this returns true, we skip the "READY FOR OPERATION" boot-complete
    // posture and continue flying.
    resumedInFlight = flightCacheLoadAndApply();
    if (resumedInFlight) {
        write(LOG_BOTH, LOG_WARN,
              "[BROWNOUT] Flight resumed from cached snapshot. Continuing phase=%u.",
              (unsigned)currentPhase.load(std::memory_order_relaxed));
    }

    // Signal successful boot with chime
    startupChime();

    write(LOG_SERIAL, LOG_INFO, "\n====================================");
    if (resumedInFlight) {
        write(LOG_SERIAL, LOG_INFO, "BOOT COMPLETE - FLIGHT RESUMED");
    } else {
        write(LOG_SERIAL, LOG_INFO, "BOOT COMPLETE - READY FOR OPERATION");
    }
    write(LOG_SERIAL, LOG_INFO, "====================================");
    write(LOG_BOTH, LOG_INFO, "[SYSTEM] Core 1 standby. Mode: TRANSPORT.");
    if (debugMode) {
    write(LOG_SERIAL, LOG_INFO, "[DEBUG] Enter command (type 'help'):\n");
    Serial.print("> ");
    }
}
void loop() {
    // Process debug CLI commands
     FlightPhase phase = currentPhase.load(std::memory_order_relaxed);

     if (debugMode == true) debugCLI_loop(); // Make debugcli more restrictive, but also easier to do HIL testing
     
     esp_task_wdt_reset();

    unsigned long currentMicros = micros();
    float dt = (currentMicros - lastMicros) / 1000000.0f;
    lastMicros = currentMicros;
    latestDt = dt;

    float raw_altitude = 0.0f;
    float lin_ax = 0.0f, lin_ay = 0.0f, lin_az = 0.0f;
    float qx = 0.0f, qy = 0.0f, qz = 0.0f, qw = 1.0f;
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

    SystemMode mode = currentSystemMode.load(std::memory_order_relaxed);

    // Armed alarm only in ACTIVE_PAD (READY/flight phases)
    if (mode == MODE_ACTIVE_PAD) armedAlarm();

    // Recovery beacon runs independently of mode
    if (phase == RECOVERY) {
        recoveryBeaconUpdate();
    }

    // Advance any scheduled (non-blocking) chime sequence
    buzzerUpdate();

    // De-energize any fired pyro pin after its configured pulse duration
    pyroPulseMaintain();

    // Determine logging interval based on mode/phase
    int logIntervalMs = 1000; // default 1 Hz for TRANSPORT, PAD, RECOVERY
    if (mode == MODE_ACTIVE_PAD) {
        logIntervalMs = 10; // 100 Hz for ACTIVE_PAD (READY/flight)
    }

    // Sensor reading & fusion runs in PAD and ACTIVE_PAD modes
    if (mode == MODE_PAD || mode == MODE_ACTIVE_PAD) {
        raw_altitude = readBaroAltitude();

        if (bmpInitialized && bmp.performReading()) {
            latestBaroPressure = bmp.pressure / 100.0f;
            latestBaroTemp     = bmp.temperature;
        }

        if (readIMU(qx, qy, qz, qw, lin_ax, lin_ay, lin_az)) {
            latestQx = qx; latestQy = qy; latestQz = qz; latestQw = qw;
            latestAx = lin_ax; latestAy = lin_ay; latestAz = lin_az;
            roll  = atan2(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy)) * 180.0f / M_PI;
            pitch = asin(2.0f * (qw * qy - qz * qx)) * 180.0f / M_PI;
            yaw   = atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz)) * 180.0f / M_PI;
        }

        updateRocketFusion(lin_ax, lin_ay, lin_az, qx, qy, qz, qw, raw_altitude, previous_altitude, dt);

        FlightPhase wifiPhase = currentPhase.load(std::memory_order_relaxed);
        if (wifiPhase == BOOST && wifiActive.load(std::memory_order_relaxed)) {
            filter_alt = raw_altitude;
            shutdownWiFiNetwork();
        }

        // Re-enable WiFi in RECOVERY phase
        if (phase == RECOVERY && !wifiActive.load(std::memory_order_relaxed)) {
            // WiFi was shut down at BOOST, restart it for recovery
            wifiActive.store(true, std::memory_order_relaxed);
            // Note: WifiServerTask will re-create the server
            write(LOG_BOTH, LOG_INFO, "[RECOVERY] WiFi re-enabled for recovery");
        }

        if (phase == PAD || phase == READY) {
            userPreFlightLoop(dt);

            if (servoOverrideActive) {
                for (int i = 0; i < 8; i++) {
                    float angle = servoOverrideAngles[i];
                    UserSpace::writeServoAngle(i, angle);
                    latestServoAngles[i] = angle;
                }
            }
        } else if (phase == BOOST || phase == COAST) {
            userFlightStabilizationLoop(roll, pitch, yaw, V_z, dt);
        }
        // DESCENT and RECOVERY: no active stabilization, no servo override

        if (gps.location.isUpdated() && gps.speed.isValid()) {
            airspeedFilter.updateGPS(gps.speed.mps(), true);
        }

    } else {
        // MODE_TRANSPORT - sensors off, minimal loop
        userPreFlightLoop(dt);
        delay(10);
    }

    updateSharedTelemetry(roll, pitch, yaw, raw_altitude, filter_alt, V_z);

    // Logging at appropriate rate
    unsigned long now = millis();
    if (now - lastLogTime >= logIntervalMs) {
        lastLogTime = now;

        FlightPhase phase = currentPhase.load(std::memory_order_relaxed);
        LogPacket packet;
        packet.timestamp_ms = now;
        packet.epoch_ms = systemBaseEpochMs.load(std::memory_order_relaxed) + now;
        packet.roll = roll;
        packet.pitch = pitch;
        packet.yaw = yaw;
        packet.raw_alt = raw_altitude;
        packet.filtered_alt = filter_alt;
        packet.vel_z = V_z;
        packet.accel_x = lin_ax;
        packet.accel_y = lin_ay;
        packet.accel_z = lin_az;
        packet.current_phase = static_cast<uint8_t>(phase);

        packet.servo0 = latestServoAngles[0];
        packet.servo1 = latestServoAngles[1];
        packet.servo2 = latestServoAngles[2];
        packet.servo3 = latestServoAngles[3];
        packet.servo4 = latestServoAngles[4];
        packet.servo5 = latestServoAngles[5];
        packet.servo6 = latestServoAngles[6];
        packet.servo7 = latestServoAngles[7];

        packet.pid0 = latestPIDOutputs[0];
        packet.pid1 = latestPIDOutputs[1];
        packet.pid2 = latestPIDOutputs[2];
        packet.pid3 = latestPIDOutputs[3];
        packet.pid4 = latestPIDOutputs[4];
        packet.pid5 = latestPIDOutputs[5];
        packet.pid6 = latestPIDOutputs[6];
        packet.pid7 = latestPIDOutputs[7];

        packet.gain_kp = latestActiveGains[0];
        packet.gain_ki = latestActiveGains[1];
        packet.gain_kd = latestActiveGains[2];

        packet.airspeed   = latestAirspeed;
        packet.kalman_P   = latestKalmanP;
        packet.baro_alpha = latestBaroAlpha;

        packet.qx = qx;
        packet.qy = qy;
        packet.qz = qz;
        packet.qw = qw;

        packet.baro_pressure = latestBaroPressure;
        packet.baro_temp     = latestBaroTemp;
        packet.dt            = latestDt;

        pushLogPacket(packet);
    }
}

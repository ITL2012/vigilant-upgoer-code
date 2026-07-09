#include "globals.h"
#include "instruments.h"
#include "guidance_flight_control.h"
#include "web_server.h"
#include "Launchsequence.h"
#include "debug_cli.h"

#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <SPI.h>
#include <esp_task_wdt.h>
#include <esp_heap_caps.h>
#include <SD.h>
#include <TinyGPSPlus.h>
#include <stdarg.h>

std::atomic<SystemMode> currentSystemMode(MODE_TRANSPORT);
std::atomic<FlightPhase> currentPhase(PAD);
std::atomic<bool> systemArmed(false);
std::atomic<bool> wifiActive(true);
std::atomic<unsigned long long> systemBaseEpochMs(0);
std::atomic<unsigned long> systemBaseMillis(0);

static StaticQueue_t *sdLogQueueStatic = NULL;
static uint8_t *sdLogQueueStorage = NULL;

QueueHandle_t createPSRAMBackedQueue(size_t queueLength, size_t itemSize) {
    // 1. Allocate control structure locally in fast internal RAM
    StaticQueue_t* queueStatic = static_cast<StaticQueue_t*>(
        heap_caps_malloc(sizeof(StaticQueue_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
    );
    
    if (queueStatic == NULL) {
        controlAllocFailures++;
        Serial.printf("[WARN] PSRAM Queue: Internal control allocation failed (count=%u). Falling back to standard queue. FreeHeap=%u\n",
                      controlAllocFailures, esp_get_free_heap_size());
        return xQueueCreate(queueLength, itemSize);
    }

    // 2. Allocate the massive payload storage ring buffer in PSRAM
    uint8_t* queueStorage = static_cast<uint8_t*>(
        heap_caps_malloc(queueLength * itemSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
    );

    // Fallback optimization: If PSRAM allocation fails, clean up internal RAM first
    if (queueStorage == NULL) {
        psramAllocFailures++;
        heap_caps_free(queueStatic);
        Serial.printf("[WARN] PSRAM Queue: PSRAM allocation failed (count=%u). FreeHeap=%u FreePSRAM=%u\n",
                      psramAllocFailures, esp_get_free_heap_size(), heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        return xQueueCreate(queueLength, itemSize);
    }

    // 3. Create the static queue pairing the internal control block with the PSRAM storage block
    QueueHandle_t queue = xQueueCreateStatic(queueLength, itemSize, queueStorage, queueStatic);
    
    if (queue == NULL) {
        queueCreateFailures++;
        heap_caps_free(queueStorage);
        heap_caps_free(queueStatic);
        Serial.printf("[ERROR] PSRAM Queue: Static queue creation failed (count=%u). Falling back to standard queue. FreeHeap=%u FreePSRAM=%u\n",
                      queueCreateFailures, esp_get_free_heap_size(), heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        return xQueueCreate(queueLength, itemSize);
    }

    psramAllocatedBytes = queueLength * itemSize;
    Serial.printf("[OK] SD log queue created. Control: Internal RAM (%u bytes), Storage: %u bytes in PSRAM. FreePSRAM=%u\n",
                  (unsigned)sizeof(StaticQueue_t), (unsigned)psramAllocatedBytes, heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    return queue;
}


float V_z = 0.0f;
float filter_alt = 0.0f;
float baseline_altitude = 0.0f;
float previous_altitude = 0.0f;
float qnh_pressure = 1013.25f;

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

unsigned long lastMicros = 0;
unsigned long lastLogTime = 0;
std::atomic<unsigned long> lastIMUReport_ms(0);
volatile uint32_t logDropCount = 0;

SPIClass *hspi = nullptr;  // Initialized in setup() — NOT a global constructor
HardwareSerial gpsSerial(2);
TinyGPSPlus gps;
Adafruit_PWMServoDriver pwm(PCA9685_ADDR);

volatile TelemetryData sharedTelemetry;
SemaphoreHandle_t telemetryMutex = NULL;
QueueHandle_t sdLogQueue = NULL;
TaskHandle_t telemetryTaskHandle = NULL;
TaskHandle_t sdWriterTaskHandle = NULL;

bool sdReady = false;

// Diagnostic counters for verbose logging
uint32_t sdInitAttempts = 0;
uint32_t sdInitFailures = 0;
uint32_t sdWriteFailures = 0;
uint32_t psramAllocFailures = 0;
uint32_t controlAllocFailures = 0;
uint32_t queueCreateFailures = 0;
size_t psramAllocatedBytes = 0;

void logMessage(const char *message) {
    if (sdReady) {
        File file = SD.open(SYSTEM_LOG_FILE_PATH, FILE_APPEND);
        if (file) {
            file.println(message);
            file.close();
            return;
        }
    }

    Serial.println(message);
}

const char *getSeverityLabel(int severity) {
    switch (severity) {
        case LOG_WARN: return "WARN";
        case LOG_ERROR: return "ERROR";
        default: return "INFO";
    }
}

void write(int destination, int severity, const char *format, ...) {
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    char output[LOG_MESSAGE_BUFFER_SIZE];
    snprintf(output, sizeof(output), "[%s] %s", getSeverityLabel(severity), buffer);

    if (destination == LOG_SD || destination == LOG_BOTH) {
        if (sdReady) {
            File file = SD.open(SYSTEM_LOG_FILE_PATH, FILE_APPEND);
            if (file) {
                file.println(output);
                file.close();
            } else {
                Serial.println("[ERROR] Failed to open system log file on SD");
            }
        } else {
            Serial.println("[WARN] SD not ready: skipping SD log write");
        }
    }



    if (destination == LOG_SERIAL || destination == LOG_BOTH) {
        Serial.println(output);
        for (const char *p = output; *p; p++) {
            uint16_t next = (serialLogHead + 1) % SERIAL_LOG_BUF_SIZE;
            if (next != serialLogTail) {
                serialLogBuffer[serialLogHead] = *p;
                serialLogHead = next;
            }
        }
        uint16_t nextNl = (serialLogHead + 1) % SERIAL_LOG_BUF_SIZE;
        if (nextNl != serialLogTail) {
            serialLogBuffer[serialLogHead] = '\n';
            serialLogHead = nextNl;
        }
    }
}

// ---- Buzzer Functions ----
// LEDC channel for buzzer (use channel 0, timer 0)
static constexpr int BUZZER_LEDC_CHANNEL = 0;
static bool buzzerLedsInitialized = false;

void initBuzzerLEDC() {
    if (buzzerLedsInitialized) return;
    // Initialize LEDC: use 12-bit resolution (max for 5kHz), 5000Hz base freq
    // ESP32-S3: max freq at 12-bit = 80MHz/4096 ≈ 19.5kHz, so 5kHz is fine
    if (ledcSetup(BUZZER_LEDC_CHANNEL, 5000, 12) == 0) {
        // Fallback: lower frequency if setup fails
        if (ledcSetup(BUZZER_LEDC_CHANNEL, 4000, 12) == 0) {
            ledcSetup(BUZZER_LEDC_CHANNEL, 2000, 11);
        }
    }
    // Attach buzzer pin to LEDC channel 0
    ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CHANNEL);
    buzzerLedsInitialized = true;
    Serial.println("[OK] Buzzer LEDC initialized");
}

void playTone(unsigned int freq, unsigned long duration_ms) {
    if (enableBuzzer != true) return;
    initBuzzerLEDC();
    tone(BUZZER_PIN, freq, duration_ms);
}

void startupChime() {
    playTone(523, 100); delay(120);
    playTone(659, 100); delay(120);
    playTone(784, 150); delay(180);
    playTone(1047, 200); delay(250);
}

void errorBeep() {
    playTone(200, 1000);
}

void armChime() {
    playTone(880, 150); delay(180);
    playTone(880, 150); delay(180);
    playTone(1760, 400); delay(450);
}

void disarmChime() {
    playTone(1760, 150); delay(180);
    playTone(880, 300); delay(350);
}

void modeChangeChime() {
    playTone(587, 80); delay(100);
    playTone(880, 80); delay(100);
    playTone(1175, 150); delay(180);
}

// ---- SD Card ----

void SDInit() {
    sdReady = false;
    sdInitAttempts++;

    // Silent single attempt - retries handled at higher level if needed
    if (SD.begin(SD_CS, *hspi)) {
        sdReady = true;
        Serial.println("[OK] SD Card Initialized.");
    } else {
        sdInitFailures++;
        SD.end();
        // Silent failure - SD is optional, don't spam logs
        return;
    }

    // Ensure the log files exist with the desired header
    if (!SD.exists(LOG_FILE_PATH)) {
        File file = SD.open(LOG_FILE_PATH, FILE_WRITE);
        if (file) {
            file.println("timestamp_ms,roll,pitch,yaw,raw_alt,filtered_alt,vel_z,"
                         "acc_x,acc_y,acc_z,phase,"
                         "servo0,servo1,servo2,servo3,servo4,servo5,servo6,servo7,"
                         "pid0,pid1,pid2,pid3,pid4,pid5,pid6,pid7,"
                         "gain_kp,gain_ki,gain_kd,"
                         "airspeed,kalman_P,baro_alpha,"
                         "qx,qy,qz,qw,"
                         "baro_pressure,baro_temp,dt,dropped");
            file.close();
        }
    }

    if (!SD.exists(SYSTEM_LOG_FILE_PATH)) {
        File file = SD.open(SYSTEM_LOG_FILE_PATH, FILE_WRITE);
        if (file) {
            file.println("--- SYSTEM LOG INITIALIZED ---");
            file.close();
        }
    }
}

void pushLogPacket(const LogPacket &packet) {
    if (!sdReady || sdLogQueue == NULL) {
        return;
    }

    if (xQueueSend(sdLogQueue, &packet, 0) != pdTRUE) {
        logDropCount++;
        UBaseType_t spaces = uxQueueSpacesAvailable(sdLogQueue);
        UBaseType_t queued = uxQueueMessagesWaiting(sdLogQueue);
        Serial.printf("[WARN] Log drop #%u: queue full (spaces=%u, queued=%u). packet.ts=%u\n", logDropCount, (unsigned)spaces, (unsigned)queued, packet.timestamp_ms);
    }
}

// ---- RTOS Tasks ----

void TelemetryTask(void *pvParameters) {
    (void) pvParameters;
    esp_task_wdt_add(NULL);

    for (;;) {
        while (gpsSerial.available() > 0) {
            gps.encode(gpsSerial.read());
        }

        if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (gps.location.isUpdated()) {
                sharedTelemetry.latitude = gps.location.lat();
                sharedTelemetry.longitude = gps.location.lng();
                sharedTelemetry.gpsUpdated = true;
            } else {
                sharedTelemetry.gpsUpdated = false;
            }
            xSemaphoreGive(telemetryMutex);
        }

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void SDWriterTask(void *pvParameters) {
    (void) pvParameters;
    esp_task_wdt_add(NULL);

    LogPacket packet;

    for (;;) {
        if (xQueueReceive(sdLogQueue, &packet, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (!sdReady) {
                continue;
            }

            File file = SD.open(LOG_FILE_PATH, FILE_APPEND);
            if (file) {
                file.printf("%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%u,"
                            "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
                            "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                            "%.4f,%.4f,%.4f,"
                            "%.2f,%.4f,%.4f,"
                            "%.4f,%.4f,%.4f,%.4f,"
                            "%.2f,%.2f,%.6f,%u\n",
                    packet.timestamp_ms,
                    packet.roll, packet.pitch, packet.yaw,
                    packet.raw_alt, packet.filtered_alt, packet.vel_z,
                    packet.accel_x, packet.accel_y, packet.accel_z,
                    packet.current_phase,

                    packet.servo0, packet.servo1, packet.servo2, packet.servo3,
                    packet.servo4, packet.servo5, packet.servo6, packet.servo7,

                    packet.pid0, packet.pid1, packet.pid2, packet.pid3,
                    packet.pid4, packet.pid5, packet.pid6, packet.pid7,

                    packet.gain_kp, packet.gain_ki, packet.gain_kd,

                    packet.airspeed, packet.kalman_P, packet.baro_alpha,

                    packet.qx, packet.qy, packet.qz, packet.qw,

                    packet.baro_pressure, packet.baro_temp, packet.dt,
                    (uint32_t)logDropCount);
                file.close();
            } else {
                sdWriteFailures++;
                // Silent - SD is optional, don't spam logs
                SD.end();
                if (SD.begin(SD_CS, *hspi)) {
                    sdReady = true;
                }
            }
        }

        esp_task_wdt_reset();
    }
}

void updateSharedTelemetry(float roll, float pitch, float yaw,
                           float raw_altitude, float filtered_altitude,
                           float velocity_z) {
    if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(5)) != pdTRUE) return;

    sharedTelemetry.roll = roll;
    sharedTelemetry.pitch = pitch;
    sharedTelemetry.yaw = yaw;
    sharedTelemetry.raw_altitude = raw_altitude;
    sharedTelemetry.filtered_altitude = filtered_altitude;
    sharedTelemetry.velocity_z = velocity_z;
    sharedTelemetry.phase = static_cast<uint8_t>(currentPhase.load(std::memory_order_relaxed));
    sharedTelemetry.system_mode = static_cast<uint8_t>(currentSystemMode.load(std::memory_order_relaxed));
    sharedTelemetry.armed = systemArmed.load(std::memory_order_relaxed);
    sharedTelemetry.sensors_ok = bnoInitialized && bmpInitialized;

    xSemaphoreGive(telemetryMutex);
}

// ---- User Hooks ----

void userCustomSetup() {
    // User-defined initialization code can go here
}

// ---- Setup & Loop ----

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n====================================");
    Serial.println("ISAAC L FLIGHT CONTROLLER - BOOTING");
    Serial.println("====================================");
    Serial.println("[DEBUG] Type 'help' for debug commands\n");

    esp_task_wdt_init(WDT_TIMEOUT_S, true);
    esp_task_wdt_add(NULL);

    // Do not chirp the buzzer until after hardware is initialized
    lastMicros = micros();

    // Create RTOS synchronization primitives
    telemetryMutex = xSemaphoreCreateMutex();
    webServerMutex = xSemaphoreCreateMutex();
    sdLogQueue = createPSRAMBackedQueue(LOG_QUEUE_LEN, sizeof(LogPacket));

    if (telemetryMutex == NULL || sdLogQueue == NULL || webServerMutex == NULL) {
        Serial.println("[CRITICAL ERROR] Failed to create RTOS structures!");
        // Don't call errorBeep() here — LEDC/PWM isn't initialized yet
        // and calling tone() would trigger "LEDC is not initialized" errors.
        // Instead just halt with a clear message.
        while (1) {
            delay(100);
        }
    }

    // ---- I2C Bus Initialization ----
    Serial.println("[BOOT] Initializing I2C bus...");
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_SPEED);
    delay(100);
    Serial.println("[OK] I2C bus ready");

    // ---- PWM/Servo Initialization ----
    Serial.println("[BOOT] Initializing PWM controller...");
    // Adafruit_PWMServoDriver::begin() returns void; call it and proceed.
    pwm.begin();
    pwm.setPWMFreq(50);
    UserSpace::initServos();
    Serial.println("[OK] PWM controller initialized");

    // ---- Flight Control PID Initialization ----
    Serial.println("[BOOT] Initializing stabilization PIDs...");
    initStabilizationPIDs();
    airspeedFilter.init();
    Serial.println("[OK] PIDs initialized");

    // ---- SPI Bus Initialization ----
    Serial.println("[BOOT] Initializing SPI bus...");
    hspi = new SPIClass(HSPI);
    hspi->begin(HSPI_CLK, HSPI_MISO, HSPI_MOSI, SD_CS);
    delay(100);
    Serial.println("[OK] SPI bus ready");

    // ---- SD Card Initialization ----
    Serial.println("[BOOT] Initializing SD card...");
    SDInit();
    if (sdReady) {
        Serial.println("[OK] SD card ready for logging");
        write(LOG_SD, LOG_INFO, "SD card ready for logging.");
    } else {
        Serial.println("[WARN] SD card not available - logging disabled");
    }

    // ---- GPS Initialization ----
    Serial.println("[BOOT] Initializing GPS module...");
    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX2, GPS_TX2);
    delay(200);
    if (!gpsSerial) {
        Serial.println("[WARN] GPS serial port not available");
    } else {
        Serial.println("[OK] GPS communication open at 9600 baud");
    }

    userCustomSetup();

    Serial.println("[BOOT] Creating RTOS tasks...");
    
    // Create Telemetry/GPS Task (handles GPS data collection)
    if (xTaskCreatePinnedToCore(
        TelemetryTask,
        "GPS_Telemetry_Task",
        4096,
        NULL,
        2,
        &telemetryTaskHandle,
        0
    ) != pdPASS) {
        Serial.println("[ERROR] Failed to create telemetry task!");
    } else {
        Serial.println("[OK] Telemetry task created");
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
            Serial.println("[ERROR] Failed to create SD task!");
        } else {
            Serial.println("[OK] SD logging task created");
        }
    } else {
        Serial.println("[SKIP] SD task skipped (SD not ready)");
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
        Serial.println("[ERROR] Failed to create WiFi task!");
    } else {
        Serial.println("[OK] WiFi server task created");
    }

    delay(500); // Give tasks time to start

    // Signal successful boot with chime (move after bus and task init to avoid LEDC warnings)
    startupChime();

    Serial.println("\n====================================");
    Serial.println("BOOT COMPLETE - READY FOR OPERATION");
    Serial.println("====================================");
    Serial.println("[SYSTEM] Core 1 standby. Mode: TRANSPORT.");
    Serial.println("[DEBUG] Enter command (type 'help'):\n");
    Serial.print("> ");
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

    if (currentSystemMode.load(std::memory_order_relaxed) == MODE_ACTIVE_PAD) {
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

        if (gps.location.isUpdated() && gps.speed.isValid()) {
            airspeedFilter.updateGPS(gps.speed.mps(), true);
        }

    } else {
        userPreFlightLoop(dt);
        delay(10);
    }

    updateSharedTelemetry(roll, pitch, yaw, raw_altitude, filter_alt, V_z);

    if (currentSystemMode.load(std::memory_order_relaxed) == MODE_ACTIVE_PAD) {
        unsigned long now = millis();
        if (now - lastLogTime >= 10) {
            lastLogTime = now;

            FlightPhase phase = currentPhase.load(std::memory_order_relaxed);
            LogPacket packet;
            packet.timestamp_ms = now;
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
}

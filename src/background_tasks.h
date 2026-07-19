#ifndef GUIDANCE_BACKGROUND_TASKS_H
#define GUIDANCE_BACKGROUND_TASKS_H

#include "globals.h"
#include "instruments.h"
#include <esp_heap_caps.h>
#include <esp_task_wdt.h>
#include <SD_MMC.h>
#include <stdarg.h>

// Forward declarations for externs defined in main.cpp
extern HardwareSerial gpsSerial;
extern QueueHandle_t sdLogQueue;
extern char serialLogBuffer[];
extern volatile uint16_t serialLogHead;
extern volatile uint16_t serialLogTail;
extern char logCache[];
extern volatile size_t logCacheHead;
extern volatile size_t logCacheTail;
extern volatile bool logCacheOverflow;
extern SemaphoreHandle_t logCacheMutex;

// ---- Diagnostic Counters ----

uint32_t sdInitAttempts = 0;
uint32_t sdInitFailures = 0;
uint32_t sdWriteFailures = 0;
uint32_t psramAllocFailures = 0;
uint32_t controlAllocFailures = 0;
uint32_t queueCreateFailures = 0;
size_t psramAllocatedBytes = 0;
bool sdReady = false;

// ---- PSRAM-backed RTOS Queue ----

static StaticQueue_t *sdLogQueueStatic = NULL;
static uint8_t *sdLogQueueStorage = NULL;

static constexpr size_t INTERNAL_RAM_QUEUE_LEN = 200;

QueueHandle_t createPSRAMBackedQueue(size_t queueLength, size_t itemSize) {
    StaticQueue_t* queueStatic = static_cast<StaticQueue_t*>(
        heap_caps_malloc(sizeof(StaticQueue_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
    );

    if (queueStatic == NULL) {
        controlAllocFailures++;
        Serial.printf("[WARN] PSRAM Queue: Internal control allocation failed (count=%u). Falling back to small internal queue. FreeHeap=%u\n",
                      controlAllocFailures, esp_get_free_heap_size());
        return xQueueCreate(INTERNAL_RAM_QUEUE_LEN, itemSize);
    }

    uint8_t* queueStorage = static_cast<uint8_t*>(
        heap_caps_malloc(queueLength * itemSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
    );

    if (queueStorage == NULL) {
        psramAllocFailures++;
        heap_caps_free(queueStatic);
        Serial.printf("[WARN] PSRAM Queue: PSRAM allocation failed (count=%u). FreeHeap=%u FreePSRAM=%u. Falling back to small internal queue.\n",
                      psramAllocFailures, esp_get_free_heap_size(), heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        return xQueueCreate(INTERNAL_RAM_QUEUE_LEN, itemSize);
    }

    QueueHandle_t queue = xQueueCreateStatic(queueLength, itemSize, queueStorage, queueStatic);

    if (queue == NULL) {
        queueCreateFailures++;
        heap_caps_free(queueStorage);
        heap_caps_free(queueStatic);
        Serial.printf("[ERROR] PSRAM Queue: Static queue creation failed (count=%u). Falling back to small internal queue. FreeHeap=%u FreePSRAM=%u\n",
                      queueCreateFailures, esp_get_free_heap_size(), heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        return xQueueCreate(INTERNAL_RAM_QUEUE_LEN, itemSize);
    }

    psramAllocatedBytes = queueLength * itemSize;
    Serial.printf("[OK] SD log queue created. Control: Internal RAM (%u bytes), Storage: %u bytes in PSRAM. FreePSRAM=%u\n",
                  (unsigned)sizeof(StaticQueue_t), (unsigned)psramAllocatedBytes, heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    return queue;
}

// ---- Logging Helpers ----

void initLogCache() {
    logCacheHead = 0;
    logCacheTail = 0;
    logCacheOverflow = false;
    logCacheMutex = xSemaphoreCreateMutex();
    if (logCacheMutex == NULL) {
        Serial.println("[ERROR] Failed to create log cache mutex!");
    }
}

void appendToLogCache(const char *msg) {
    if (logCacheMutex && xSemaphoreTake(logCacheMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    
    size_t len = strlen(msg);
    if (len >= LOG_CACHE_SIZE) len = LOG_CACHE_SIZE - 1;
    
    size_t space;
    if (logCacheHead >= logCacheTail) {
        space = LOG_CACHE_SIZE - (logCacheHead - logCacheTail) - 1;
    } else {
        space = logCacheTail - logCacheHead - 1;
    }
    
    if (len >= space) {
        logCacheOverflow = true;
        if (logCacheMutex) xSemaphoreGive(logCacheMutex);
        return;
    }
    
    size_t firstChunk = min(len, LOG_CACHE_SIZE - logCacheHead);
    memcpy(&logCache[logCacheHead], msg, firstChunk);
    logCacheHead = (logCacheHead + firstChunk) % LOG_CACHE_SIZE;
    
    if (firstChunk < len) {
        memcpy(&logCache[logCacheHead], msg + firstChunk, len - firstChunk);
        logCacheHead = (logCacheHead + len - firstChunk) % LOG_CACHE_SIZE;
    }
    
    if (logCacheMutex) xSemaphoreGive(logCacheMutex);
}

void flushLogCacheToSD() {
    if (!sdReady || logCacheHead == logCacheTail) return;
    
    if (logCacheMutex && xSemaphoreTake(logCacheMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    
    File file = SD_MMC.open(SYSTEM_LOG_FILE_PATH, FILE_APPEND);
    if (file) {
        size_t total = (logCacheHead >= logCacheTail) ? 
            (logCacheHead - logCacheTail) : (LOG_CACHE_SIZE - logCacheTail + logCacheHead);
        
        if (logCacheHead >= logCacheTail) {
            file.write((uint8_t*)&logCache[logCacheTail], total);
        } else {
            size_t firstChunk = LOG_CACHE_SIZE - logCacheTail;
            file.write((uint8_t*)&logCache[logCacheTail], firstChunk);
            file.write((uint8_t*)&logCache[0], logCacheHead);
        }
        file.close();
        logCacheTail = logCacheHead;
        logCacheOverflow = false;
    }
    
    if (logCacheMutex) xSemaphoreGive(logCacheMutex);
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
    File file = SD_MMC.open(SYSTEM_LOG_FILE_PATH, FILE_APPEND);
            if (file) {
                file.println(output);
                file.close();
            }
        } else {
            appendToLogCache(output);
            appendToLogCache("\n");
        }
    }

    if (destination == LOG_SERIAL || destination == LOG_BOTH) {
        Serial.println(output);
        appendToLogCache(output);
        appendToLogCache("\n");
        
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

// ---- SD Card ----

void SDInit() {
    sdReady = false;
    sdInitAttempts++;

    // Do NOT use enable5v() — 5V pin conflicts with I2C_SCL and is only for servos

    // Configure SDMMC pins for 1-bit mode
    // CLK=42, CMD=15 (pull-up), D0=47 (pull-up) per PCB schematic
    SD_MMC.setPins(SDMMC_CLK, SDMMC_CMD, SDMMC_D0);

    for (int attempt = 0; attempt < 3; attempt++) {
            if (SD_MMC.begin("/sdcard", true, true, SDMMC_FREQ_DEFAULT, 5)) {
            sdReady = true;
            Serial.println("[OK] SD Card Initialized (SDMMC 1-bit)");
            break;
        }
        sdInitFailures++;
        Serial.printf("[WARN] SD init attempt %d/3 failed\n", attempt + 1);
        SD_MMC.end();
        delay(100);
    }

    if (!sdReady) {
        Serial.println("[WARN] SD Card not available after 3 attempts");
        return;
    }

if (!SD_MMC.exists(LOG_FILE_PATH)) {
        File file = SD_MMC.open(LOG_FILE_PATH, FILE_WRITE);
        if (file) {
            file.println("timestamp_ms,epoch_ms,roll,pitch,yaw,raw_alt,filtered_alt,vel_z,"
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

    if (!SD_MMC.exists(SYSTEM_LOG_FILE_PATH)) {
        File file = SD_MMC.open(SYSTEM_LOG_FILE_PATH, FILE_WRITE);
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
        uint32_t drops = logDropCount.fetch_add(1, std::memory_order_relaxed) + 1;
        UBaseType_t spaces = uxQueueSpacesAvailable(sdLogQueue);
        UBaseType_t queued = uxQueueMessagesWaiting(sdLogQueue);
        Serial.printf("[WARN] Log drop #%u: queue full (spaces=%u, queued=%u). packet.ts=%u\n", drops, (unsigned)spaces, (unsigned)queued, packet.timestamp_ms);
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
    File logFile;
    bool fileOpen = false;

    // Helper: safely (re)open log file
    auto openLogFile = [&]() -> bool {
        if (fileOpen) {
            logFile.close();
            fileOpen = false;
        }
        logFile = SD_MMC.open(LOG_FILE_PATH, FILE_APPEND);
        if (logFile) {
            fileOpen = true;
            return true;
        }
        return false;
    };

    // Initial open
    if (!openLogFile()) {
        write(LOG_SERIAL, LOG_ERROR, "[SDWriter] Failed to open log file at startup");
        sdReady = false;
    }

    unsigned long lastReinitMs = 0;
    const unsigned long REINIT_INTERVAL_MS = 15000;

    for (;;) {
        // Block up to 1s for a packet
        if (xQueueReceive(sdLogQueue, &packet, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (!sdReady) {
                continue;  // packet dropped, will be counted by pushLogPacket
            }

        // Ensure file is open; attempt recovery if needed
        if (!fileOpen) {
            unsigned long now = millis();
            if (now - lastReinitMs < REINIT_INTERVAL_MS) {
                continue;
            }
            lastReinitMs = now;
            sdWriteFailures++;
            SD_MMC.end();
            vTaskDelay(pdMS_TO_TICKS(100));
            SD_MMC.setPins(SDMMC_CLK, SDMMC_CMD, SDMMC_D0);
        if (SD_MMC.begin("/sdcard", true, true, SDMMC_FREQ_DEFAULT, 5)) {
                sdReady = true;
                if (!openLogFile()) {
                    sdReady = false;
                    continue;
                }
            } else {
                sdReady = false;
                continue;
            }
        }

        // Write the packet
            logFile.printf("%u,%llu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%u,"
                           "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
                           "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                           "%.4f,%.4f,%.4f,"
                           "%.2f,%.4f,%.4f,"
                           "%.4f,%.4f,%.4f,%.4f,"
                           "%.2f,%.2f,%.6f,%u\n",
                packet.timestamp_ms,
                packet.epoch_ms,
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
                logDropCount.load(std::memory_order_relaxed));

            // Flush strategy: only when queue drains to avoid I/O blocking during burst
            UBaseType_t remaining = uxQueueMessagesWaiting(sdLogQueue);
            if (remaining == 0) {
                logFile.flush();
            }
        }

        // Periodic flush + watchdog
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

#endif // GUIDANCE_BACKGROUND_TASKS_H

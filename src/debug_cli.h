// This is AI generated, but it will never be used when in flight because it has to have the var  debugMode = true

#ifndef GUIDANCE_DEBUG_CLI_H
#define GUIDANCE_DEBUG_CLI_H

#include "globals.h"
#include "instruments.h"
#include "Launchsequence.h"
#include "brownout_recovery.h"
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <SPI.h>
#include <SD_MMC.h>
#include <Adafruit_BMP5xx.h>
#include <esp_heap_caps.h>

// Declare external variables from main.cpp
extern TinyGPSPlus gps;
extern HardwareSerial gpsSerial;
extern uint32_t sdInitAttempts;
extern uint32_t sdInitFailures;
extern uint32_t sdWriteFailures;
extern uint32_t psramAllocFailures;
extern uint32_t controlAllocFailures;
extern uint32_t queueCreateFailures;
extern size_t psramAllocatedBytes;

// Global debug command buffer
static String debugCmdBuffer = "";

// ============================================================================
// DEBUG CLI — INTERACTIVE INSTRUMENT AND BUS TESTING
// ============================================================================

void debugCLI_printHelp() {
    Serial.println("\n=== ISAAC AVIONICS DEBUG CLI ===");
    Serial.println("Commands:");
    Serial.println("  help              - Show this menu");
    Serial.println("  status            - Show system & instrument status");
    Serial.println("  read gps          - Read GPS position & speed");
    Serial.println("  read imu          - Read IMU quaternion & accel");
    Serial.println("  read baro         - Read barometer pressure & temp");
    Serial.println("  test i2c          - Scan I2C bus for devices");
    Serial.println("  reset i2c         - Reset I2C bus (Wire.begin)");
    Serial.println("  reset spi         - Reinit SPI bus");
    Serial.println("  init instruments  - Reinitialize all sensors");
    Serial.println("  read sd           - List files on SD card");
    Serial.println("  sd info           - Show SD card info");
    Serial.println("  servo <ch> <ang>  - Set servo angle (0-7, 60-120\")");
    Serial.println("  mode <transport|pad|active_pad> - Set system mode");
    Serial.println("");
    Serial.println("  pyro list         - Show all pyro channels (role, pin, state)");
    Serial.println("  pyro fire <ch>    - Manual fire channel (0-2)");
    Serial.println("  pyro fireRole <r> - Manual fire whichever channel holds role r (0-5)");
    Serial.println("  pyro reset        - Clear all channel states to IDLE (ground test)");
    Serial.println("  pyro backup on|off- (BUILD-TIME only) — flags cannot be changed at runtime");
    Serial.println("");
    Serial.println("  brownout status   - Show cache/reset diagnostics");
    Serial.println("  brownout recovery on|off - (BUILD-TIME only)");
    Serial.println("  brownout dualwrite on|off- (BUILD-TIME only)");
    Serial.println("  brownout invalidate- Clear cached in-flight snapshot");
    Serial.println("");
    Serial.println("  baro status       - Show stored ground calibration (baseline/QNH/source)");
    Serial.println("  baro calibrate    - Re-sample & save ground calibration (NVS flash)");
    Serial.println("  baro invalidate   - Clear stored calibration, back to ISA defaults");
    Serial.println("  imu cal           - Show BNO085 calibration status (0-3)");
    Serial.println("");
}

void debugCLI_printStatus() {
    Serial.println("\n=== SYSTEM STATUS ===");
    SystemMode mode = currentSystemMode.load(std::memory_order_relaxed);
    const char* modeStr = (mode == MODE_TRANSPORT) ? "TRANSPORT" : 
                          (mode == MODE_PAD) ? "PAD" : "ACTIVE_PAD";
    Serial.printf("Mode: %s\n", modeStr);
    const char* phaseNames[] = {"TRANSPORT","PAD","READY","BOOST","COAST","DESCENT","RECOVERY"};
    FlightPhase phase = currentPhase.load(std::memory_order_relaxed);
    Serial.printf("Phase: %s (%d)\n", phaseNames[phase], (int)phase);
    Serial.printf("Armed: %s\n", systemArmed.load(std::memory_order_relaxed) ? "YES" : "NO");
    
    Serial.println("\n=== INSTRUMENTS ===");
    Serial.printf("IMU (BNO085): %s\n", bnoInitialized ? "OK" : "FAILED");
    Serial.printf("Barometer (BMP580): %s\n", bmpInitialized ? "OK" : "FAILED");
    Serial.printf("GPS (TinyGPS): %s\n", sharedTelemetry.gpsUpdated ? "OK" : "NO FIX");
    Serial.printf("SD Card: %s\n", sdReady ? "OK" : "NOT MOUNTED");
    Serial.println("");
    Serial.println("--- Diagnostics ---");
    Serial.printf("SD init attempts: %u, failures: %u\n", sdInitAttempts, sdInitFailures);
    Serial.printf("SD write failures: %u\n", sdWriteFailures);
    Serial.printf("PSRAM alloc fails: %u, control alloc fails: %u, queue create fails: %u\n",
                  psramAllocFailures, controlAllocFailures, queueCreateFailures);
    Serial.printf("PSRAM allocated bytes: %u, FreeHeap=%u FreePSRAM=%u\n",
                  (unsigned)psramAllocatedBytes, esp_get_free_heap_size(), heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    Serial.printf("\nLatitude:  %.6f\n", sharedTelemetry.latitude);
    Serial.printf("Longitude: %.6f\n", sharedTelemetry.longitude);
    Serial.printf("Altitude (filtered): %.2f m\n", filter_alt);
    Serial.printf("Velocity Z: %.2f m/s\n", V_z);
    Serial.printf("Roll: %.2f Pitch: %.2f Yaw: %.2f\n", 
                  current_roll, current_pitch, current_yaw);

    Serial.println("\n=== PYRO CHANNELS ===");
    Serial.printf("Backup chute: %s\n", enableBackupChute ? "ENABLED" : "DISABLED");
    for (int i = 0; i < 3; i++) {
        const char* stateStr = (pyroState[i].state == PYRO_IDLE)   ? "IDLE"  :
                               (pyroState[i].state == PYRO_FIRING) ? "FIRING":
                               (pyroState[i].state == PYRO_FIRED)  ? "FIRED" : "FAILED";
        Serial.printf("  ch%d pin=%d role=%s state=%s\n",
                      i, pyroChannels[i].pin, pyroRoleName(pyroChannels[i].role), stateStr);
    }
    Serial.println("\n=== BROWNOUT ===");
    Serial.printf("Boot count: %u  Last reset reason: %u\n",
                  (unsigned)bootCount, (unsigned)rtcLastResetReason);
    Serial.printf("Recovery enabled: %s  Dual-write: %s  Cache valid: %s\n",
                  enableBrownoutRecovery ? "Y" : "N",
                  enableDualWriteCache  ? "Y" : "N",
                  flightCacheValid      ? "Y" : "N");
    Serial.println("");
}

void debugCLI_readGPS() {
    if (!gps.location.isValid()) {
        Serial.println("[GPS] No valid fix yet");
        return;
    }
    Serial.printf("[GPS] Lat: %.6f, Lng: %.6f\n", gps.location.lat(), gps.location.lng());
    Serial.printf("[GPS] Speed: %.2f m/s, Altitude: %.2f m\n", gps.speed.mps(), gps.altitude.meters());
    Serial.printf("[GPS] Satellites: %d, HDOP: %.2f\n", gps.satellites.value(), gps.hdop.hdop());
}

void debugCLI_readIMU() {
    if (!bnoInitialized) {
        Serial.println("[IMU] Not initialized!");
        return;
    }
    
    float qx = latestQx, qy = latestQy, qz = latestQz, qw = latestQw;
    float ax = latestAx, ay = latestAy, az = latestAz;
    
    Serial.printf("[IMU] Quat: (%.4f, %.4f, %.4f, %.4f)\n", qx, qy, qz, qw);
    Serial.printf("[IMU] Accel: (%.3f, %.3f, %.3f) m/s²\n", ax, ay, az);
    Serial.printf("[IMU] Euler: Roll=%.2f° Pitch=%.2f° Yaw=%.2f°\n",
                  current_roll, current_pitch, current_yaw);
}

void debugCLI_readBaro() {
    if (!bmpInitialized) {
        Serial.println("[BARO] Not initialized!");
        return;
    }
    
    if (bmp.performReading()) {
        Serial.printf("[BARO] Pressure: %.2f hPa\n", bmp.pressure / 100.0f);
        Serial.printf("[BARO] Temperature: %.2f °C\n", bmp.temperature);
        Serial.printf("[BARO] Altitude: %.2f m (ICP-10125)\n", bmp.readAltitude(1013.25));
    } else {
        Serial.println("[BARO] Read failed!");
    }
}

void debugCLI_scanI2C() {
    Serial.println("[I2C] Scanning bus...");
    byte count = 0;
    for (byte i = 8; i < 120; i++) {
        Wire.beginTransmission(i);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  Found device at 0x%02X\n", i);
            count++;
        }
    }
    Serial.printf("[I2C] Found %d device(s)\n", count);
}

void debugCLI_resetI2C() {
    Serial.println("[I2C] Resetting bus...");
    Wire.end();
    delay(100);
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_SPEED);
    Serial.println("[I2C] Bus reset complete");
}

void debugCLI_resetSPI() {
    Serial.println("[SPI] VSPI bus reset...");
    SPI.end();
    delay(100);
    SPI.begin(VSPI_CLK, VSPI_MISO, VSPI_MOSI);
    Serial.println("[SPI] VSPI bus reset complete");
}

void debugCLI_initInstruments() {
    Serial.println("[INIT] Reinitializing instruments...");
    
    // Reset I2C
    Wire.end();
    delay(50);
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_SPEED);
    delay(100);
    
    // Reset VSPI
    SPI.end();
    delay(50);
    SPI.begin(VSPI_CLK, VSPI_MISO, VSPI_MOSI);
    delay(100);
    
    // Reinit instruments
    bnoInitialized = false;
    bmpInitialized = false;
    
    bool success = initInstruments();
    
    Serial.printf("[INIT] Result: %s\n", success ? "SUCCESS" : "PARTIAL/FAILED");
    Serial.printf("  IMU: %s\n", bnoInitialized ? "✓" : "✗");
    Serial.printf("  BARO: %s\n", bmpInitialized ? "✓" : "✗");
}

void debugCLI_sdInfo() {
    if (!sdReady) {
        Serial.println("[SD] Not mounted!");
        return;
    }
    
    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    Serial.printf("[SD] Card size: %llu MB\n", cardSize);
    
    // List files
    Serial.println("[SD] Files on card:");
    File root = SD_MMC.open("/");
    File file = root.openNextFile();
    while (file) {
        if (!file.isDirectory()) {
            Serial.printf("  %-24s %10lu bytes\n", file.name(), file.size());
        }
        file = root.openNextFile();
    }
    root.close();
}

void debugCLI_processCommand(String cmd) {
    cmd.trim();
    cmd.toLowerCase();
    
    if (cmd == "help") {
        debugCLI_printHelp();
    }
    else if (cmd == "status") {
        debugCLI_printStatus();
    }
    else if (cmd == "read gps") {
        debugCLI_readGPS();
    }
    else if (cmd == "read imu") {
        debugCLI_readIMU();
    }
    else if (cmd == "read baro") {
        debugCLI_readBaro();
    }
    else if (cmd == "test i2c") {
        debugCLI_scanI2C();
    }
    else if (cmd == "reset i2c") {
        debugCLI_resetI2C();
    }
    else if (cmd == "reset spi") {
        debugCLI_resetSPI();
    }
    else if (cmd == "init instruments") {
        debugCLI_initInstruments();
    }
    else if (cmd == "sd info" || cmd == "read sd") {
        debugCLI_sdInfo();
    }
    else if (cmd.startsWith("servo ")) {
        int space1 = cmd.indexOf(' ');
        int space2 = cmd.indexOf(' ', space1 + 1);
        if (space2 > 0) {
            int ch = cmd.substring(space1 + 1, space2).toInt();
            float ang = cmd.substring(space2 + 1).toFloat();
            if (ch >= 0 && ch <= 7 && ang >= 60 && ang <= 120) {
                UserSpace::writeServoAngle(ch, ang);
                latestServoAngles[ch] = ang;
                Serial.printf("[SERVO] Ch %d set to %.1f°\n", ch, ang);
            } else {
                Serial.println("[SERVO] Invalid params: servo <0-7> <60-120>");
            }
        }
    }
    else if (cmd == "pyro list" || cmd == "pyro") {
        Serial.println("\n=== PYRO CHANNELS ===");
        Serial.printf("Backup chute: %s\n", enableBackupChute ? "ENABLED" : "DISABLED");
        for (int i = 0; i < 3; i++) {
            const char* stateStr = (pyroState[i].state == PYRO_IDLE)   ? "IDLE"  :
                                   (pyroState[i].state == PYRO_FIRING) ? "FIRING":
                                   (pyroState[i].state == PYRO_FIRED)  ? "FIRED" : "FAILED";
            Serial.printf("  ch%d pin=%d role=%s pulse=%lu ms enabled=%s state=%s fired_ms=%lu\n",
                          i,
                          pyroChannels[i].pin,
                          pyroRoleName(pyroChannels[i].role),
                          pyroChannels[i].pulseMs,
                          pyroChannels[i].enabled ? "Y" : "N",
                          stateStr,
                          pyroState[i].firedAtMs);
        }
        Serial.println("Roles: 0=NONE 1=PRIMARY 2=BACKUP 3=AUX_STAGING 4=AUX_MAIN 5=MANUAL");
        Serial.println("");
    }
    else if (cmd.startsWith("pyro fire ")) {
        String arg = cmd.substring(10);
        arg.trim();
        int ch = arg.toInt();
        if (ch < 0 || ch > 2) {
            Serial.println("[PYRO] Invalid channel: pyro fire <0-2>");
        } else {
            bool ok = firePyroChannel(ch, /*safetyOverride=*/false);
            Serial.println(ok ? "[PYRO] Fire command accepted" : "[PYRO] Fire rejected (see log)");
        }
    }
    else if (cmd.startsWith("pyro fireRole ")) {
        String arg = cmd.substring(14);
        arg.trim();
        int r = arg.toInt();
        if (r < (int)ROLE_NONE || r > (int)ROLE_MANUAL) {
            Serial.println("[PYRO] Invalid role (0-5). See 'pyro list'.");
        } else {
            bool ok = fireByRole(pyroRoleFromInt(r), /*safetyOverride=*/false);
            Serial.println(ok ? "[PYRO] Fire-by-role accepted" : "[PYRO] Fire-by-role rejected (see log)");
        }
    }
    else if (cmd == "pyro reset") {
        if (systemArmed.load(std::memory_order_relaxed)) {
            Serial.println("[PYRO] Disarm before reset.");
        } else {
            resetPyroStates();
        }
    }
    else if (cmd.startsWith("pyro backup ")) {
        // enableBackupChute is static constexpr — cannot be mutated at runtime.
        Serial.println("[PYRO] enableBackupChute is BUILD-TIME only. Edit globals.h and reflash.");
    }
    else if (cmd == "brownout status") {
        Serial.println("\n=== BROWNOUT RECOVERY ===");
        Serial.printf("Recovery enabled:    %s  (BUILD-TIME)\n", enableBrownoutRecovery ? "YES" : "NO");
        Serial.printf("Dual-write log mirror: %s  (BUILD-TIME)\n", enableDualWriteCache ? "YES" : "NO");
        Serial.printf("Backup chute:        %s  (BUILD-TIME)\n", enableBackupChute ? "ENABLED" : "DISABLED");
        Serial.printf("Boot count:          %u\n", (unsigned)bootCount);
        Serial.printf("Last reset reason:   %u (4=Brownout 14=Panic 15=SW_RST 16=WDT)\n",
                      (unsigned)rtcLastResetReason);
        Serial.printf("Flight cache valid:  %s\n", flightCacheValid ? "YES (reloaded this boot)" : "NO");
        if (flightCacheValid) {
            Serial.printf("  Restored phase: %u  armed: %u  V_z: %.2f  filter_alt: %.2f\n",
                          (unsigned)flightCacheRestored.flightPhase,
                          (unsigned)flightCacheRestored.armed,
                          flightCacheRestored.V_z, flightCacheRestored.filter_alt);
        }
        Serial.println("");
    }
    else if (cmd.startsWith("brownout recovery ")) {
        Serial.println("[BROWNOUT] enableBrownoutRecovery is BUILD-TIME only. Edit globals.h and reflash.");
    }
    else if (cmd.startsWith("brownout dualwrite ")) {
        Serial.println("[BROWNOUT] enableDualWriteCache is BUILD-TIME only. Edit globals.h and reflash.");
    }
    else if (cmd == "brownout invalidate") {
        flightCacheInvalidate();
        Serial.println("[BROWNOUT] In-flight cache invalidated");
    }
    else if (cmd == "baro status") {
        extern std::atomic<unsigned long long> systemBaseEpochMs;
        unsigned long long ageMs = (baroCal.calibratedAtEpoch_ms > 0 && systemBaseEpochMs > 0)
            ? (systemBaseEpochMs - baroCal.calibratedAtEpoch_ms) : 0;
        Serial.println("\n=== BAROMETER CALIBRATION ===");
        Serial.printf("Source:            %s\n", baroCalSource);
        Serial.printf("Baseline altitude: %.2f m\n", baseline_altitude);
        Serial.printf("QNH reference:     %.2f hPa\n", qnh_pressure);
        Serial.printf("Cal age:           %s\n", baroCal.calibratedAtEpoch_ms ? "(see age_ms below)" : "never");
        Serial.printf("  age_ms=%llu\n", ageMs);
        Serial.println("");
    }
    else if (cmd == "baro calibrate") {
        FlightPhase phaseNow = currentPhase.load(std::memory_order_relaxed);
        if (phaseNow == BOOST || phaseNow == COAST || phaseNow == DESCENT || phaseNow == RECOVERY) {
            Serial.println("[BARO] BLOCKED: Cannot calibrate during flight!");
        } else if (!bmpInitialized) {
            Serial.println("[BARO] Barometer not initialized! Check wiring.");
        } else {
            Serial.println("[BARO] Sampling 50x over ~2s — keep rocket still...");
            calibrateGroundAltitude();
            Serial.printf("[BARO] Calibration saved to NVS flash — baseline=%.2fm qnh=%.2f hPa\n",
                          baseline_altitude, qnh_pressure);
        }
    }
    else if (cmd == "baro invalidate") {
        baroCalibrationInvalidate();
        memset(&baroCal, 0, sizeof(baroCal));
        baroCalSource = "none";
        qnh_pressure = 1013.25f;
        Serial.println("[BARO] Stored calibration cleared — using ISA defaults");
    }
    else if (cmd == "imu cal") {
        uint8_t cal = imuCalStatus.load(std::memory_order_relaxed);
        const char* names[] = {"UNRELIABLE (0/3)", "LOW (1/3)", "MEDIUM (2/3)", "FULLY CALIBRATED (3/3)"};
        Serial.println("\n=== IMU CALIBRATION (BNO085 dynamic cal) ===");
        Serial.printf("Status: %s\n", names[cal > 3 ? 0 : cal]);
        if (cal < 3) {
            Serial.println("Tip: keep rocket level & still for 30-60s; gyro/accel calibrate automatically.");
            Serial.println("     Yaw drift is normal — no magnetometer is used.");
        }
        Serial.println("");
    }
    else if (cmd.startsWith("mode ")) {
        // Prevent mode changes during flight
        FlightPhase phaseNow = currentPhase.load(std::memory_order_relaxed);
        if (phaseNow == BOOST || phaseNow == COAST || phaseNow == DESCENT) {
            Serial.println("[MODE] BLOCKED: Cannot change mode during flight");
            return;
        }

        String mode = cmd.substring(5);
        if (mode == "transport") {
            currentSystemMode.store(MODE_TRANSPORT, std::memory_order_relaxed);
            currentPhase.store(TRANSPORT, std::memory_order_relaxed);
            Serial.println("[MODE] Set to TRANSPORT");
        } else if (mode == "pad") {
            if (!bnoInitialized || !bmpInitialized) {
                if (!initInstruments()) {
                    Serial.println("[MODE] Instrument init failed - staying in TRANSPORT");
                    return;
                }
                calibrateGroundAltitude();
            }
            currentSystemMode.store(MODE_PAD, std::memory_order_relaxed);
            currentPhase.store(PAD, std::memory_order_relaxed);
            Serial.println("[MODE] Set to PAD");
        } else if (mode == "active_pad") {
            currentSystemMode.store(MODE_ACTIVE_PAD, std::memory_order_relaxed);
            currentPhase.store(READY, std::memory_order_relaxed);
            Serial.println("[MODE] Set to ACTIVE_PAD (READY)");
        }
    }
    else if (cmd.length() > 0) {
        Serial.println("[?] Unknown command. Type 'help' for commands.");
    }
}

void debugCLI_loop() {
    // Check for incoming serial data
    while (Serial.available()) {
        int c = Serial.read();
        
        if (c == '\n' || c == '\r') {
            if (debugCmdBuffer.length() > 0) {
                debugCLI_processCommand(debugCmdBuffer);
                debugCmdBuffer = "";
                Serial.print("> ");
            }
        } else if (c == '\b' || c == 0x7F) {
            if (debugCmdBuffer.length() > 0) {
                debugCmdBuffer.remove(debugCmdBuffer.length() - 1);
                Serial.write('\b');
                Serial.print(' ');
                Serial.write('\b');
            }
        } else if (c >= 32 && c < 127) {
            debugCmdBuffer += (char)c;
            Serial.write(c);
        }
    }
}

#endif // GUIDANCE_DEBUG_CLI_H

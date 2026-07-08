// This is AI generated, but it will never be used when in flight because it has to have the var  debugMode = true

#ifndef GUIDANCE_DEBUG_CLI_H
#define GUIDANCE_DEBUG_CLI_H

#include "globals.h"
#include "instruments.h"
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <SPI.h>
#include <Adafruit_BMP5xx.h>
#include <esp_heap_caps.h>

// Declare external variables from main.cpp
extern TinyGPSPlus gps;
extern SPIClass *hspi;
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
    Serial.println("  servo <ch> <ang>  - Set servo angle (0-7, 60-120°)");
    Serial.println("  mode <transport|active_pad> - Set system mode");
    Serial.println("");
}

void debugCLI_printStatus() {
    Serial.println("\n=== SYSTEM STATUS ===");
    Serial.printf("Mode: %s\n", (currentSystemMode.load(std::memory_order_relaxed) == MODE_TRANSPORT) ? "TRANSPORT" : "ACTIVE_PAD");
    Serial.printf("Phase: %d (0=TRANSPORT, 1=PAD, 2=READY, 3=BOOST, 4=COAST, 5=DESCENT)\n", 
                  (int)currentPhase.load(std::memory_order_relaxed));
    Serial.printf("Armed: %s\n", systemArmed.load(std::memory_order_relaxed) ? "YES" : "NO");
    
    Serial.println("\n=== INSTRUMENTS ===");
    Serial.printf("IMU (BNO085): %s\n", bnoInitialized ? "✓ OK" : "✗ FAILED");
    Serial.printf("Barometer (BMP580): %s\n", bmpInitialized ? "✓ OK" : "✗ FAILED");
    Serial.printf("GPS (TinyGPS): %s\n", sharedTelemetry.gpsUpdated ? "✓ OK" : "✗ NO FIX");
    Serial.printf("SD Card: %s\n", sdReady ? "✓ OK" : "✗ NOT MOUNTED");
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
    Serial.printf("Roll: %.2f° Pitch: %.2f° Yaw: %.2f°\n", 
                  current_roll, current_pitch, current_yaw);
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
    Serial.println("[SPI] Resetting bus...");
    hspi->end();
    delay(100);
    hspi->begin(HSPI_CLK, HSPI_MISO, HSPI_MOSI, SD_CS);
    Serial.println("[SPI] Bus reset complete");
}

void debugCLI_initInstruments() {
    Serial.println("[INIT] Reinitializing instruments...");
    
    // Reset I2C
    Wire.end();
    delay(50);
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_SPEED);
    delay(100);
    
    // Reset SPI
    hspi->end();
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
    
    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("[SD] Card size: %llu MB\n", cardSize);
    
    // List files
    Serial.println("[SD] Files on card:");
    File root = SD.open("/");
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
    else if (cmd.startsWith("mode ")) {
        String mode = cmd.substring(5);
        if (mode == "transport") {
            currentSystemMode.store(MODE_TRANSPORT, std::memory_order_relaxed);
            Serial.println("[MODE] Set to TRANSPORT");
        } else if (mode == "active_pad") {
            if (initInstruments()) {
                currentSystemMode.store(MODE_ACTIVE_PAD, std::memory_order_relaxed);
                Serial.println("[MODE] Set to ACTIVE_PAD");
            } else {
                Serial.println("[MODE] Instrument init failed - staying in TRANSPORT");
            }
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

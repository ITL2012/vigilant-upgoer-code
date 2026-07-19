#include "test_tools.h"
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_BMP5xx.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_PWMServoDriver.h>
#include <esp_heap_caps.h>

static String cmdBuf = "";

// ============== HARDWARE PROBE FUNCTIONS ==============

void probeVSPI() {
    Serial.println("\n=== VSPI BUS PROBE ===");
    Serial.printf("Pins: MOSI=%d, MISO=%d, CLK=%d\n", VSPI_MOSI, VSPI_MISO, VSPI_CLK);
    Serial.printf("BNO085: CS=%d, INT=%d, RST=%d\n", BNO_CS, BNO_INT, BNO_RST);
    Serial.printf("BMP580: CS=%d\n", BMP_CS);

    // Test VSPI bus by initializing and trying a loopback
    SPI.begin(VSPI_CLK, VSPI_MISO, VSPI_MOSI);

    // --- BMP580 ---
    Serial.println("\n-- BMP580 --");
    Adafruit_BMP5xx bmp;
    if (!bmp.begin(BMP_CS, &SPI)) {
        Serial.println("[FAIL] BMP580 not found on SPI (CS=10)");
    } else {
        Serial.println("[OK] BMP580 detected!");
        bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_8X);
        bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_4X);
        if (bmp.performReading()) {
            Serial.printf("  Pressure: %.2f hPa, Temp: %.2f C\n",
                bmp.pressure / 100.0f, bmp.temperature);
        }
    }

    // --- BNO085 (with diagnostics) ---
    Serial.println("\n-- BNO085 --");
    Adafruit_BNO08x bno08x(BNO_RST);

    // Manual hardware reset
    pinMode(BNO_RST, OUTPUT);
    digitalWrite(BNO_RST, LOW);
    delay(20);
    digitalWrite(BNO_RST, HIGH);
    delay(50);

    // Check CS pin state
    pinMode(BNO_CS, OUTPUT);
    digitalWrite(BNO_CS, HIGH);
    delay(1);

    // Try begin_SPI with explicit frequency
    if (!bno08x.begin_SPI(BNO_CS, BNO_INT, &SPI)) {
        Serial.println("[FAIL] BNO085 begin_SPI failed");
        // Try with SPI_MODE0
        Serial.println("  Retrying with SPI_MODE0...");
        SPI.end();
        delay(10);
        SPI.begin(VSPI_CLK, VSPI_MISO, VSPI_MOSI);
        if (!bno08x.begin_SPI(BNO_CS, BNO_INT, &SPI)) {
            Serial.println("[FAIL] BNO085 also failed with SPI_MODE0");
        } else {
            Serial.println("[OK] BNO085 detected with SPI_MODE0!");
        }
    } else {
        Serial.println("[OK] BNO085 detected!");
    }
    SPI.end();

    Serial.println("=== VSPI PROBE DONE ===\n");
}

void probeHSPI_SD() {
    Serial.println("\n=== HSPI (SD CARD) PROBE ===");
    Serial.printf("Pins: MOSI=%d, MISO=%d, CLK=%d, CS=%d\n",
        HSPI_MOSI, HSPI_MISO, HSPI_CLK, SD_CS);

    SPIClass hspi(HSPI);
    hspi.begin(HSPI_CLK, HSPI_MISO, HSPI_MOSI, SD_CS);

    // Enable internal pull-ups on CS line
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH);
    delay(10);

    // Try different SPI speeds
    for (int freq = 1000000; freq <= 4000000; freq += 1000000) {
        hspi.setFrequency(freq);
        Serial.printf("  Trying SD.begin() at %d Hz... ", freq);
        if (SD.begin(SD_CS, hspi)) {
            Serial.println("[OK] SD mounted!");
            uint64_t cardSize = SD.cardSize() / (1024 * 1024);
            Serial.printf("  Card size: %llu MB\n", cardSize);
            uint64_t used = SD.usedBytes() / 1024;
            uint64_t total = SD.totalBytes() / 1024;
            Serial.printf("  Used: %llu KB / Total: %llu KB\n", used, total);
            SD.end();
            Serial.println("=== HSPI PROBE DONE ===\n");
            return;
        } else {
            Serial.println("FAIL");
        }
    }

    // Check MISO line state
    pinMode(HSPI_MISO, INPUT_PULLUP);
    int misoVal = digitalRead(HSPI_MISO);
    Serial.printf("  MISO pin state (with pullup): %d\n", misoVal);

    // Try without CS signal using begin()
    Serial.println("  Trying SD.begin() with default CS assignment...");
    hspi.end();
    hspi.begin(HSPI_CLK, HSPI_MISO, HSPI_MOSI);
    if (SD.begin(SD_CS)) {
        Serial.println("[OK] SD mounted with default SPI!");
    } else {
        Serial.println("[FAIL] SD also failed with default SPI");
    }

    Serial.println("=== HSPI PROBE DONE ===\n");
}

void probeI2C() {
    Serial.println("\n=== I2C BUS PROBE ===");
    Serial.printf("Pins: SDA=%d, SCL=%d\n", I2C_SDA, I2C_SCL);

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000); // slow for probing

    // Check if 5V pin conflicts with SCL
    Serial.println("  Note: Enable5VPin=3 conflicts with I2C_SCL=3 in globals.h");
    Serial.println("  The 5V enable pin is NOT configured independently");

    // Scan I2C
    byte count = 0;
    for (byte addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  Found device at 0x%02X", addr);
            if (addr == 0x40) Serial.print(" (PCA9685 PWM)");
            if (addr == 0x28) Serial.print(" (BNO055)");
            if (addr == 0x29) Serial.print(" (BNO055 alt addr)");
            Serial.println();
            count++;
        }
    }
    if (count == 0) Serial.println("  No I2C devices found");
    Serial.printf("  Total I2C devices: %d\n", count);

    // Try PCA9685 directly
    Adafruit_PWMServoDriver pwm(0x40);
    pwm.begin();
    pwm.setPWMFreq(50);
    Serial.println("  PCA9685 PWM controller initialized");
    Serial.println("  (If no error above, I2C + PCA9685 are working)");

    Serial.println("=== I2C PROBE DONE ===\n");
}

void probe5V() {
    Serial.println("\n=== 5V POWER RAIL PROBE ===");
    Serial.println("  Enable5VPin in globals.h = 3 (same as I2C_SCL)");
    Serial.println("  This is a PIN CONFLICT - cannot independently control 5V");
    Serial.println("  If the board has a separate 5V enable, the pin is wrong.");
    Serial.println("  Checking if we can set pin 3 without breaking I2C...");

    pinMode(ENABLE_5V_PIN, OUTPUT);
    digitalWrite(ENABLE_5V_PIN, HIGH);
    delay(100);
    Serial.println("  Pin 3 set HIGH (may disrupt I2C SCL if shared)");

    // Check if I2C still works after this
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.beginTransmission(0x40);
    if (Wire.endTransmission() == 0) {
        Serial.println("  PCA9685 still reachable on I2C (no conflict)");
    } else {
        Serial.println("  PCA9685 NOT reachable - pin conflict confirmed!");
    }

    Serial.println("=== 5V PROBE DONE ===\n");
}

void doProbeAll() {
    Serial.println("\n\n===========================================");
    Serial.println("   COMPREHENSIVE HARDWARE PROBE");
    Serial.println("===========================================");

    probe5V();
    probeI2C();
    probeVSPI();
    probeHSPI_SD();

    Serial.println("\n===========================================");
    Serial.println("   PROBE COMPLETE");
    Serial.println("===========================================\n");
}

// ============== LEGACY COMMANDS ==============

void printHelp() {
    Serial.println("Test Tools CLI");
    Serial.println("Commands:");
    Serial.println("  help       - show this menu");
    Serial.println("  probe      - run all hardware probes");
    Serial.println("  sd test    - attempt SD init and create a test file");
    Serial.println("  sd info    - list files on SD");
    Serial.println("  psram test - try allocating 1MB in PSRAM");
    Serial.println("  i2c scan   - scan I2C bus");
    Serial.println("  mem info   - print heap/psram sizes");
}

void doMemInfo() {
    Serial.printf("FreeHeap=%u FreePSRAM=%u\n", esp_get_free_heap_size(),
        heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

void doI2cScan() {
    Serial.println("Scanning I2C...");
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    byte count = 0;
    for (byte i = 8; i < 120; i++) {
        Wire.beginTransmission(i);
        if (Wire.endTransmission() == 0) {
            Serial.printf(" Found 0x%02X\n", i);
            count++;
        }
    }
    Serial.printf("I2C devices found: %u\n", count);
}

void doPsramTest() {
    size_t testSize = 1024 * 1024;
    Serial.println("Attempting PSRAM allocation of 1MB...");
    void *p = heap_caps_malloc(testSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (p == NULL) {
        Serial.println("PSRAM allocation FAILED");
        Serial.printf("FreeHeap=%u FreePSRAM=%u\n",
            esp_get_free_heap_size(),
            heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    } else {
        Serial.println("PSRAM allocation succeeded");
        memset(p, 0xA5, testSize);
        heap_caps_free(p);
        Serial.printf("PSRAM test wrote and freed %u bytes\n", (unsigned)testSize);
    }
}

void doSdTest() {
    SPIClass hspi(HSPI);
    Serial.println("Starting HSPI for SD...");
    hspi.begin(HSPI_CLK, HSPI_MISO, HSPI_MOSI, SD_CS);
    delay(10);
    if (!SD.begin(SD_CS, hspi)) {
        Serial.println("SD.begin() FAILED");
        return;
    }
    Serial.println("SD mounted. Creating test file 'test_tools.txt'");
    File f = SD.open("/test_tools.txt", FILE_WRITE);
    if (!f) {
        Serial.println("Failed to open test file for writing");
    } else {
        f.println("test_tools: ok");
        f.close();
        Serial.println("Wrote test file");
    }
}

void handleCommand(String cmd) {
    cmd.trim();
    cmd.toLowerCase();
    if (cmd == "help") printHelp();
    else if (cmd == "probe") doProbeAll();
    else if (cmd == "sd test") doSdTest();
    else if (cmd == "sd info") {
        SPIClass hspi(HSPI);
        hspi.begin(HSPI_CLK, HSPI_MISO, HSPI_MOSI, SD_CS);
        if (!SD.begin(SD_CS, hspi)) { Serial.println("SD not mounted"); return; }
        File root = SD.open("/");
        File file = root.openNextFile();
        while (file) {
            if (!file.isDirectory()) {
                Serial.printf("%s %lu\n", file.name(), file.size());
            }
            file = root.openNextFile();
        }
    }
    else if (cmd == "psram test") doPsramTest();
    else if (cmd == "i2c scan") doI2cScan();
    else if (cmd == "mem info") doMemInfo();
    else Serial.println("Unknown command (type help)");
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== ESP32 Hardware Probe ===");
    Serial.printf("Free heap: %u bytes\n", esp_get_free_heap_size());
    Serial.printf("PSRAM free: %u bytes\n",
        heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    Serial.println("\nType 'probe' to run all hardware diagnostics.");
    Serial.print("> ");
}

void loop() {
    while (Serial.available()) {
        int c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmdBuf.length() > 0) {
                handleCommand(cmdBuf);
                cmdBuf = "";
                Serial.print("> ");
            }
        } else if (c >= 32 && c < 127) {
            cmdBuf += (char)c;
            Serial.write(c);
        }
    }
}

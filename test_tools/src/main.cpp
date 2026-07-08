#include "test_tools.h"
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_BMP5xx.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_PWMServoDriver.h>
#include <esp_heap_caps.h>

static String cmdBuf = "";

void printHelp() {
    Serial.println("Test Tools CLI");
    Serial.println("Commands:");
    Serial.println("  help       - show this menu");
    Serial.println("  sd test    - attempt SD init and create a test file");
    Serial.println("  sd info    - list files on SD");
    Serial.println("  psram test - try allocating 1MB in PSRAM");
    Serial.println("  i2c scan   - scan I2C bus");
    Serial.println("  mem info   - print heap/psram sizes");
}

void doMemInfo() {
    Serial.printf("FreeHeap=%u FreePSRAM=%u\n", esp_get_free_heap_size(), heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
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
    size_t testSize = 1024 * 1024; // 1MB
    Serial.println("Attempting PSRAM allocation of 1MB...");
    void *p = heap_caps_malloc(testSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (p == NULL) {
        Serial.println("PSRAM allocation FAILED");
        Serial.printf("FreeHeap=%u FreePSRAM=%u\n", esp_get_free_heap_size(), heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    } else {
        Serial.println("PSRAM allocation succeeded");
        memset(p, 0xA5, testSize);
        heap_caps_free(p);
        Serial.printf("PSRAM test wrote and freed %u bytes\n", (unsigned)testSize);
    }
}

void doSdTest() {
    SPIClass hspi(HSPI);
    Serial.println("Starting SPI for SD...");
    hspi.begin();
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
    else if (cmd == "sd test") doSdTest();
    else if (cmd == "sd info") {
        if (!SD.begin(SD_CS)) { Serial.println("SD not mounted"); return; }
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
    Serial.println("\n=== ESP32 Test Tools ===\n");
    printHelp();
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

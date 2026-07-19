#pragma once

#include <Arduino.h>

// Pins matching the main flight controller project:
// VSPI (sensors): BNO085 + BMP580
#define VSPI_MOSI 11
#define VSPI_MISO 13
#define VSPI_CLK  12
#define BMP_CS   10
#define BNO_CS    9
#define BNO_INT   8
#define BNO_RST  14

// HSPI (SD card)
#define SD_CS    21
#define HSPI_MOSI 35
#define HSPI_MISO 37
#define HSPI_CLK  36

// I2C (PCA9685 PWM controller)
#define I2C_SDA   1
#define I2C_SCL   3

// 5V enable
#define ENABLE_5V_PIN 3

void printHelp();
void handleCommand(String cmd);
void doSdTest();
void doPsramTest();
void doI2cScan();
void doMemInfo();
void doProbeAll();

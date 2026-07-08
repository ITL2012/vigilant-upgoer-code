#pragma once

#include <Arduino.h>

// Pins (override in main if needed)
#define SD_CS 5
#define I2C_SDA 21
#define I2C_SCL 22

void printHelp();
void handleCommand(String cmd);
void doSdTest();
void doPsramTest();
void doI2cScan();
void doMemInfo();

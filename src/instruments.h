#ifndef GUIDANCE_INSTRUMENTS_H
#define GUIDANCE_INSTRUMENTS_H

#include "globals.h"
#include "buzzers.h"
#include <Adafruit_BNO08x.h>
#include <Adafruit_BMP5xx.h>
#include <SPI.h>

Adafruit_BNO08x bno08x(BNO_RST);
sh2_SensorValue_t sh2_SensorValue;
bool bnoInitialized = false;

Adafruit_BMP5xx bmp;
bool bmpInitialized = false;

void enableIMUReports() {
    bno08x.enableReport(SH2_ROTATION_VECTOR, 10000);
    bno08x.enableReport(SH2_LINEAR_ACCELERATION, 10000);
    bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 10000);
}

bool initInstruments() {
    bool success = true;

    SPI.begin(VSPI_CLK, VSPI_MISO, VSPI_MOSI);

    if (!bmp.begin((int8_t)BMP_CS, &SPI)) {
        Serial.println("[ERROR] BMP5xx barometer not found on SPI!");
        success = false;
    } else {
        Serial.println("[OK] BMP5xx barometer connected.");
        bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_8X);
        bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_4X);
        bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
        bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);
        bmpInitialized = true;
    }

    if (!bno08x.begin_SPI(BNO_CS, BNO_INT, &SPI)) {
        Serial.println("[ERROR] BNO085 IMU not found on SPI!");
        success = false;
    } else {
        Serial.println("[OK] BNO085 IMU connected.");
        enableIMUReports();
        bnoInitialized = true;
        lastIMUReport_ms.store(millis(), std::memory_order_relaxed);
    }

    if (!success) {
        playTone(200, 500);
    }

    return success;
}

bool readIMU(float &qx, float &qy, float &qz, float &qw,
             float &lin_ax, float &lin_ay, float &lin_az) {
    if (!bnoInitialized) return false;

    if (bno08x.wasReset()) {
        Serial.println("[WARN] IMU Reset Event Occurred!");
        enableIMUReports();
    }

    bool gotData = false;
    if (bno08x.getSensorEvent(&sh2_SensorValue)) {
        switch (sh2_SensorValue.sensorId) {
            case SH2_ROTATION_VECTOR:
                qx = sh2_SensorValue.un.rotationVector.i;
                qy = sh2_SensorValue.un.rotationVector.j;
                qz = sh2_SensorValue.un.rotationVector.k;
                qw = sh2_SensorValue.un.rotationVector.real;
                gotData = true;
                break;
            case SH2_LINEAR_ACCELERATION:
                lin_ax = sh2_SensorValue.un.linearAcceleration.x;
                lin_ay = sh2_SensorValue.un.linearAcceleration.y;
                lin_az = sh2_SensorValue.un.linearAcceleration.z;
                gotData = true;
                break;
        }
    }

    if (gotData) {
        lastIMUReport_ms.store(millis(), std::memory_order_relaxed);
    }

    unsigned long lastReport = lastIMUReport_ms.load(std::memory_order_relaxed);
    if ((millis() - lastReport) > IMU_TIMEOUT_MS) {
        Serial.println("[WARN] IMU timeout — no data received!");
        return false;
    }

    return gotData;
}

float readBaroAltitude() {
    if (!bmpInitialized || !bmp.performReading()) return 0.0f;
    return bmp.readAltitude(qnh_pressure) - baseline_altitude;
}

void calibrateGroundAltitude() {
    if (!bmpInitialized) return;

    Serial.print("Calibrating launchpad ground level...");
    float sum = 0.0f;
    int samples = 50;

    for (int i = 0; i < samples; i++) {
        if (bmp.performReading()) {
            sum += bmp.readAltitude(1013.25f);
        }
        delay(40);
    }
    baseline_altitude = sum / samples;
    previous_altitude = baseline_altitude;

    float measuredPressure = bmp.pressure / 100.0f;
    qnh_pressure = measuredPressure;
    Serial.printf(" Done! Baseline: %.2fm, QNH: %.2f hPa\n", baseline_altitude, qnh_pressure);
}

#endif // GUIDANCE_INSTRUMENTS_H

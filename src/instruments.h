#ifndef GUIDANCE_INSTRUMENTS_H
#define GUIDANCE_INSTRUMENTS_H

#include "globals.h"
#include "buzzers.h"
#include <Adafruit_BNO08x.h>
#include <Adafruit_BMP5xx.h>
#include <SPI.h>
#include <Preferences.h>

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

    // Re-init VSPI bus fresh before sensor init (mimics test_tools probe sequence)
    SPI.end();
    delay(10);
    SPI.begin(VSPI_CLK, VSPI_MISO, VSPI_MOSI);
    delay(10);

    // Manual hardware reset of BNO085 (replicates test_tools sequence)
    pinMode(BNO_RST, OUTPUT);
    digitalWrite(BNO_RST, LOW);
    delay(20);
    digitalWrite(BNO_RST, HIGH);
    delay(50);

    // Ensure CS is deselected
    pinMode(BNO_CS, OUTPUT);
    digitalWrite(BNO_CS, HIGH);
    delay(1);

    if (!bno08x.begin_SPI(BNO_CS, BNO_INT, &SPI)) {
        Serial.println("[ERROR] BNO085 IMU not found on SPI!");
        Serial.printf("[DEBUG] BNO085 pins: CS=%d, INT=%d, RST=%d\n", BNO_CS, BNO_INT, BNO_RST);
        Serial.printf("[DEBUG] VSPI pins: MOSI=%d, MISO=%d, CLK=%d\n", VSPI_MOSI, VSPI_MISO, VSPI_CLK);
    } else {
        Serial.println("[OK] BNO085 IMU connected.");
        enableIMUReports();
        bnoInitialized = true;
        lastIMUReport_ms.store(millis(), std::memory_order_relaxed);
    }

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

    if (!success) {
        playTone(200, 500);
    }

    return success;
}

void checkInstruments() {
    Serial.println("\n========== INSTRUMENT CHECK ==========");
    Serial.printf("  BMP580 Barometer: %s\n", bmpInitialized ? "OK" : "FAIL");
    Serial.printf("  BNO085 IMU:       %s\n", bnoInitialized ? "OK" : "FAIL");
    Serial.printf("  GPS:              %s\n", sharedTelemetry.gpsUpdated ? "LOCKED" : "NO LOCK");
    Serial.printf("  SD Card:          %s\n", sdReady ? "OK" : "NOT MOUNTED");
    Serial.println("======================================\n");
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
                imuCalStatus.store(sh2_SensorValue.un.rotationVector.accuracy,
                                   std::memory_order_relaxed);
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

// ============================================================================
// BAROMETER CALIBRATION (ground level + QNH)
// ============================================================================
// Stores the result so the controller reboots into the same reference. A
// calibration record is tiny (~24 bytes) and only written on explicit
// calibration, so NVS flash wear is a non-issue.
//
// Validity window: a fresh calibration is preferred, but if no calibration
// has been performed and the device is rebooted on the launchpad, the stored
// value is loaded as a best-effort baseline. Stale (>BARO_CAL_VALIDITY_S)
// values are still loaded (logged as stale) but the operator is prompted to
// re-calibrate. (Pressure weather drift exceeds ~0.5 hPa / 6 hours, so a
// 24-hour cap is generous.)

struct BaroCalibration {
    uint32_t magic;            // BARO_CAL_MAGIC if valid
    uint32_t version;
    uint64_t calibratedAtEpoch_ms;
    float    baseline_altitude;
    float    qnh_pressure;
    float    ground_temperature_c;  // diagnostic
};

static constexpr uint32_t   BARO_CAL_MAGIC             = 0xBA40CA1B;  // "BARO-CAL"
static constexpr uint32_t   BARO_CAL_VERSION           = 1;
static constexpr unsigned long BARO_CAL_VALIDITY_S      = 24UL * 60 * 60;  // 24h
// Read availability of "is this stale" — older means less trustworthy but still applied.

// Always-on toggle: applied at every boot regardless of enableBrownoutRecovery.
// (Calibration is not flight-state; weather drift is the only concern.)
// To force a fresh calibration on every boot, disable this.
static constexpr bool        enableBaroCalRestoreOnBoot = true;

// Last-loaded calibration (kept in RAM for telemetry + diagnostics).
extern BaroCalibration baroCal;
extern const char *baroCalSource;          // "NVS", "manual", "auto", "none"

void baroCalibrationSampleAndCompute(BaroCalibration &out) {
    if (!bmpInitialized) {
        memset(&out, 0, sizeof(out));
        return;
    }
    float sum = 0.0f;
    float t_sum = 0.0f;
    float p_sum = 0.0f;
    int samples = 50;

    for (int i = 0; i < samples; i++) {
        if (bmp.performReading()) {
            sum += bmp.readAltitude(1013.25f);  // baseline uses ISA reference
            t_sum += bmp.temperature;
            p_sum += (bmp.pressure / 100.0f);
        }
        delay(40);
    }
    out.baseline_altitude = sum / samples;
    out.ground_temperature_c = t_sum / samples;
    out.qnh_pressure = p_sum / samples;  // measured mean pressure (basic QNH proxy)
    out.magic = BARO_CAL_MAGIC;
    out.version = BARO_CAL_VERSION;
    extern std::atomic<unsigned long long> systemBaseEpochMs;
    extern std::atomic<unsigned long>       systemBaseMillis;
    unsigned long long baseEp = systemBaseEpochMs.load(std::memory_order_relaxed);
    unsigned long      baseMs = systemBaseMillis.load(std::memory_order_relaxed);
    out.calibratedAtEpoch_ms = (baseEp > 0) ? (baseEp + (millis() - baseMs)) : 0;
}

void applyBaroCalibration(const BaroCalibration &c) {
    baseline_altitude = c.baseline_altitude;
    qnh_pressure      = c.qnh_pressure;
    previous_altitude = baseline_altitude;
}

// Persist to NVS. Always-on — writes happen only on explicit calibration.
void baroCalibrationSave(const BaroCalibration &c) {
    Preferences p;
    if (!p.begin("baroCal", false)) {
        write(LOG_BOTH, LOG_ERROR, "[BAROCAL] NVS open failed");
        return;
    }
    p.putBytes("cal", &c, sizeof(c));
    p.end();
    write(LOG_BOTH, LOG_INFO,
          "[BAROCAL] Saved to NVS — baseline=%.2fm qnh=%.2f hPa",
          c.baseline_altitude, c.qnh_pressure);
}

// Load from NVS. Returns true + fills `out` on success.
bool baroCalibrationLoad(BaroCalibration &out) {
    Preferences p;
    if (!p.begin("baroCal", true)) return false;
    size_t avail = p.getBytesLength("cal");
    if (avail != sizeof(BaroCalibration)) { p.end(); return false; }
    size_t got = p.getBytes("cal", &out, sizeof(out));
    p.end();
    return got == sizeof(out) && out.magic == BARO_CAL_MAGIC && out.version == BARO_CAL_VERSION;
}

void baroCalibrationInvalidate() {
    Preferences p;
    if (p.begin("baroCal", false)) {
        p.remove("cal");
        p.end();
    }
    write(LOG_BOTH, LOG_INFO, "[BAROCAL] Stored calibration invalidated");
}

void calibrateGroundAltitude() {
    if (!bmpInitialized) return;

    Serial.print("Calibrating launchpad ground level...");
    BaroCalibration c;
    baroCalibrationSampleAndCompute(c);
    applyBaroCalibration(c);
    baroCalibrationSave(c);
    baroCal = c;
    baroCalSource = "manual";
    Serial.printf(" Done! Baseline: %.2fm, QNH: %.2f hPa\n", baseline_altitude, qnh_pressure);
}

// Apply stored calibration at boot (always-on). Returns true if a calibration
// was loaded from NVS and applied. (Stale values are still applied — operator
// is warned in the log and on the dashboard.)
bool baroCalibrationRestoreOnBoot() {
    if (!enableBaroCalRestoreOnBoot) return false;
    BaroCalibration c;
    if (!baroCalibrationLoad(c)) {
        baroCalSource = "none";
        write(LOG_BOTH, LOG_INFO, "[BAROCAL] No stored calibration — baseline stays 0, qnh=ISA");
        return false;
    }
    applyBaroCalibration(c);
    baroCal = c;
    baroCalSource = "NVS";

    // Staleness diagnostic
    extern std::atomic<unsigned long long> systemBaseEpochMs;
    extern std::atomic<unsigned long>      systemBaseMillis;
    unsigned long long baseEp = systemBaseEpochMs.load(std::memory_order_relaxed);
    if (c.calibratedAtEpoch_ms > 0 && baseEp > 0) {
        unsigned long long nowEpoch = baseEp + (millis() - systemBaseMillis.load(std::memory_order_relaxed));
        unsigned long ageS = (unsigned long)((nowEpoch - c.calibratedAtEpoch_ms) / 1000ULL);
        if (ageS > BARO_CAL_VALIDITY_S) {
            write(LOG_BOTH, LOG_WARN,
                  "[BAROCAL] Stored calibration is STALE (%lus > %lus). Recommend re-calibrate on the pad.",
                  ageS, BARO_CAL_VALIDITY_S);
        } else {
            write(LOG_BOTH, LOG_INFO,
                  "[BAROCAL] Restored from NVS — age %lus, baseline=%.2fm, qnh=%.2f hPa",
                  ageS, c.baseline_altitude, c.qnh_pressure);
        }
    } else {
        write(LOG_BOTH, LOG_INFO,
              "[BAROCAL] Restored from NVS (no clock) — baseline=%.2fm, qnh=%.2f hPa",
              c.baseline_altitude, c.qnh_pressure);
    }
    return true;
}

#endif // GUIDANCE_INSTRUMENTS_H

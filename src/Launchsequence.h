#ifndef LAUNCHSEQUENCE_H
#define LAUNCHSEQUENCE_H

#include "globals.h"
#include "Arduino.h"
#include "instruments.h"


int enable5v(bool onOff) {
    if (onOff) {
        digitalWrite(Enable5VPin, HIGH);
        Enabled5V = true;
        return true;
    } else {
        digitalWrite(Enable5VPin, LOW);
        Enabled5V = false;
        return false;
    }
}

void firePyro(int pyroPin, int pulseDurationMs) {
    static uint32_t startTimes[70] = {0};
    static bool isPinActive[70] = {false};

    if (pyroPin < 0 || pyroPin >= 70) return;

    if (!isPinActive[pyroPin]) {
        pinMode(pyroPin, OUTPUT);
        digitalWrite(pyroPin, HIGH);
        startTimes[pyroPin] = millis();
        isPinActive[pyroPin] = true;
    } else {
        if (millis() - startTimes[pyroPin] >= (uint32_t)pulseDurationMs) {
            digitalWrite(pyroPin, LOW);
            isPinActive[pyroPin] = false;
        }
    }
}



// MAKE SURE TO ADD SPEED AND ACCELARATION CONSTRAINTS, plus looping retires
void fire_apogee_pyro() {
    if (filter_alt < minAltitudeForParachuteMeters) {
        write(LOG_BOTH, LOG_ERROR,
              "[ABORT PYRO] Apogee trigger rejected! Altitude (%.1fft) below safety floor (%.0fft).",
              filter_alt * METERS_TO_FEET, minAltitudeForParachuteMeters * METERS_TO_FEET);
        return;
    }

    bool armed = systemArmed.load(std::memory_order_relaxed);
    if (!armed) {
        write(LOG_BOTH, LOG_ERROR, "[ABORT PYRO] Safety trigger active but physical pyro rails are UNARMED.");
        return;
    }

    write(LOG_BOTH, LOG_INFO, "[DEPLOY] Safety constraints cleared. Firing apogee charge at %.1f feet!",
          filter_alt * METERS_TO_FEET);

    firePyro(parachutePyroPin, parachutePulseDurationMs);
}

int instrumentCheck() { // 0 is success, anything else is fail
    write(LOG_BOTH, LOG_INFO, "Running Instrument check");
    
    bool allCriticalPass = true;
    
    // ===== CRITICAL SENSORS (must pass) =====
    
    // Check BMP5xx Barometer
    BMP580good = (bmpInitialized && bmp.performReading()) ? 1 : 0;
    if (!BMP580good) {
        write(LOG_BOTH, LOG_ERROR, "[INSTRUMENT CHECK] BMP580 barometer FAILED");
        allCriticalPass = false;
    } else {
        write(LOG_BOTH, LOG_INFO, "[INSTRUMENT CHECK] BMP580 barometer OK");
    }
    
    // Check BNO085 IMU
    BNO080good = (bnoInitialized && !bno08x.wasReset()) ? 1 : 0;
    if (!BNO080good) {
        write(LOG_BOTH, LOG_ERROR, "[INSTRUMENT CHECK] BNO085 IMU FAILED");
        allCriticalPass = false;
    } else {
        write(LOG_BOTH, LOG_INFO, "[INSTRUMENT CHECK] BNO085 IMU OK");
    }
    
    // Check Servo Driver (PCA9685)
    servoGood = pwm.begin() ? 1 : 0;
    if (!servoGood) {
        write(LOG_BOTH, LOG_ERROR, "[INSTRUMENT CHECK] Servo Driver (PCA9685) FAILED");
        allCriticalPass = false;
    } else {
        write(LOG_BOTH, LOG_INFO, "[INSTRUMENT CHECK] Servo Driver OK");
    }
    
    // ===== OPTIONAL SENSORS (fail only if enabled) =====
    
    // Check GPS (only if enabled)
    if (enforceGPSLock) {
        // Try to read GPS data - if no valid fix, fail
        GPSgood = (gps.location.isValid() && gps.location.age() < 2000) ? 1 : 0;
        if (!GPSgood) {
            write(LOG_BOTH, LOG_ERROR, "[INSTRUMENT CHECK] GPS FAILED (but enabled)");
            allCriticalPass = false;
        } else {
            write(LOG_BOTH, LOG_INFO, "[INSTRUMENT CHECK] GPS OK");
        }
    } else {
        GPSgood = 1; // GPS not required, mark as OK
        write(LOG_BOTH, LOG_INFO, "[INSTRUMENT CHECK] GPS disabled - not checked");
    }
    
    // Check SD Card (warning only, not critical)
    SDgood = sdReady ? 1 : 0;
    if (!SDgood && enforceSDCard) {
        write(LOG_BOTH, LOG_WARN, "[INSTRUMENT CHECK] SD card not ready (but enabled)");
        allCriticalPass = false;
    } else {
        write(LOG_BOTH, LOG_INFO, "[INSTRUMENT CHECK] SD card OK or not enforced");
    }
    
    // ===== RETURN RESULT =====
    if (allCriticalPass) {
        write(LOG_BOTH, LOG_INFO, "[INSTRUMENT CHECK] All critical systems operational");
        return 0;
    } else {
        write(LOG_BOTH, LOG_ERROR, "[INSTRUMENT CHECK] One or more critical systems FAILED");
        return -1;
    }
}

int arm(bool armToggle) { //  bool armtoggle should be able to arm and disarm, true for arm

    if (!armToggle) {
        systemArmed.store(false, std::memory_order_relaxed);
        write(LOG_BOTH, LOG_INFO, "[ARM] System disarmed");
        return 0;
    }

    if (instrumentCheck() != 0) { 
        write(LOG_BOTH, LOG_ERROR, "Instrument Check failed, cannot arm");
        return -1;
    }
    
    if (armToggle) {
        systemArmed.store(true, std::memory_order_relaxed);
        write(LOG_BOTH, LOG_INFO, "[ARM] System armed");
        return 1;
    }
    
        return -2; // This should never be reached, but just in case
}

#endif // LAUNCHSEQUENCE_H

#ifndef LAUNCHSEQUENCE_H
#define LAUNCHSEQUENCE_H

#include "globals.h"
#include "Arduino.h"
#include "instruments.h"

#define METERS_TO_FEET 3.28084f

void enable5v(bool onOff) {
    if (onOff) {
        digitalWrite(Enable5VPin, HIGH);
    } else {
        digitalWrite(Enable5VPin, LOW);
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

void readyInit() {
}

#endif // LAUNCHSEQUENCE_H

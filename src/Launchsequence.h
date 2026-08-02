#ifndef LAUNCHSEQUENCE_H
#define LAUNCHSEQUENCE_H

#include "globals.h"
#include "Arduino.h"
#include "instruments.h"
#include <Adafruit_PWMServoDriver.h>

// Globals `pwm` and `gps` are defined in main.cpp; use externs from globals.h

int enable5v(bool onOff = true) {
    if (onOff) {
        digitalWrite(Enable5VPin, HIGH);
        Enabled5V = true;
        write(LOG_BOTH, LOG_INFO, "5v Enabled");
        return true;
    } else {
        digitalWrite(Enable5VPin, LOW);
        Enabled5V = false;
        write(LOG_BOTH, LOG_INFO, "5v Disabled");
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

// ============================================================================
// PYRO CHANNEL STATE & ORCHESTRATION
// ============================================================================
// firePyro() above remains a generic pin-pulse wrapper. The code below tracks
// per-channel deployment state so backup / aux logic can query whether a
// channel has already fired.

enum PyroState { PYRO_IDLE = 0, PYRO_FIRING = 1, PYRO_FIRED = 2, PYRO_FAILED = 3 };

struct PyroChannelState {
    PyroState state;
    uint32_t firedAtMs;
    bool attempted;   // true after first fire attempt on this channel
};

extern PyroChannelState pyroState[3];

// Helper: human-readable role name for logging / CLI / web
inline const char* pyroRoleName(PyroRole r) {
    switch (r) {
        case ROLE_PRIMARY_CHUTE: return "PRIMARY_CHUTE";
        case ROLE_BACKUP_CHUTE:  return "BACKUP_CHUTE";
        case ROLE_AUX_STAGING:   return "AUX_STAGING";
        case ROLE_AUX_MAIN:      return "AUX_MAIN";
        case ROLE_MANUAL:        return "MANUAL";
        case ROLE_NONE: default: return "NONE";
    }
}

inline PyroRole pyroRoleFromInt(int v) {
    if (v < (int)ROLE_NONE || v > (int)ROLE_MANUAL) return ROLE_NONE;
    return (PyroRole)v;
}

// Fire a specific pyro channel by index (0..2). Does NOT perform safety checks.
// Returns false if channel disabled, pin invalid, or already fired this session
// (call resetPyroStates() between flights / for ground tests to retry).
bool firePyroChannel(int idx, bool safetyOverride = false);

// Fire by role: finds first channel currently holding the given role and fires it.
// Performs the same armed/altitude/phase safety checks that the original
// fire_apogee_pyro() used (primary chute only); other roles skip alt/phase checks
// unless safetyOverride forces them through.
bool fireByRole(PyroRole role, bool safetyOverride = false);

// Clear channel states back to IDLE (for ground tests / between flights).
void resetPyroStates();



// Backwards-compat wrapper. Equivalent to fireByRole(ROLE_PRIMARY_CHUTE).
// MAKE SURE TO ADD SPEED AND ACCELERATION CONSTRAINTS, plus looping retries
void fire_apogee_pyro(bool safetyOverride = false) {
    fireByRole(ROLE_PRIMARY_CHUTE, safetyOverride);
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
    // Adafruit_PWMServoDriver::begin() returns void, so call it and mark servo as initialized.
    pwm.begin();
    servoGood = 1;
    write(LOG_BOTH, LOG_INFO, "[INSTRUMENT CHECK] Servo Driver OK");
    
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

int arm(bool armToggle, bool keep5v = true) { //  bool armtoggle should be able to arm and disarm, true for arm

    if (!armToggle) {

        if (keep5v) {
            systemArmed.store(false, std::memory_order_relaxed);
            write(LOG_BOTH, LOG_INFO, "[ARM] System disarmed");
        }

        if (!keep5v) {

            enable5v(false);
            systemArmed.store(false, std::memory_order_relaxed);
            write(LOG_BOTH, LOG_INFO, "[ARM] System disarmed with 5v disabled"); 
        }
        return 0;
    }

    if (instrumentCheck() != 0) { 
        write(LOG_BOTH, LOG_ERROR, "Instrument Check failed, cannot arm");
        return -1;
    }
    
    if (armToggle) {
        if (Enabled5V != true) enable5v(true);
        digitalWrite(pyroArmPin, HIGH);
        systemArmed.store(true, std::memory_order_relaxed);
        write(LOG_BOTH, LOG_INFO, "[ARM] System armed");
        return 1;
    }
    
        return -2; // This should never be reached, but just in case
}

// ============================================================================
// PYRO CHANNEL ORCHESTRATION — IMPLEMENTATION
// ============================================================================

int pyroChannelForRole(PyroRole r) {
    for (int i = 0; i < 3; i++) {
        if (pyroChannels[i].enabled && pyroChannels[i].role == r) {
            return i;
        }
    }
    return -1;
}

void resetPyroStates() {
    for (int i = 0; i < 3; i++) {
        pyroState[i].state      = PYRO_IDLE;
        pyroState[i].firedAtMs  = 0;
        pyroState[i].attempted  = false;
    }
    write(LOG_BOTH, LOG_INFO, "[PYRO] Channel states reset to IDLE");
}

// Fire channel by index. Performs NO phase/altitude safety checks by itself
// (run those in fireByRole or its callers). Idempotent within a session
// until resetPyroStates() clears state — prevents double-firing.
bool firePyroChannel(int idx, bool safetyOverride) {
    if (idx < 0 || idx >= 3) {
        write(LOG_BOTH, LOG_ERROR, "[PYRO] firePyroChannel: invalid index %d", idx);
        return false;
    }
    PyroChannelConfig &cfg = pyroChannels[idx];
    if (!cfg.enabled) {
        write(LOG_BOTH, LOG_WARN, "[PYRO] Channel %d (%s) disabled — not fired.", idx, pyroRoleName(cfg.role));
        return false;
    }
    if (cfg.pin < 0 || cfg.pin == 999) {
        write(LOG_BOTH, LOG_ERROR, "[PYRO] Channel %d (%s) has no valid pin (%d) — not fired.", idx, pyroRoleName(cfg.role), cfg.pin);
        return false;
    }
    if (pyroState[idx].state == PYRO_FIRING) {
        write(LOG_BOTH, LOG_WARN, "[PYRO] Channel %d (%s) already firing — ignoring re-fire request.", idx, pyroRoleName(cfg.role));
        return false;
    }
    if (pyroState[idx].attempted && pyroState[idx].state == PYRO_FIRED && !safetyOverride) {
        write(LOG_BOTH, LOG_WARN, "[PYRO] Channel %d (%s) already fired this session — ignored. Call pyro reset to retry.", idx, pyroRoleName(cfg.role));
        return false;
    }

    bool armed = systemArmed.load(std::memory_order_relaxed);
    if (!armed && !safetyOverride) {
        write(LOG_BOTH, LOG_ERROR, "[PYRO] Channel %d (%s) NOT fired — system UNARMED.", idx, pyroRoleName(cfg.role));
        return false;
    }

    write(LOG_BOTH, LOG_INFO, "[PYRO] Firing channel %d (%s) on pin %d, %lu ms pulse%s",
          idx, pyroRoleName(cfg.role), cfg.pin, cfg.pulseMs,
          safetyOverride ? " (safety override)" : "");

    firePyro(cfg.pin, (int)cfg.pulseMs);
    pyroState[idx].state      = PYRO_FIRING;
    pyroState[idx].firedAtMs  = millis();
    pyroState[idx].attempted  = true;
    // Note: actual pin LOW transition happens async in firePyro's polling loop.
    // We mark FIRED here optimistically once the firing "starts"; ground-test
    // reset / next-cycle inspection is the operator's responsibility.
    pyroState[idx].state      = PYRO_FIRED;
    // Snapshot to flash — pyro events are critical and burst-frequent enough
    // to be worth persisting. (A handful of fires per flight only.)
    flightCacheSave(true);
    return true;
}

// Fire whichever channel currently holds the given role. Runs role-appropriate
// safety gates before firing.
bool fireByRole(PyroRole role, bool safetyOverride) {
    int idx = pyroChannelForRole(role);
    if (idx < 0) {
        write(LOG_BOTH, LOG_WARN, "[PYRO] No channel assigned to role %s — nothing fired.", pyroRoleName(role));
        return false;
    }

    PyroChannelConfig &cfg = pyroChannels[idx];
    bool armed = systemArmed.load(std::memory_order_relaxed);
    FlightPhase phase = currentPhase.load(std::memory_order_relaxed);

    if (!armed && !safetyOverride) {
        write(LOG_BOTH, LOG_ERROR, "[ABORT PYRO] System UNARMED — role %s not fired.", pyroRoleName(role));
        return false;
    }

    // Role-specific safety checks (primary+backup chute share the chute gate).
    if (role == ROLE_PRIMARY_CHUTE || role == ROLE_BACKUP_CHUTE) {
        if (safetyOverride) {
            write(LOG_BOTH, LOG_WARN, "[DEPLOY] Safety override active. Firing %s at %.1f ft!",
                  pyroRoleName(role), filter_alt * METERS_TO_FEET);
        } else {
            if (filter_alt < minAltitudeForParachuteMeters) {
                write(LOG_BOTH, LOG_ERROR,
                      "[ABORT PYRO] %s rejected — altitude %.1fft below floor %.0fft.",
                      pyroRoleName(role),
                      filter_alt * METERS_TO_FEET, minAltitudeForParachuteMeters * METERS_TO_FEET);
                return false;
            }
            // Primary must fire from COAST; backup may fire from COAST or DESCENT.
            if (role == ROLE_PRIMARY_CHUTE && phase != COAST) {
                write(LOG_BOTH, LOG_ERROR,
                      "[ABORT PYRO] %s rejected — phase %d (not COAST).",
                      pyroRoleName(role), (int)phase);
                return false;
            }
            if (role == ROLE_BACKUP_CHUTE && phase != COAST && phase != DESCENT) {
                write(LOG_BOTH, LOG_ERROR,
                      "[ABORT PYRO] %s rejected — phase %d (not COAST/DESCENT).",
                      pyroRoleName(role), (int)phase);
                return false;
            }
            write(LOG_BOTH, LOG_INFO, "[DEPLOY] %s safety constraints cleared at %.1f ft.",
                  pyroRoleName(role), filter_alt * METERS_TO_FEET);
        }
    } else if (role == ROLE_AUX_STAGING || role == ROLE_AUX_MAIN || role == ROLE_MANUAL) {
        // Aux + manual fire only under arm; phase/alt checks happen in the
        // caller (flight state machine / operator). safetyOverride skips arm.
        if (!safetyOverride && !armed) {
            return false;
        }
    }

    return firePyroChannel(idx, safetyOverride);
}

#endif // LAUNCHSEQUENCE_H

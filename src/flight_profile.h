#ifndef GUIDANCE_FLIGHT_PROFILE_H
#define GUIDANCE_FLIGHT_PROFILE_H

#include "globals.h"
#include <math.h>
#include <string.h>
#include <ArduinoJson.h>
#include <SD.h>

// ============================================================================
// FLIGHT PROFILE SYSTEM
// ============================================================================
// Programmable attitude target sequences for custom flight profiles.
// Each profile is an ordered list of steps. Each step specifies a target
// attitude (roll/pitch/yaw), a trigger that ends the step, and a max rate.
// Setpoints are generated with a quintic S-curve (zero vel/accel at ends)
// to minimize servo jerk and mechanical stress.
// ============================================================================

// ---- Trigger Types ----
enum ProfileTrigger : uint8_t {
    TRIG_TIME     = 0,  // Step ends after durationSec
    TRIG_APOGEE   = 1,  // Step ends when apogee is reached (COAST->DESCENT)
    TRIG_ALTITUDE = 2,  // Step ends when altitude <= triggerValue (descending)
    TRIG_VELOCITY = 3,  // Step ends when |Vz| <= triggerValue
    TRIG_MANUAL   = 4   // Step ends only on manual next()/abort
};

// ---- Step Definition ----
struct ProfileStep {
    float targetRoll;       // deg
    float targetPitch;      // deg
    float targetYaw;        // deg
    float durationSec;      // used by TRIG_TIME
    uint8_t triggerType;    // ProfileTrigger
    float triggerValue;     // altitude (m) or velocity (m/s) for trigger
    float maxRateDegPerSec; // rate limit for this step (0 = unlimited)
};

// ---- Profile Definition ----
struct FlightProfile {
    const char* name;
    ProfileStep steps[16];
    uint8_t numSteps;
    bool loop;              // repeat profile after last step
};

// Flash-builtin profiles (defined in main.cpp)
extern const FlightProfile* const availableProfiles[];
extern const uint8_t numAvailableProfiles;

// ---- JSON Profile (heap-allocated, runtime-loaded from SD/HTTP) ----
struct JsonProfile {
    char name[32];
    ProfileStep steps[16];
    uint8_t numSteps;
    bool loop;
    bool valid;
};

// Trigger name <-> enum helpers (for JSON serialization)
static const char* triggerToName(uint8_t t) {
    switch (t) {
        case TRIG_TIME:     return "time";
        case TRIG_APOGEE:   return "apogee";
        case TRIG_ALTITUDE: return "altitude";
        case TRIG_VELOCITY: return "velocity";
        case TRIG_MANUAL:   return "manual";
        default:            return "time";
    }
}

static uint8_t triggerFromName(const char* s) {
    if (!s) return TRIG_TIME;
    if (!strcasecmp(s, "apogee"))   return TRIG_APOGEE;
    if (!strcasecmp(s, "altitude")) return TRIG_ALTITUDE;
    if (!strcasecmp(s, "velocity")) return TRIG_VELOCITY;
    if (!strcasecmp(s, "manual"))   return TRIG_MANUAL;
    return TRIG_TIME;
}

// ---- Safety Limits ----
static constexpr float MAX_PROFILE_ROLL   = 180.0f;
static constexpr float MAX_PROFILE_PITCH  = 90.0f;
static constexpr float MAX_PROFILE_YAW    = 180.0f;
static constexpr float PROFILE_STEP_TIMEOUT_SEC = 30.0f;  // watchdog per step

// ============================================================================
// FLIGHT PROFILE ENGINE
// ============================================================================
class FlightProfileEngine {
public:
    void startProfile(const FlightProfile& p) {
        profile = &p;
        currentStep = 0;
        stepStartMs = millis();
        active = true;
        stepComplete = false;
        // Capture current attitude as start point for smooth blending
        startSetpoint[0] = currentSetpoint[0];
        startSetpoint[1] = currentSetpoint[1];
        startSetpoint[2] = currentSetpoint[2];
        loadStepTarget();
        write(LOG_BOTH, LOG_INFO, "[PROFILE] Started '%s' (step 0/%u)",
              p.name, (unsigned)p.numSteps);
    }

    void abortProfile() {
        if (!active) return;
        active = false;
        currentStep = 0;
        write(LOG_BOTH, LOG_WARN, "[PROFILE] Aborted '%s'",
              profile ? profile->name : "?");
        profile = nullptr;
    }

    // Called every loop iteration from main.cpp after sensor fusion.
    void update(float dt, float currentAltitude, float currentVelocity,
                FlightPhase phase, bool apogeeDetected) {
        if (!active || profile == nullptr) {
            // Idle: decay setpoint toward level (0,0,0) for safe recovery
            decayToLevel(dt);
            return;
        }

        // Phase guard: never run during ground phases
        if (phase == TRANSPORT || phase == PAD || phase == READY) {
            abortProfile();
            decayToLevel(dt);
            return;
        }

        // Safety: require system armed to execute maneuvers
        if (!systemArmed.load(std::memory_order_relaxed)) {
            abortProfile();
            decayToLevel(dt);
            return;
        }

        bool stepDone = false;

        switch (profile->steps[currentStep].triggerType) {
            case TRIG_TIME: {
                float elapsed = (millis() - stepStartMs) / 1000.0f;
                if (elapsed >= profile->steps[currentStep].durationSec) stepDone = true;
                break;
            }
            case TRIG_APOGEE:
                if (apogeeDetected) stepDone = true;
                break;
            case TRIG_ALTITUDE:
                if (currentAltitude <= profile->steps[currentStep].triggerValue) stepDone = true;
                break;
            case TRIG_VELOCITY:
                if (fabsf(currentVelocity) <= profile->steps[currentStep].triggerValue) stepDone = true;
                break;
            case TRIG_MANUAL:
                stepDone = stepComplete;  // only via next()/abort()
                break;
        }

        // Watchdog: force-advance if step runs too long
        float elapsed = (millis() - stepStartMs) / 1000.0f;
        if (elapsed > PROFILE_STEP_TIMEOUT_SEC) {
            write(LOG_BOTH, LOG_WARN, "[PROFILE] Step %u timeout — force advancing",
                  (unsigned)currentStep);
            stepDone = true;
        }

        if (stepDone) {
            advanceStep();
        }

        // Compute S-curve interpolated setpoint for current step
        computeSetpoints(dt);
    }

    // Manual step advance (for TRIG_MANUAL)
    void next() {
        if (active) {
            stepComplete = true;
        }
    }

    // ---- Accessors ----
    float getSetpointRoll()  const { return currentSetpoint[0]; }
    float getSetpointPitch() const { return currentSetpoint[1]; }
    float getSetpointYaw()   const { return currentSetpoint[2]; }
    bool isActive()          const { return active; }
    uint8_t getCurrentStep() const { return currentStep; }
    uint8_t getNumSteps()    const { return profile ? profile->numSteps : 0; }
    const char* getProfileName() const { return profile ? profile->name : "NONE"; }
    float getStepProgress()  const {
        if (!active || profile == nullptr) return 0.0f;
        float dur = profile->steps[currentStep].durationSec;
        if (dur <= 0.0f) return 0.0f;
        float elapsed = (millis() - stepStartMs) / 1000.0f;
        return constrain(elapsed / dur, 0.0f, 1.0f);
    }

private:
    const FlightProfile* profile = nullptr;
    uint8_t currentStep = 0;
    uint32_t stepStartMs = 0;
    bool active = false;
    bool stepComplete = false;

    float startSetpoint[3]  = {0, 0, 0};
    float currentSetpoint[3] = {0, 0, 0};
    float targetSetpoint[3]  = {0, 0, 0};

    void loadStepTarget() {
        targetSetpoint[0] = constrain(profile->steps[currentStep].targetRoll,
                                      -MAX_PROFILE_ROLL, MAX_PROFILE_ROLL);
        targetSetpoint[1] = constrain(profile->steps[currentStep].targetPitch,
                                      -MAX_PROFILE_PITCH, MAX_PROFILE_PITCH);
        targetSetpoint[2] = constrain(profile->steps[currentStep].targetYaw,
                                      -MAX_PROFILE_YAW, MAX_PROFILE_YAW);
        startSetpoint[0] = currentSetpoint[0];
        startSetpoint[1] = currentSetpoint[1];
        startSetpoint[2] = currentSetpoint[2];
        stepStartMs = millis();
        stepComplete = false;
    }

    void advanceStep() {
        if (profile == nullptr) return;

        if (currentStep + 1 >= profile->numSteps) {
            if (profile->loop) {
                currentStep = 0;
                loadStepTarget();
                write(LOG_BOTH, LOG_INFO, "[PROFILE] Loop restart '%s'", profile->name);
            } else {
                write(LOG_BOTH, LOG_INFO, "[PROFILE] Completed '%s' — returning to level",
                      profile->name);
                active = false;
                profile = nullptr;
            }
        } else {
            currentStep++;
            loadStepTarget();
            write(LOG_BOTH, LOG_INFO, "[PROFILE] -> step %u/%u (R%.0f P%.0f Y%.0f)",
                  (unsigned)currentStep, (unsigned)profile->numSteps,
                  targetSetpoint[0], targetSetpoint[1], targetSetpoint[2]);
        }
    }

    void computeSetpoints(float dt) {
        if (!active || profile == nullptr) return;

        float dur = profile->steps[currentStep].durationSec;
        float maxRate = profile->steps[currentStep].maxRateDegPerSec;

        // Normalized progress for S-curve
        float t;
        if (dur > 0.0f) {
            t = constrain((millis() - stepStartMs) / 1000.0f / dur, 0.0f, 1.0f);
        } else {
            t = 1.0f;  // instantaneous (trigger-based steps snap to target)
        }

        // Quintic blend: zero velocity & acceleration at both ends
        float s = quinticBlend(t);

        for (int i = 0; i < 3; i++) {
            float desired = startSetpoint[i] + (targetSetpoint[i] - startSetpoint[i]) * s;

            // Apply per-step rate limit
            if (maxRate > 0.0f && dt > 0.0f) {
                float maxDelta = maxRate * dt;
                float delta = desired - currentSetpoint[i];
                if (delta > maxDelta) desired = currentSetpoint[i] + maxDelta;
                else if (delta < -maxDelta) desired = currentSetpoint[i] - maxDelta;
            }

            currentSetpoint[i] = desired;
        }
    }

    void decayToLevel(float dt) {
        // Exponential relaxation to level attitude (safe default)
        float rate = 30.0f;  // deg/sec relaxation
        for (int i = 0; i < 3; i++) {
            float delta = 0.0f - currentSetpoint[i];
            float maxDelta = rate * dt;
            if (delta > maxDelta) currentSetpoint[i] += maxDelta;
            else if (delta < -maxDelta) currentSetpoint[i] -= maxDelta;
            else currentSetpoint[i] = 0.0f;
        }
    }

    static float quinticBlend(float t) {
        // 10t^3 - 15t^4 + 6t^5 : smooth S-curve, zero 1st/2nd derivative at ends
        if (t <= 0.0f) return 0.0f;
        if (t >= 1.0f) return 1.0f;
        return t * t * t * (10.0f + t * (-15.0f + 6.0f * t));
    }
};

extern FlightProfileEngine profileEngine;

// ============================================================================
// PROFILE STORAGE MANAGER
// ============================================================================
// Manages runtime-loaded profiles from SD card (/profiles/*.json) and HTTP
// uploads. Keeps a registry of loaded JsonProfiles + matching FlightProfiles
// so the engine can reference them by name. Thread-safe via webServerMutex.
// ============================================================================

#ifndef PROFILE_STORE_PATH
#define PROFILE_STORE_PATH "/profiles"
#endif
#ifndef PROFILE_STORE_MAX
#define PROFILE_STORE_MAX 8
#endif

struct LoadedProfile {
    JsonProfile json;          // owns the name buffer + steps
    FlightProfile flight;      // name points into json.name
    bool used;
};

static LoadedProfile loadedProfiles[PROFILE_STORE_MAX];
static uint8_t numLoadedProfiles = 0;

// Forward declarations (defined later, after JSON helpers)
static bool parseProfileJSON(const char* json, JsonProfile& out);
static void jsonProfileToFlightProfile(const JsonProfile& src, FlightProfile& dst);

// Find a loaded profile by name (case-insensitive). Returns index or -1.
static int findLoadedProfile(const char* name) {
    for (uint8_t i = 0; i < numLoadedProfiles; i++) {
        if (loadedProfiles[i].used && !strcasecmp(loadedProfiles[i].json.name, name)) {
            return (int)i;
        }
    }
    return -1;
}

// Register a parsed JsonProfile into the store and return its FlightProfile*,
// or nullptr if store is full.
static const FlightProfile* registerJsonProfile(const JsonProfile& p) {
    int idx = findLoadedProfile(p.name);
    if (idx < 0) {
        if (numLoadedProfiles >= PROFILE_STORE_MAX) {
            write(LOG_BOTH, LOG_ERROR, "[PROFILE] Store full (%u), cannot load '%s'",
                  PROFILE_STORE_MAX, p.name);
            return nullptr;
        }
        idx = numLoadedProfiles++;
        loadedProfiles[idx].used = true;
    }
    loadedProfiles[idx].json = p;
    jsonProfileToFlightProfile(loadedProfiles[idx].json, loadedProfiles[idx].flight);
    return &loadedProfiles[idx].flight;
}

// Load a single profile JSON file from SD. Returns true on success.
static bool loadProfileFromSD(const char* filepath) {
    if (!sdReady) return false;
    File f = SD.open(filepath, FILE_READ);
    if (!f) {
        write(LOG_BOTH, LOG_ERROR, "[PROFILE] Cannot open %s", filepath);
        return false;
    }
    // ArduinoJson v7: use readBytes to fill a Doc, or read into a String
    String contents;
    while (f.available()) {
        contents += (char)f.read();
    }
    f.close();

    JsonProfile p;
    if (!parseProfileJSON(contents.c_str(), p)) return false;

    const FlightProfile* fp = registerJsonProfile(p);
    if (!fp) return false;

    write(LOG_BOTH, LOG_INFO, "[PROFILE] Loaded '%s' from %s", p.name, filepath);
    return true;
}

// Scan /profiles directory on SD and load all .json files
static void loadAllProfilesFromSD() {
    if (!sdReady) return;
    if (!SD.exists(PROFILE_STORE_PATH)) {
        SD.mkdir(PROFILE_STORE_PATH);
        write(LOG_BOTH, LOG_INFO, "[PROFILE] Created %s directory", PROFILE_STORE_PATH);
        return;
    }
    File root = SD.open(PROFILE_STORE_PATH);
    if (!root || !root.isDirectory()) {
        write(LOG_BOTH, LOG_WARN, "[PROFILE] %s not a directory", PROFILE_STORE_PATH);
        return;
    }
    File entry;
    int count = 0;
    while ((entry = root.openNextFile())) {
        if (!entry.isDirectory()) {
            String name = entry.name();
            if (name.endsWith(".json") || name.endsWith(".JSON")) {
                // Build full path
                char full[64];
                snprintf(full, sizeof(full), "%s/%s", PROFILE_STORE_PATH, name.c_str());
                if (loadProfileFromSD(full)) count++;
            }
        }
        entry.close();
    }
    root.close();
    write(LOG_BOTH, LOG_INFO, "[PROFILE] Loaded %d profile(s) from SD", count);
}

// Write a profile JSON to SD (used by HTTP upload). Returns true on success.
static bool saveProfileToSD(const char* name, const char* json) {
    if (!sdReady) return false;
    if (!SD.exists(PROFILE_STORE_PATH)) SD.mkdir(PROFILE_STORE_PATH);

    char full[64];
    snprintf(full, sizeof(full), "%s/%s.json", PROFILE_STORE_PATH, name);

    File f = SD.open(full, FILE_WRITE);
    if (!f) {
        write(LOG_BOTH, LOG_ERROR, "[PROFILE] Cannot create %s", full);
        return false;
    }
    f.print(json);
    f.close();
    write(LOG_BOTH, LOG_INFO, "[PROFILE] Saved '%s' to %s", name, full);
    return true;
}

// Start a profile by name (checks flash profiles first, then SD-loaded)
static bool startProfileByName(const char* name) {
    // 1. Flash-builtin profiles
    for (uint8_t i = 0; i < numAvailableProfiles; i++) {
        if (!strcasecmp(availableProfiles[i]->name, name)) {
            profileEngine.startProfile(*availableProfiles[i]);
            return true;
        }
    }
    // 2. SD-loaded profiles
    int idx = findLoadedProfile(name);
    if (idx >= 0) {
        profileEngine.startProfile(loadedProfiles[idx].flight);
        return true;
    }
    write(LOG_BOTH, LOG_ERROR, "[PROFILE] No profile named '%s' found", name);
    return false;
}

// ============================================================================
// JSON PROFILE (DE)SERIALIZATION
// ============================================================================
// JSON schema:
// {
//   "name": "MY_PROFILE",
//   "loop": false,
//   "steps": [
//     { "roll": 180, "pitch": 0, "yaw": 0, "duration": 5.0,
//       "trigger": "time", "value": 0, "rate": 40 },
//     { "roll": 0, "pitch": 0, "yaw": 0, "trigger": "apogee", "rate": 25 }
//   ]
// }
// ============================================================================

// Parse a JSON string into a JsonProfile (heap-allocated name-safe copy)
static bool parseProfileJSON(const char* json, JsonProfile& out) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, json);
    if (err) {
        write(LOG_BOTH, LOG_ERROR, "[PROFILE] JSON parse error: %s", err.c_str());
        out.valid = false;
        return false;
    }

    const char* nm = doc["name"] | "UNNAMED";
    strncpy(out.name, nm, sizeof(out.name) - 1);
    out.name[sizeof(out.name) - 1] = '\0';

    out.loop = doc["loop"] | false;

    JsonArray steps = doc["steps"];
    if (steps.isNull() || steps.size() == 0) {
        write(LOG_BOTH, LOG_ERROR, "[PROFILE] JSON has no 'steps' array");
        out.valid = false;
        return false;
    }

    uint8_t n = 0;
    for (JsonObject step : steps) {
        if (n >= 16) break;
        out.steps[n].targetRoll  = step["roll"]  | 0.0f;
        out.steps[n].targetPitch = step["pitch"] | 0.0f;
        out.steps[n].targetYaw   = step["yaw"]   | 0.0f;
        out.steps[n].durationSec = step["duration"] | 0.0f;
        out.steps[n].triggerType = triggerFromName(step["trigger"] | "time");
        out.steps[n].triggerValue = step["value"] | 0.0f;
        out.steps[n].maxRateDegPerSec = step["rate"] | 0.0f;
        n++;
    }
    out.numSteps = n;
    out.valid = (n > 0);

    if (out.valid) {
        write(LOG_BOTH, LOG_INFO, "[PROFILE] Parsed '%s' with %u steps from JSON",
              out.name, (unsigned)n);
    }
    return out.valid;
}

// Serialize a JsonProfile back to a JSON string (caller provides buffer)
static void serializeProfileJSON(const JsonProfile& p, char* buf, size_t bufSize) {
    JsonDocument doc;
    doc["name"] = p.name;
    doc["loop"] = p.loop;
    JsonArray steps = doc["steps"].to<JsonArray>();
    for (uint8_t i = 0; i < p.numSteps; i++) {
        JsonObject s = steps.add<JsonObject>();
        s["roll"]  = p.steps[i].targetRoll;
        s["pitch"] = p.steps[i].targetPitch;
        s["yaw"]   = p.steps[i].targetYaw;
        s["duration"] = p.steps[i].durationSec;
        s["trigger"]  = triggerToName(p.steps[i].triggerType);
        s["value"]    = p.steps[i].triggerValue;
        s["rate"]     = p.steps[i].maxRateDegPerSec;
    }
    serializeJson(doc, buf, bufSize);
}

// Convert a JsonProfile (heap) into a FlightProfile (name points to jsonProfile.name)
static void jsonProfileToFlightProfile(const JsonProfile& src, FlightProfile& dst) {
    dst.name = src.name;
    dst.numSteps = src.numSteps;
    dst.loop = src.loop;
    for (uint8_t i = 0; i < src.numSteps; i++) {
        dst.steps[i] = src.steps[i];
    }
}

#endif // GUIDANCE_FLIGHT_PROFILE_H

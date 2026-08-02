// ============================================================================
// PYRO TEST BENCH — standalone igniter / pyro channel tester
//
// Purpose: bench-test pyro channels & igniters with a web UI built ONLY for
// firing. No flight logic, no sensors, no auto-fire. Same physical pins as the
// flight program (src/globals.h) by default; each channel's pin + pulse length
// can be changed from the UI and persists to NVS.
//
// Safety:
//   - Two-step arming: press ARM -> get 4-digit code -> confirm with code.
//     The 5V rail (Enable5VPin) + arm rail (pyroArmPin) only go HIGH when armed.
//   - Auto-disarm: after ARMED_TIMEOUT_S seconds without a fire, the system
//     disarms itself.
//   - Fires are rejected while disarmed; pulse length is clamped to
//     MAX_PULSE_MS; each fire logs start/end with micros() precision and the
//     measured pulse duration is shown in the UI.
//   - Firmware OTA on port 81 (same layout as the flight program).
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <ElegantOTA.h>

#include "../globals.h"   // pyro1Pin..3, Enable5VPin, pyroArmPin, AP_SSID/AP_PASSWORD
#include "web_ui.h"

// ----------------------------------------------------------------------------
// Config (persisted to NVS)
// ----------------------------------------------------------------------------
static const int    NUM_CHANNELS     = 3;
static const char   NVS_NS[]         = "pyrotest";
static const char   NVS_KEY[]        = "config";

static const unsigned long MAX_PULSE_MS    = 5000;   // hard clamp for fire pulses
static const unsigned long ARMED_TIMEOUT_S = 90;     // auto-disarm after this long armed
static const unsigned long ARM_CODE_VALID_S = 10;    // arm code expires after this

struct ChannelCfg {
    int  pin;                  // GPIO number (999 = not connected)
    unsigned long pulseMs;     // pulse duration
    bool enabled;
    char label[20];
};

struct FireEvent {             // last fire result per channel
    bool   fired;
    uint32_t startedAtMs;
    uint64_t startedUs;        // micros() when pin went HIGH
    uint64_t endedUs;          // micros() when pin went LOW (0 = still firing)
    unsigned long requestedMs;
    unsigned long measuredUs;  // actual HIGH time in microseconds
};

ChannelCfg channels[NUM_CHANNELS];
FireEvent  fireState[NUM_CHANNELS];

bool        armed           = false;
unsigned long armedAtMs     = 0;
unsigned long armPendingMs  = 0;   // when the code was issued (0 = none pending)
int         armCode         = 0;

Preferences prefs;

WebServer server(80);
WebServer otaServer(81);

// ----------------------------------------------------------------------------
// Event log (RAM ring, also mirrored to Serial)
// ----------------------------------------------------------------------------
static const int LOG_LINES = 60;
static String logLines[LOG_LINES];
static int  logHead = 0;

void addLog(const char* fmt, ...) {
    char buf[160];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    logLines[logHead % LOG_LINES] = String(buf);
    logHead++;
    Serial.println(buf);
}

String logHtml() {
    String out;
    int start = logHead > LOG_LINES ? logHead - LOG_LINES : 0;
    for (int i = start; i < logHead; i++) {
        out += logLines[i % LOG_LINES];
        out += '\n';
    }
    return out;
}

// ----------------------------------------------------------------------------
// Channel helpers
// ----------------------------------------------------------------------------
static void setPin(int pin, bool high) {
    if (pin < 0 || pin == 999) return;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, high ? HIGH : LOW);
}

void saveConfig() {
    prefs.begin(NVS_NS, false);
    prefs.putBytes(NVS_KEY, channels, sizeof(channels));
    prefs.end();
}

void loadConfig() {
    prefs.begin(NVS_NS, true);
    size_t avail = prefs.getBytesLength(NVS_KEY);
    bool loaded = false;
    if (avail == sizeof(channels)) {
        loaded = prefs.getBytes(NVS_KEY, channels, sizeof(channels)) == sizeof(channels);
    }
    prefs.end();

    if (!loaded) {
        const struct { int pin; unsigned long ms; const char* label; } dflt[NUM_CHANNELS] = {
            { pyro1Pin, 75,  "Pyro 1" },
            { pyro2Pin, 75,  "Pyro 2" },
            { pyro3Pin, 100, "Pyro 3" },
        };
        for (int i = 0; i < NUM_CHANNELS; i++) {
            channels[i].pin      = dflt[i].pin;
            channels[i].pulseMs  = dflt[i].ms;
            channels[i].enabled  = true;
            snprintf(channels[i].label, sizeof(channels[i].label), "%s", dflt[i].label);
        }
        saveConfig();
        addLog("[CFG] No stored config — loaded defaults from globals.h (pyro1-3 pins)");
    } else {
        addLog("[CFG] Config restored from NVS");
    }
}

// ----------------------------------------------------------------------------
// Pulse engine — polled in loop(); releases pins at the exact pulse end
// ----------------------------------------------------------------------------
void pulseTick() {
    for (int i = 0; i < NUM_CHANNELS; i++) {
        if (fireState[i].fired && fireState[i].endedUs == 0) {
            unsigned long elapsed = millis() - fireState[i].startedAtMs;
            if (elapsed >= channels[i].pulseMs) {
                setPin(channels[i].pin, false);
                fireState[i].endedUs = micros();
                fireState[i].measuredUs = fireState[i].endedUs - fireState[i].startedUs;
                addLog("[FIRE] ch%d (%s) released after %lu us (requested %lu ms)",
                       i, channels[i].label,
                       (unsigned long)fireState[i].measuredUs,
                       channels[i].pulseMs);
            }
        }
    }
}

// ----------------------------------------------------------------------------
// Arming
// ----------------------------------------------------------------------------
void doDisarm(const char* why) {
    if (!armed && armPendingMs == 0) return;
    armed = false;
    armPendingMs = 0;
    setPin(Enable5VPin, false);
    setPin(pyroArmPin, false);
    addLog("[ARM] DISARMED — %s", why);
}

bool doArm(int code) {
    if (armPendingMs == 0) {
        addLog("[ARM] Rejected: no arm code pending. Press ARM first.");
        return false;
    }
    if (millis() - armPendingMs > ARM_CODE_VALID_S * 1000UL) {
        armPendingMs = 0;
        addLog("[ARM] Rejected: code expired. Request a new one.");
        return false;
    }
    if (code != armCode) {
        addLog("[ARM] Rejected: wrong code.");
        return false;
    }

    armPendingMs = 0;
    armed = true;
    armedAtMs = millis();
    setPin(Enable5VPin, true);
    setPin(pyroArmPin, true);
    addLog("[ARM] ARMED (5V rail + arm rail HIGH). Auto-disarm in %lu s.", ARMED_TIMEOUT_S);
    return true;
}

// ----------------------------------------------------------------------------
// Firing
// ----------------------------------------------------------------------------
bool fireChannel(int idx, unsigned long ms) {
    if (!armed) {
        addLog("[FIRE] REJECTED ch%d (%s): system DISARMED", idx, channels[idx].label);
        return false;
    }
    if (idx < 0 || idx >= NUM_CHANNELS) return false;
    ChannelCfg &c = channels[idx];
    if (!c.enabled) {
        addLog("[FIRE] REJECTED ch%d (%s): channel disabled", idx, c.label);
        return false;
    }
    if (c.pin < 0 || c.pin == 999) {
        addLog("[FIRE] REJECTED ch%d (%s): no pin wired (pin=999)", idx, c.label);
        return false;
    }
    if (fireState[idx].fired && fireState[idx].endedUs == 0) {
        addLog("[FIRE] REJECTED ch%d (%s): already firing", idx, c.label);
        return false;
    }

    unsigned long pulse = (ms > 0) ? min(ms, MAX_PULSE_MS) : min(c.pulseMs, MAX_PULSE_MS);

    setPin(c.pin, true);
    fireState[idx].fired       = true;
    fireState[idx].startedAtMs = millis();
    fireState[idx].startedUs   = micros();
    fireState[idx].endedUs     = 0;
    fireState[idx].measuredUs  = 0;
    fireState[idx].requestedMs = pulse;
    addLog("[FIRE] ch%d (%s) pin %d HIGH — pulse %lu ms", idx, c.label, c.pin, pulse);
    return true;
}

void fireAll(unsigned long ms) {
    for (int i = 0; i < NUM_CHANNELS; i++) fireChannel(i, ms);
}

// ----------------------------------------------------------------------------
// Web handlers
// ----------------------------------------------------------------------------
String jsonStatus() {
    unsigned long armedForS = armed ? (millis() - armedAtMs) / 1000UL : 0;
    String out = "{\"armed\":" + String(armed ? "true" : "false") + ",";
    out += "\"armed_for_s\":" + String(armedForS) + ",";
    out += "\"auto_disarm_s\":" + String(ARMED_TIMEOUT_S) + ",";
    out += "\"arm_pending\":" + String(armPendingMs ? "true" : "false") + ",";
    out += "\"uptime_s\":" + String(millis() / 1000UL) + ",";
    out += "\"channels\":[";
    for (int i = 0; i < NUM_CHANNELS; i++) {
        if (i) out += ",";
        ChannelCfg &c = channels[i];
        FireEvent  &f = fireState[i];
        out += "{\"ch\":" + String(i);
        out += ",\"pin\":" + String(c.pin);
        out += ",\"pulse_ms\":" + String(c.pulseMs);
        out += ",\"enabled\":" + String(c.enabled ? "true" : "false");
        out += ",\"label\":\"" + String(c.label) + "\"";
        out += ",\"fired\":" + String(f.fired ? "true" : "false");
        out += ",\"firing\":" + String((f.fired && f.endedUs == 0) ? "true" : "false");
        out += ",\"measured_us\":" + String((unsigned long)f.measuredUs);
        out += ",\"started_ms\":" + String(f.startedAtMs);
        out += "}";
    }
    out += "]}";
    return out;
}

void handleStatus()  { server.send(200, "application/json", jsonStatus()); }

void handleArmBegin() {
    if (armed) { server.send(400, "text/plain", "Already ARMED — disarm first."); return; }
    armCode = random(1000, 10000);
    armPendingMs = millis();
    addLog("[ARM] Arm code issued (%d) — valid %lu s", armCode, ARM_CODE_VALID_S);
    server.send(200, "text/plain", String(armCode));
}

void handleArm() {
    if (armed) { server.send(400, "text/plain", "Already ARMED — disarm first."); return; }
    if (!server.hasArg("code")) { server.send(400, "text/plain", "Missing ?code="); return; }
    int code = server.arg("code").toInt();
    if (doArm(code)) server.send(200, "text/plain", "ARMED");
    else             server.send(400, "text/plain", "Arm rejected (see log)");
}

void handleDisarm() {
    doDisarm("operator request");
    server.send(200, "text/plain", "DISARMED");
}

void handleConfig() {
    if (armed) { server.send(400, "text/plain", "Disarm before changing config."); return; }
    if (!server.hasArg("ch")) { server.send(400, "text/plain", "Missing ?ch="); return; }
    int ch = server.arg("ch").toInt();
    if (ch < 0 || ch >= NUM_CHANNELS) { server.send(400, "text/plain", "ch out of range"); return; }

    ChannelCfg &c = channels[ch];
    if (server.hasArg("pin")) {
        int pin = server.arg("pin").toInt();
        c.pin = (pin < 0 || pin > 47) ? 999 : pin;   // S3 GPIO range 0-47
    }
    if (server.hasArg("ms")) {
        unsigned long ms = server.arg("ms").toInt();
        c.pulseMs = constrain(ms, 1UL, MAX_PULSE_MS);
    }
    if (server.hasArg("en")) c.enabled = server.arg("en") == "1" || server.arg("en") == "true";
    if (server.hasArg("label")) {
        String lbl = server.arg("label");
        lbl.trim();
        lbl = lbl.substring(0, sizeof(c.label) - 1);
        lbl.toCharArray(c.label, sizeof(c.label));
    }
    saveConfig();
    addLog("[CFG] ch%d -> pin=%d pulse=%lu ms enabled=%s label=%s",
           ch, c.pin, c.pulseMs, c.enabled ? "Y" : "N", c.label);
    server.send(200, "text/plain", "saved");
}

void handleFire() {
    if (!server.hasArg("ch")) { server.send(400, "text/plain", "Missing ?ch="); return; }
    int ch = server.arg("ch").toInt();
    unsigned long ms = server.hasArg("ms") ? server.arg("ms").toInt() : 0;
    if (fireChannel(ch, ms)) server.send(200, "text/plain", "fired");
    else                     server.send(400, "text/plain", "rejected (see log)");
}

void handleFireAll() {
    unsigned long ms = server.hasArg("ms") ? server.arg("ms").toInt() : 0;
    fireAll(ms);
    server.send(200, "text/plain", "fired all (see log)");
}

void handleResetFire() {
    for (int i = 0; i < NUM_CHANNELS; i++) {
        fireState[i].fired = false;
        fireState[i].measuredUs = 0;
        fireState[i].startedAtMs = 0;
        fireState[i].endedUs = 0;
    }
    addLog("[LOG] Fire results cleared");
    server.send(200, "text/plain", "cleared");
}

void handleLog()  { server.send(200, "text/plain", logHtml()); }
void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

// ----------------------------------------------------------------------------
// Setup / loop
// ----------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== PYRO TEST BENCH ===");

    randomSeed(esp_random());
    loadConfig();
    for (int i = 0; i < NUM_CHANNELS; i++) {
        setPin(channels[i].pin, false);          // ensure all channels idle LOW
    }
    setPin(Enable5VPin, false);                  // 5V rail OFF at boot
    setPin(pyroArmPin, false);                   // arm rail OFF at boot

    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.printf("[WIFI] AP live: %s  IP: ", AP_SSID);
    Serial.println(WiFi.softAPIP());

    server.on("/", HTTP_GET, handleRoot);
    server.on("/status", HTTP_GET, handleStatus);
    server.on("/arm_begin", HTTP_POST, handleArmBegin);
    server.on("/arm", HTTP_POST, handleArm);
    server.on("/disarm", HTTP_POST, handleDisarm);
    server.on("/config", HTTP_POST, handleConfig);
    server.on("/fire", HTTP_POST, handleFire);
    server.on("/fire_all", HTTP_POST, handleFireAll);
    server.on("/reset_fire", HTTP_POST, handleResetFire);
    server.on("/log", HTTP_GET, handleLog);
    server.begin();

    ElegantOTA.begin(&otaServer);
    ElegantOTA.setAutoReboot(true);
    ElegantOTA.onStart([]()   { Serial.println("[OTA] Update started"); });
    ElegantOTA.onProgress([](size_t cur, size_t tot) {
        static unsigned long last = 0;
        if (millis() - last > 1000) {
            last = millis();
            Serial.printf("[OTA] Progress: %u / %u bytes\n", (unsigned)cur, (unsigned)tot);
        }
    });
    ElegantOTA.onEnd([](bool ok) { Serial.printf("[OTA] Update %s\n", ok ? "OK" : "FAILED"); });
    otaServer.begin();

    Serial.println("[WEB] http://<ap-ip>/   (OTA: http://<ap-ip>:81/update)");
    addLog("[BOOT] Pyro test bench ready — DISARMED");
}

void loop() {
    otaServer.handleClient();
    ElegantOTA.loop();
    server.handleClient();

    pulseTick();

    if (armed && (millis() - armedAtMs > ARMED_TIMEOUT_S * 1000UL)) {
        doDisarm("auto-disarm timeout");
    }

    esp_task_wdt_reset();
    delay(2);
}

#ifndef GUIDANCE_WEB_SERVER_H
#define GUIDANCE_WEB_SERVER_H

#include "globals.h"
#include "instruments.h"
#include "buzzers.h"
#include <WiFi.h>
#include <WebServer.h>
#include <stdlib.h>
#include <SD_MMC.h>
#include <FS.h>
#include <esp_task_wdt.h>
#include "index.html"
#include <ElegantOTA.h>

WebServer* serverPtr = nullptr;
TaskHandle_t wifiServerTaskHandle = NULL;
SemaphoreHandle_t webServerMutex = NULL;

extern volatile bool servoOverrideActive;
extern volatile float servoOverrideAngles[8];
extern char serialLogBuffer[];
extern volatile uint16_t serialLogHead;
extern volatile uint16_t serialLogTail;
extern char logCache[];
extern volatile size_t logCacheHead;
extern volatile size_t logCacheTail;
extern volatile bool logCacheOverflow;
extern SemaphoreHandle_t logCacheMutex;

extern std::atomic<unsigned long long> systemBaseEpochMs;
extern std::atomic<unsigned long> systemBaseMillis;

// ============================================================================
// READY-PHASE MINIMAL HTML PAGE
// ============================================================================

const char READY_INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ISAAC-L READY</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body { background: #0d0f12; color: #f5f6f7; font-family: monospace; padding: 20px; text-align: center; }
        .badge { display: inline-block; padding: 10px 24px; border-radius: 30px; font-size: 2rem; font-weight: bold; animation: pulse 1s infinite; margin-bottom: 16px; }
        @keyframes pulse { 0%,100% { opacity:1; } 50% { opacity:0.5; } }
        .ready-badge { background: #ff7700; color: #fff; }
        .grid { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 8px; max-width: 480px; margin: 0 auto 16px; }
        .card { background: #161a22; border: 1px solid #232d38; border-radius: 8px; padding: 10px; }
        .label { color: #8a99ad; font-size: 0.7rem; text-transform: uppercase; }
        .val { font-size: 1.3rem; font-weight: bold; font-variant-numeric: tabular-nums; }
        .unit { color: #8a99ad; font-size: 0.75rem; }
        .abort-btn { background: #00ff66; color: #000; font-size: 1.5rem; font-weight: bold; padding: 20px 60px; border: none; border-radius: 12px; cursor: pointer; margin: 12px; min-width: 200px; }
        .launch-btn { background: #ff0055; color: #fff; font-size: 1rem; font-weight: bold; padding: 14px 40px; border: none; border-radius: 10px; cursor: pointer; margin: 6px; }
        .abort-btn:active, .launch-btn:active { transform: scale(0.97); }
        .log-box { background: #0a0c10; border: 1px solid #232d38; border-radius: 8px; padding: 8px; font-size: 0.65rem; height: 150px; overflow-y: auto; white-space: pre-wrap; text-align: left; color: #7aa2cc; max-width: 480px; margin: 0 auto; }
    </style>
</head>
<body>
    <div class="badge ready-badge">READY</div>
    <div class="grid">
        <div class="card"><div class="label">Alt (m)</div><div class="val" id="alt">0.0</div></div>
        <div class="card"><div class="label">Vel Z (m/s)</div><div class="val" id="vz">0.0</div></div>
        <div class="card"><div class="label">Alt Raw (m)</div><div class="val" id="altr">0.0</div></div>
        <div class="card"><div class="label">Ax (m/s2)</div><div class="val" id="ax">0.0</div></div>
        <div class="card"><div class="label">Ay (m/s2)</div><div class="val" id="ay">0.0</div></div>
        <div class="card"><div class="label">Az (m/s2)</div><div class="val" id="az">0.0</div></div>
    </div>
        <div style="max-width:480px;margin:12px auto 0;">
            <div class="card" style="text-align:center;"><div class="label">Instruments</div><div class="val" id="instr_ready">UNKNOWN</div></div>
        </div>
    <button class="abort-btn" onclick="doAbort()">ABORT</button>
    <br>
    <button class="launch-btn" onclick="doLaunch()">LAUNCH</button>
    <div class="log-box" id="slog"></div>
<script>
    function update() {
        fetch('/ready_data').then(r=>r.json()).then(d=>{
            document.getElementById('alt').innerText=d.alt_filt.toFixed(1);
            document.getElementById('altr').innerText=d.alt_raw.toFixed(1);
            document.getElementById('vz').innerText=d.vel_z.toFixed(1);
            document.getElementById('ax').innerText=d.ax.toFixed(2);
            document.getElementById('ay').innerText=d.ay.toFixed(2);
            document.getElementById('az').innerText=d.az.toFixed(2);
            if(d.phase!==2) location.reload();
                // instruments
                var instrEl = document.getElementById('instr_ready');
                var missing = [];
                if(!d.imu_ok) missing.push('IMU');
                if(!d.baro_ok) missing.push('BARO');
                if(!d.gps_ok) missing.push('GPS');
                if(!d.sd_ready) missing.push('SD');
                if(missing.length===0){ instrEl.innerText='ALL OK'; instrEl.style.color='#00ff66'; }
                else { instrEl.innerText='Missing: '+missing.join(', '); instrEl.style.color='#ff0055'; }
        }).catch(()=>{});
    }
    function fetchLog(){fetch('/serial_log').then(r=>r.text()).then(t=>{if(!t)return;var b=document.getElementById('slog');b.textContent+=t;b.scrollTop=b.scrollHeight;}).catch(()=>{});}
    function doAbort(){fetch('/abort',{method:'POST'});}
    function doLaunch(){fetch('/launch',{method:'POST'});}
    setInterval(update,200);
    setInterval(fetchLog,500);
    update();
    // send browser time once to seed wall-clock if GPS not available
    try { fetch('/set_time?ts=' + Date.now()); } catch(e) {}
</script>
</body>
</html>
)rawliteral";

// ============================================================================
// READY-PHASE MINIMAL TELEMETRY ENDPOINT
// ============================================================================

void handleReadyData() {
    char json[320];
    int len = snprintf(json, sizeof(json),
        "{\"phase\":%u,\"armed\":%s,"
        "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
        "\"alt_filt\":%.2f,\"alt_raw\":%.2f,\"vel_z\":%.2f,"
        "\"imu_ok\":%s,\"baro_ok\":%s,\"gps_ok\":%s,\"sd_ready\":%s}",
        static_cast<uint8_t>(currentPhase.load(std::memory_order_relaxed)),
        systemArmed.load(std::memory_order_relaxed) ? "true" : "false",
        latestAx, latestAy, latestAz,
        filter_alt, latestBaroPressure > 0.0f ? filter_alt : 0.0f,
        V_z,
        bnoInitialized ? "true" : "false",
        bmpInitialized ? "true" : "false",
        sharedTelemetry.gpsUpdated ? "true" : "false",
        sdReady ? "true" : "false"
    );

    if (len < 0 || len >= (int)sizeof(json)) {
        serverPtr->send(500, "application/json", "{\"status\":\"error\"}");
        return;
    }
    serverPtr->send(200, "application/json", json);
}

// ============================================================================
// ABORT — READY -> PAD
// ============================================================================

void handleAbort() {
    FlightPhase phase = currentPhase.load(std::memory_order_relaxed);
    if (phase != READY) {
        serverPtr->send(403, "text/plain", "CAN ONLY ABORT FROM READY PHASE");
        return;
    }

    if (xSemaphoreTake(webServerMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        currentPhase.store(PAD, std::memory_order_relaxed);
        systemArmed.store(false, std::memory_order_relaxed);
        servoOverrideActive = false;
        xSemaphoreGive(webServerMutex);
    }

    playTone(1760, 150); delay(180);
    playTone(880, 300); delay(350);
    playTone(440, 500); delay(550);

    write(LOG_BOTH, LOG_WARN, "[ABORT] READY -> PAD abort triggered by operator");

    serverPtr->send(200, "text/plain", "ABORTED TO PAD");
}

// ============================================================================
// FULL-PAD ROUTE HANDLERS
// ============================================================================

// root handler already defined earlier

void handleGetData() {
    float r, p, y, ra, fa, vz;
    double lat, lng;
    bool gps_u, armed_state, sensors_ready;
    uint8_t ph, sys_m;

    if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        serverPtr->send(503, "application/json", "{\"status\":\"locked\"}");
        return;
    }

    r = sharedTelemetry.roll;
    p = sharedTelemetry.pitch;
    y = sharedTelemetry.yaw;
    ra = sharedTelemetry.raw_altitude;
    fa = sharedTelemetry.filtered_altitude;
    vz = sharedTelemetry.velocity_z;
    lat = sharedTelemetry.latitude;
    lng = sharedTelemetry.longitude;
    gps_u = sharedTelemetry.gpsUpdated;
    ph = sharedTelemetry.phase;
    sys_m = sharedTelemetry.system_mode;
    armed_state = sharedTelemetry.armed;
    sensors_ready = sharedTelemetry.sensors_ok;
    xSemaphoreGive(telemetryMutex);

    char json[1400];
    int len = snprintf(json, sizeof(json),
      "{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,"
       "\"raw_alt\":%.2f,\"filtered_alt\":%.2f,\"vel_z\":%.2f,"
       "\"lat\":%.6f,\"lng\":%.6f,\"gps_updated\":%s,"
       "\"phase\":%u,\"system_mode\":%u,\"armed\":%s,\"sensors_ok\":%s,"
       "\"airspeed\":%.2f,\"kalman_P\":%.4f,\"baro_alpha\":%.3f,"
       "\"pid0\":%.3f,\"pid1\":%.3f,\"pid2\":%.3f,\"pid3\":%.3f,"
       "\"pid4\":%.3f,\"pid5\":%.3f,\"pid6\":%.3f,\"pid7\":%.3f,"
       "\"gain_kp\":%.4f,\"gain_ki\":%.4f,\"gain_kd\":%.4f,"
       "\"baro_pressure\":%.2f,\"baro_temp\":%.2f,"
       "\"qx\":%.4f,\"qy\":%.4f,\"qz\":%.4f,\"qw\":%.4f,"
       "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
       "\"dt\":%.6f,\"log_drops\":%u,"
       "\"imu_ok\":%s,\"baro_ok\":%s,\"gps_ok\":%s,\"sd_ready\":%s,"
       "\"servo0\":%.1f,\"servo1\":%.1f,\"servo2\":%.1f,\"servo3\":%.1f,"
       "\"servo4\":%.1f,\"servo5\":%.1f,\"servo6\":%.1f,\"servo7\":%.1f,"
       "\"servo_override\":%s}",
      r, p, y,
      ra, fa, vz,
      lat, lng,
      gps_u ? "true" : "false",
      ph, sys_m,
      armed_state ? "true" : "false",
      sensors_ready ? "true" : "false",
      latestAirspeed, latestKalmanP, latestBaroAlpha,
      latestPIDOutputs[0], latestPIDOutputs[1], latestPIDOutputs[2], latestPIDOutputs[3],
      latestPIDOutputs[4], latestPIDOutputs[5], latestPIDOutputs[6], latestPIDOutputs[7],
      latestActiveGains[0], latestActiveGains[1], latestActiveGains[2],
      latestBaroPressure, latestBaroTemp,
      latestQx, latestQy, latestQz, latestQw,
      latestAx, latestAy, latestAz,
    latestDt, (uint32_t)logDropCount,
    bnoInitialized ? "true" : "false",
    bmpInitialized ? "true" : "false",
    gps_u ? "true" : "false",
    sdReady ? "true" : "false",
    latestServoAngles[0], latestServoAngles[1], latestServoAngles[2], latestServoAngles[3],
      latestServoAngles[4], latestServoAngles[5], latestServoAngles[6], latestServoAngles[7],
      servoOverrideActive ? "true" : "false"
    );

    if (len < 0 || len >= (int)sizeof(json)) {
        serverPtr->send(500, "application/json", "{\"status\":\"error\"}");
        return;
    }

    serverPtr->send(200, "application/json", json);
}

void handleArm() {
    if (xSemaphoreTake(webServerMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (currentSystemMode.load(std::memory_order_relaxed) == MODE_ACTIVE_PAD) {
            systemArmed.store(true, std::memory_order_relaxed);
            xSemaphoreGive(webServerMutex);
            playTone(880, 150); delay(180);
            playTone(880, 150); delay(180);
            playTone(1760, 400); delay(450);
            serverPtr->send(200, "text/plain", "ARMED");
        } else {
            xSemaphoreGive(webServerMutex);
            serverPtr->send(403, "text/plain", "CANNOT ARM IN TRANSPORT MODE");
        }
    } else {
        serverPtr->send(503, "text/plain", "SYSTEM BUSY");
    }
}

void handleDisarm() {
    if (xSemaphoreTake(webServerMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        systemArmed.store(false, std::memory_order_relaxed);
        servoOverrideActive = false;
        xSemaphoreGive(webServerMutex);
    }

    playTone(1760, 150); delay(180);
    playTone(880, 300); delay(350);

    serverPtr->send(200, "text/plain", "DISARMED");
}

void handleRoot() {
    serverPtr->send_P(200, "text/html", INDEX_HTML);
}

void handleSetSystemMode() {
    if (!serverPtr->hasArg("val")) {
        serverPtr->send(400, "text/plain", "Missing val parameter");
        return;
    }

    // Prevent mode changes during flight
    FlightPhase phaseNow = currentPhase.load(std::memory_order_relaxed);
    if (phaseNow == BOOST || phaseNow == COAST || phaseNow == DESCENT) {
        serverPtr->send(403, "text/plain", "CANNOT CHANGE MODE DURING FLIGHT");
        return;
    }

    String modeVal = serverPtr->arg("val");
    if (modeVal == "transport") {
        if (xSemaphoreTake(webServerMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            currentSystemMode.store(MODE_TRANSPORT, std::memory_order_relaxed);
            currentPhase.store(TRANSPORT, std::memory_order_relaxed);
            systemArmed.store(false, std::memory_order_relaxed);
            servoOverrideActive = false;
            xSemaphoreGive(webServerMutex);
        }
        playTone(587, 80); delay(100);
        playTone(880, 80); delay(100);
        playTone(1175, 150); delay(180);
        serverPtr->send(200, "text/plain", "MODE: TRANSPORT");
    } else if (modeVal == "pad") {
        // Initialize instruments if not already done
        if (!bnoInitialized || !bmpInitialized) {
            if (!initInstruments()) {
                playTone(200, 500);
                serverPtr->send(500, "text/plain", "INSTRUMENT INITIALIZATION FAILED");
                return;
            }
            calibrateGroundAltitude();
        }
        if (xSemaphoreTake(webServerMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            currentSystemMode.store(MODE_PAD, std::memory_order_relaxed);
            currentPhase.store(PAD, std::memory_order_relaxed);
            systemArmed.store(false, std::memory_order_relaxed);
            servoOverrideActive = false;
            xSemaphoreGive(webServerMutex);
        }
        playTone(587, 80); delay(100);
        playTone(880, 80); delay(100);
        playTone(1175, 150); delay(180);
        serverPtr->send(200, "text/plain", "MODE: PAD");
    } else if (modeVal == "active_pad") {
        if (xSemaphoreTake(webServerMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            currentSystemMode.store(MODE_ACTIVE_PAD, std::memory_order_relaxed);
            currentPhase.store(READY, std::memory_order_relaxed);
            xSemaphoreGive(webServerMutex);
        }
        playTone(587, 80); delay(100);
        playTone(880, 80); delay(100);
        playTone(1175, 150); delay(180);
        serverPtr->send(200, "text/plain", "MODE: ACTIVE_PAD");
    } else {
        serverPtr->send(400, "text/plain", "Unknown mode");
    }
}

// Allow a connected client to set the system wall-clock time (ms since epoch)
void handleSetTime() {
    if (!serverPtr->hasArg("ts")) {
        serverPtr->send(400, "text/plain", "Missing ts parameter");
        return;
    }
    String ts = serverPtr->arg("ts");
    unsigned long long val = (unsigned long long) atoll(ts.c_str());
    if (val == 0) {
        serverPtr->send(400, "text/plain", "Invalid ts parameter");
        return;
    }
    // set base epoch and base millis
    if (xSemaphoreTake(webServerMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        systemBaseEpochMs.store(val, std::memory_order_relaxed);
        systemBaseMillis.store(millis(), std::memory_order_relaxed);
        xSemaphoreGive(webServerMutex);
    }
    serverPtr->send(200, "text/plain", "OK");
}

void handleSetServo() {
    if (!serverPtr->hasArg("ch") || !serverPtr->hasArg("angle")) {
        serverPtr->send(400, "text/plain", "Missing ch or angle parameter");
        return;
    }
    int ch = serverPtr->arg("ch").toInt();
    float angle = serverPtr->arg("angle").toFloat();

    if (ch < 0 || ch > 7) {
        serverPtr->send(400, "text/plain", "Channel must be 0-7");
        return;
    }
    if (angle < 60.0f || angle > 120.0f) {
        serverPtr->send(400, "text/plain", "Angle must be 60-120 deg");
        return;
    }

    FlightPhase phase = currentPhase.load(std::memory_order_relaxed);
    if (phase != PAD && phase != TRANSPORT) {
        serverPtr->send(403, "text/plain", "CANNOT OVERRIDE SERVOS IN THIS PHASE");
        return;
    }

    servoOverrideActive = true;
    servoOverrideAngles[ch] = angle;

    char resp[64];
    snprintf(resp, sizeof(resp), "OK: Servo %d -> %.1f deg", ch, angle);
    serverPtr->send(200, "text/plain", resp);
}

void handleSetAllServos() {
    if (!serverPtr->hasArg("angle")) {
        serverPtr->send(400, "text/plain", "Missing angle parameter");
        return;
    }
    float angle = serverPtr->arg("angle").toFloat();

    if (angle < 60.0f || angle > 120.0f) {
        serverPtr->send(400, "text/plain", "Angle must be 60-120 deg");
        return;
    }

    FlightPhase phase = currentPhase.load(std::memory_order_relaxed);
    if (phase != PAD && phase != TRANSPORT) {
        serverPtr->send(403, "text/plain", "CANNOT OVERRIDE SERVOS IN THIS PHASE");
        return;
    }

    servoOverrideActive = true;
    for (int i = 0; i < 8; i++) {
        servoOverrideAngles[i] = angle;
    }

    char resp[64];
    snprintf(resp, sizeof(resp), "OK: All servos -> %.1f deg", angle);
    serverPtr->send(200, "text/plain", resp);
}

void handleReleaseServos() {
    servoOverrideActive = false;
    for (int i = 0; i < 8; i++) {
        servoOverrideAngles[i] = 90.0f;
    }
    serverPtr->send(200, "text/plain", "Servo override released - PID control active");
}

void handleServoTest() {
    FlightPhase phase = currentPhase.load(std::memory_order_relaxed);
    if (phase != PAD && phase != TRANSPORT) {
        serverPtr->send(403, "text/plain", "CANNOT TEST SERVOS IN THIS PHASE");
        return;
    }

    int ch = -1;
    if (serverPtr->hasArg("ch")) {
        ch = serverPtr->arg("ch").toInt();
        if (ch < 0 || ch > 7) {
            serverPtr->send(400, "text/plain", "Channel must be 0-7 or omit for all");
            return;
        }
    }

    servoOverrideActive = true;

    int startCh = (ch >= 0) ? ch : 0;
    int endCh   = (ch >= 0) ? ch : 7;

    for (int c = startCh; c <= endCh; c++) {
        servoOverrideAngles[c] = 90.0f;
        delay(200);

        servoOverrideAngles[c] = 120.0f;
        delay(300);

        servoOverrideAngles[c] = 60.0f;
        delay(300);

        servoOverrideAngles[c] = 90.0f;
        delay(200);
    }

    serverPtr->send(200, "text/plain", (ch >= 0) ? "Single servo test complete" : "All servo test complete");
}

void handleSerialLog() {
    String out;
    out.reserve(4096);

    // First, serve from the pre-SD-init log cache
    if (logCacheMutex && xSemaphoreTake(logCacheMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (logCacheHead != logCacheTail) {
            if (logCacheHead > logCacheTail) {
                out.concat(&logCache[logCacheTail], logCacheHead - logCacheTail);
            } else {
                out.concat(&logCache[logCacheTail], LOG_CACHE_SIZE - logCacheTail);
                out.concat(&logCache[0], logCacheHead);
            }
            logCacheTail = logCacheHead;
        }
        xSemaphoreGive(logCacheMutex);
    }

    // Then serve from the serial log buffer
    uint16_t h = serialLogHead;
    uint16_t t = serialLogTail;

    if (h >= t) {
        for (uint16_t i = t; i < h; i++) {
            out += serialLogBuffer[i];
        }
    } else {
        for (uint16_t i = t; i < SERIAL_LOG_BUF_SIZE; i++) {
            out += serialLogBuffer[i];
        }
        for (uint16_t i = 0; i < h; i++) {
            out += serialLogBuffer[i];
        }
    }

    serialLogTail = h;

    serverPtr->send(200, "text/plain", out);
}

void handleDownloadLog() {
    if (!SD_MMC.exists(LOG_FILE_PATH)) {
        serverPtr->send(404, "text/plain", "No flight log found.");
        return;
    }
    File file = SD_MMC.open(LOG_FILE_PATH, FILE_READ);
    if (!file) {
        serverPtr->send(500, "text/plain", "Failed to open log file.");
        return;
    }
    serverPtr->streamFile(file, "text/csv");
    file.close();
}

void handleDeleteLog() {
    if (SD_MMC.exists("/flight_log.csv")) {
        SD_MMC.remove("/flight_log.csv");
        File file = SD_MMC.open("/flight_log.csv", FILE_WRITE);
        if (file) {
            file.println("timestamp_ms,roll,pitch,yaw,raw_alt,filtered_alt,vel_z,"
                         "acc_x,acc_y,acc_z,phase,"
                         "servo0,servo1,servo2,servo3,servo4,servo5,servo6,servo7,"
                         "pid0,pid1,pid2,pid3,pid4,pid5,pid6,pid7,"
                         "gain_kp,gain_ki,gain_kd,"
                         "airspeed,kalman_P,baro_alpha,"
                         "qx,qy,qz,qw,"
                         "baro_pressure,baro_temp,dt,dropped");
            file.close();
        }
        serverPtr->send(200, "text/plain", "LOGS ERASED");
    } else {
        serverPtr->send(404, "text/plain", "File not found");
    }
}

void handleConfirmRecovery() {
    if (xSemaphoreTake(webServerMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        FlightPhase phase = currentPhase.load(std::memory_order_relaxed);
        if (phase == RECOVERY) {
            currentPhase.store(TRANSPORT, std::memory_order_relaxed);
            currentSystemMode.store(MODE_TRANSPORT, std::memory_order_relaxed);
            systemArmed.store(false, std::memory_order_relaxed);
            servoOverrideActive = false;
            recoveryBeaconStop();
            xSemaphoreGive(webServerMutex);
            playTone(587, 80); delay(100);
            playTone(880, 80); delay(100);
            playTone(1175, 150); delay(180);
            serverPtr->send(200, "text/plain", "RECOVERY CONFIRMED -> TRANSPORT");
            write(LOG_BOTH, LOG_INFO, "[RECOVERY] Operator confirmed recovery -> TRANSPORT");
            return;
        }
        xSemaphoreGive(webServerMutex);
    }
    serverPtr->send(403, "text/plain", "NOT IN RECOVERY PHASE");
}

// ============================================================================
// DYNAMIC ROUTE REGISTRATION
// ============================================================================
// DYNAMIC ROUTE REGISTRATION
// ============================================================================

void handleLaunch() {
    if (xSemaphoreTake(webServerMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        SystemMode mode = currentSystemMode.load(std::memory_order_relaxed);
        FlightPhase phase = currentPhase.load(std::memory_order_relaxed);
        if (mode == MODE_ACTIVE_PAD) {
            // Already in ACTIVE_PAD (READY phase)
            systemArmed.store(true, std::memory_order_relaxed);
            servoOverrideActive = false;
            xSemaphoreGive(webServerMutex);

            playTone(880, 150); delay(180);
            playTone(1760, 400); delay(450);

            serverPtr->send(200, "text/plain", "ALREADY IN ACTIVE_PAD (READY)");
            return;
        }
        xSemaphoreGive(webServerMutex);
    }
    serverPtr->send(403, "text/plain", "CANNOT LAUNCH: MUST BE IN ACTIVE_PAD MODE");
}


void registerFullRoutes() {
    serverPtr->on("/", HTTP_GET, handleRoot);
    serverPtr->on("/data", HTTP_GET, handleGetData);
    serverPtr->on("/set_time", HTTP_GET, handleSetTime);
    serverPtr->on("/arm", HTTP_POST, handleArm);
    serverPtr->on("/disarm", HTTP_POST, handleDisarm);
    serverPtr->on("/launch", HTTP_POST, handleLaunch);
    serverPtr->on("/set_mode", HTTP_POST, handleSetSystemMode);
    serverPtr->on("/confirm_recovery", HTTP_POST, handleConfirmRecovery);
    serverPtr->on("/servo", HTTP_POST, handleSetServo);
    serverPtr->on("/servo_all", HTTP_POST, handleSetAllServos);
    serverPtr->on("/servo_release", HTTP_POST, handleReleaseServos);
    serverPtr->on("/servo_test", HTTP_POST, handleServoTest);
    serverPtr->on("/serial_log", HTTP_GET, handleSerialLog);
    serverPtr->on("/download_log", HTTP_GET, handleDownloadLog);
    serverPtr->on("/delete_log", HTTP_POST, handleDeleteLog);
    serverPtr->on("/confirm_recovery", HTTP_POST, handleConfirmRecovery);
    ElegantOTA.begin(serverPtr);
    Serial.println("[WIFI] Full dashboard routes registered");
}

void registerReadyRoutes() {
    serverPtr->on("/", HTTP_GET, []() { serverPtr->send_P(200, "text/html", READY_INDEX_HTML); });
    serverPtr->on("/ready_data", HTTP_GET, handleReadyData);
    serverPtr->on("/set_time", HTTP_GET, handleSetTime);
    serverPtr->on("/abort", HTTP_POST, handleAbort);
    serverPtr->on("/launch", HTTP_POST, handleLaunch);
    serverPtr->on("/serial_log", HTTP_GET, handleSerialLog);
    Serial.println("[WIFI] Ready-phase minimal routes registered");
}

// ============================================================================
// WIFI SERVER TASK — PHASE-AWARE ROUTE SWAPPING
// ============================================================================

void WifiServerTask(void *pvParameters) {
    (void) pvParameters;
    esp_task_wdt_add(NULL);

    WiFi.softAP(AP_SSID, AP_PASSWORD);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("[WIFI] Access Point Live! SSID: ");
    Serial.println(AP_SSID);
    Serial.print("[WIFI] GCS Dashboard IP: ");
    Serial.println(IP);

    serverPtr = new WebServer(80);
    registerFullRoutes();
    serverPtr->begin();

    FlightPhase prevPhase = currentPhase.load(std::memory_order_relaxed);
    uint8_t readyTick = 0;

    for (;;) {
        if (!wifiActive.load(std::memory_order_relaxed)) {
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        FlightPhase curPhase = currentPhase.load(std::memory_order_relaxed);

        if (curPhase == BOOST) {
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (curPhase != prevPhase) {
            if (prevPhase == PAD && curPhase == READY) {
                Serial.println("[WIFI] PAD -> READY: switching to minimal server");
                delete serverPtr;
                serverPtr = new WebServer(80);
                registerReadyRoutes();
                serverPtr->begin();
            } else if (prevPhase == READY && curPhase == PAD) {
                Serial.println("[WIFI] READY -> PAD (abort): restoring full dashboard");
                delete serverPtr;
                serverPtr = new WebServer(80);
                registerFullRoutes();
                serverPtr->begin();
            } else if (curPhase == RECOVERY) {
                // Re-enable WiFi and restore full dashboard for recovery confirmation
                Serial.println("[WIFI] -> RECOVERY: re-enabling AP and full dashboard");
                if (!wifiActive.load(std::memory_order_relaxed)) {
                    wifiActive.store(true, std::memory_order_relaxed);
                    WiFi.softAP(AP_SSID, AP_PASSWORD);
                    IPAddress IP = WiFi.softAPIP();
                    Serial.print("[WIFI] Recovery AP Live! IP: ");
                    Serial.println(IP);
                }
                delete serverPtr;
                serverPtr = new WebServer(80);
                registerFullRoutes();
                serverPtr->begin();
            }
            prevPhase = curPhase;
        }

        if (curPhase == READY) {
            readyTick++;
            if (readyTick >= 3) {
                if (serverPtr) serverPtr->handleClient();
                readyTick = 0;
            }
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(50));
        } else {
            if (serverPtr) {
                serverPtr->handleClient();
                ElegantOTA.loop();
            } else {
                // server pointer temporarily unavailable; yield safely
                esp_task_wdt_reset();
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(15));
        }
    }
}

void shutdownWiFiNetwork() {
    if (!wifiActive.load(std::memory_order_relaxed)) return;
    wifiActive.store(false, std::memory_order_relaxed);

    Serial.println("\n=======================================================");
    Serial.println("LAUNCH PHASE INIT! SHUTTING DOWN WIFI");
    Serial.println("=======================================================");

    playTone(330, 400); delay(450);
    playTone(220, 600);

    // synchronize server pointer cleanup to avoid races with WifiServerTask
    if (xSemaphoreTake(webServerMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        if (serverPtr) {
            serverPtr->stop();
            delete serverPtr;
            serverPtr = nullptr;
        }
        xSemaphoreGive(webServerMutex);
    } else {
        // fallback: try best-effort cleanup
        if (serverPtr) {
            serverPtr->stop();
            delete serverPtr;
            serverPtr = nullptr;
        }
    }

    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    wifiGood = 2;

    Serial.println("[SYSTEM] WiFi Transceiver powered down to safe flight state.");

    if (wifiServerTaskHandle != NULL) {
        wifiServerTaskHandle = NULL;
    }
}

#endif // GUIDANCE_WEB_SERVER_H
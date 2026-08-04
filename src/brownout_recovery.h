#ifndef BROWNOUT_RECOVERY_H
#define BROWNOUT_RECOVERY_H

// ============================================================================
// BROWNOUT / PANIC-RESET RECOVERY
// ============================================================================
// Three-tier persistence:
//   1) RTC slow memory (RTC_DATA_ATTR): updated every loop, zero wear.
//      Survives brownout & panic reset (NOT full power-cycle).
//   2) NVS / Preferences (flash): written on phase transitions + pyro events.
//      Survives full power loss.
//   3) SD card (file): same triggers as NVS, independent medium.
//
// At boot, if enableBrownoutRecovery is true AND a recent (<60s) valid
// snapshot is found that is marked in-flight, the controller auto-resumes
// the cached phase instead of booting into PAD/TRANSPORT.

#include "globals.h"
#include "Launchsequence.h"
#include "guidance_flight_control.h"   // pid[] + MAX_DEFLECTION_DEG for clamp
#include <Preferences.h>
#include <SD_MMC.h>
#include <esp_system.h>
#include <rom/rtc.h>

// ---- config flags (defined in main.cpp / globals.h) ----
// enableBrownoutRecovery, enableDualWriteCache, enableBackupChute are
// static constexpr in globals.h — read-only. Do not write to them.

// ---- diagnostics ----
extern uint32_t bootCount;
extern bool    flightCacheValid;
extern FlightCache flightCacheRestored;

// ---- RTC-mirrored copy (zero wear, updated per loop) ----
RTC_DATA_ATTR FlightCache rtcCache;
RTC_NOINIT_ATTR uint32_t rtcBootCount;        // survives soft reboot only
RTC_NOINIT_ATTR uint32_t rtcLastResetReason;  // last reset reason (NOINIT survives reboot only)

// ---- NVS handle, lazy-initialized ----
static Preferences brownoutPrefs;
static bool brownoutPrefsOpen = false;

static const char *BROWNOUT_NVS_NAMESPACE = "flight";
static const char *BROWNOUT_NVS_KEY       = "cache";
static const char *BROWNOUT_SD_PATH        = "/flight_state.bin";

// QuickPID integrators are public iTerm members; we access them by name
// (declared in guidance_flight_control.h):
//   extern QuickPID pid[8];
// We declare them again with a slightly weaker extern here so this header is
// self-contained if guidance_flight_control.h hasn't been included yet.
#ifndef BROWNOUT_PID_DECLARED
#define BROWNOUT_PID_DECLARED
extern QuickPID pid[8];
#endif

// ============================================================================
// Internal helpers
// ============================================================================

static FlightCache buildSnapshot() {
    FlightCache c;
    memset(&c, 0, sizeof(c));

    c.magic              = BROWNOUT_CACHE_MAGIC;
    c.version            = BROWNOUT_CACHE_VERSION;
    // Use the system base epoch (if available) for snapshot time; fall back to 0
    extern std::atomic<unsigned long long> systemBaseEpochMs;
    extern std::atomic<unsigned long>       systemBaseMillis;
    unsigned long long baseEp = systemBaseEpochMs.load(std::memory_order_relaxed);
    unsigned long      baseMs = systemBaseMillis.load(std::memory_order_relaxed);
    c.snapshotEpoch_ms  = (baseEp > 0) ? (baseEp + (millis() - baseMs)) : 0;

    c.systemMode  = (uint8_t)currentSystemMode.load(std::memory_order_relaxed);
    c.flightPhase = (uint8_t)currentPhase.load(std::memory_order_relaxed);
    c.armed       = systemArmed.load(std::memory_order_relaxed) ? 1 : 0;

    c.liftoffEpoch_ms = 0;
    c.liftoffAgeMs   = liftoff_time_ms.load(std::memory_order_relaxed);
    // Estimate liftoff epoch similarly if base epoch is known:
    // epoch_at_liftoff = baseEp + (liftoff_time_ms - baseMs)
    if (baseEp > 0) {
        unsigned long liftoffMs = c.liftoffAgeMs;
        c.liftoffEpoch_ms = (liftoffMs > 0) ? (baseEp + (liftoffMs - baseMs)) : 0;
    }

    c.filter_alt       = filter_alt;
    c.V_z              = V_z;
    c.max_altitude     = ::max_altitude;
    c.baseline_altitude = baseline_altitude;
    c.qnh_pressure      = qnh_pressure;

    extern float latestServoAngles[8];
    memcpy(c.servoAngles, latestServoAngles, sizeof(c.servoAngles));

    for (int i = 0; i < 3; i++) {
        c.pyroStateArr[i]  = (uint8_t)pyroState[i].state;
        c.pyroFiredAtMs[i] = pyroState[i].firedAtMs;
        c.pyroAttempted[i] = pyroState[i].attempted ? 1 : 0;
    }
    c.backupEnabled = enableBackupChute ? 1 : 0;

    // PID integrators — read but only persist on phase change (caller decides).
    // We store outputSum (the public integral accumulator) per PID controller.
    for (size_t i = 0; i < BROWNOUT_NUM_PID; i++) {
        c.pid_iTerm[i] = pid[i].GetOutputSum();
    }

    c.bootCount       = bootCount;
    c.lastResetReason = rtcLastResetReason;
    return c;
}

static bool saveToNVS(const FlightCache &c) {
    if (!brownoutPrefsOpen) {
        if (!brownoutPrefs.begin(BROWNOUT_NVS_NAMESPACE, false)) {
            return false;
        }
        brownoutPrefsOpen = true;
    }
    size_t w = brownoutPrefs.putBytes(BROWNOUT_NVS_KEY, &c, sizeof(c));
    return w == sizeof(c);
}

static bool loadFromNVS(FlightCache &out) {
    if (!brownoutPrefsOpen) {
        if (!brownoutPrefs.begin(BROWNOUT_NVS_NAMESPACE, true)) return false;
        brownoutPrefsOpen = true;
    }
    size_t avail = brownoutPrefs.getBytesLength(BROWNOUT_NVS_KEY);
    if (avail != sizeof(FlightCache)) return false;
    size_t got = brownoutPrefs.getBytes(BROWNOUT_NVS_KEY, &out, sizeof(out));
    return got == sizeof(out) && out.magic == BROWNOUT_CACHE_MAGIC
           && out.version == BROWNOUT_CACHE_VERSION;
}

static bool saveToSD(const FlightCache &c) {
    if (!sdReady) return false;
    File f = SD_MMC.open(BROWNOUT_SD_PATH, FILE_WRITE);
    if (!f) return false;
    size_t w = f.write((const uint8_t *)&c, sizeof(c));
    f.close();
    return w == sizeof(c);
}

static bool loadFromSD(FlightCache &out) {
    if (!sdReady) return false;
    if (!SD_MMC.exists(BROWNOUT_SD_PATH)) return false;
    File f = SD_MMC.open(BROWNOUT_SD_PATH, FILE_READ);
    if (!f) return false;
    size_t got = f.read((uint8_t *)&out, sizeof(out));
    f.close();
    return got == sizeof(out) && out.magic == BROWNOUT_CACHE_MAGIC
           && out.version == BROWNOUT_CACHE_VERSION;
}

// ============================================================================
// Public API
// ============================================================================

void flightCacheSnapshotRTC() {
    FlightCache c = buildSnapshot();
    rtcCache = c;
}

void flightCacheSave(bool writeFlash) {
    FlightCache c = buildSnapshot();
    rtcCache = c;
    if (!writeFlash) return;
    if (!saveToNVS(c)) {
        write(LOG_BOTH, LOG_WARN, "[BROWNOUT] NVS save failed");
    }
    if (!saveToSD(c)) {
        write(LOG_BOTH, LOG_WARN, "[BROWNOUT] SD snapshot save failed (may be detached)");
    }
}

void flightCacheInvalidate() {
    // Mark both stores as invalid by zeroing the magic.
    rtcCache.magic = 0;
    if (brownoutPrefsOpen || brownoutPrefs.begin(BROWNOUT_NVS_NAMESPACE, false)) {
        brownoutPrefsOpen = true;
        brownoutPrefs.remove(BROWNOUT_NVS_KEY);
    }
    if (sdReady && SD_MMC.exists(BROWNOUT_SD_PATH)) {
        SD_MMC.remove(BROWNOUT_SD_PATH);
    }
    write(LOG_BOTH, LOG_INFO, "[BROWNOUT] Cache invalidated");
}

void flightCacheRestorePyros(const FlightCache &c) {
    for (int i = 0; i < 3; i++) {
        pyroState[i].state      = (PyroState)c.pyroStateArr[i];
        pyroState[i].firedAtMs  = c.pyroFiredAtMs[i];
        pyroState[i].attempted  = c.pyroAttempted[i] != 0;
    }
}

void flightCacheRestorePID(const FlightCache &c) {
    // QuickPID exposes SetOutputSum() which corresponds to the integral
    // accumulator (outputSum is public + has a setter). pTerm/iTerm/dTerm
    // themselves are private and Reset() on next Compute() anyway, so we
    // restore via the public path — and clamp the seeded accumulators to the
    // control output range so a corrupt/stale sum can't wind the servos
    // straight to the hard stop on the first loop after a mid-flight resume.
    extern float latestServoAngles[8];
    memcpy(latestServoAngles, c.servoAngles, sizeof(c.servoAngles));

    for (size_t i = 0; i < BROWNOUT_NUM_PID; i++) {
        float clamped = constrain(c.pid_iTerm[i], -MAX_DEFLECTION_DEG, MAX_DEFLECTION_DEG);
        if (fabsf(clamped - c.pid_iTerm[i]) > 0.001f) {
            write(LOG_BOTH, LOG_WARN,
                  "[BROWNOUT] PID %u outputSum %.2f clamped to %.2f",
                  (unsigned)i, c.pid_iTerm[i], clamped);
        }
        pid[i].SetOutputSum(clamped);
    }
}

// Decide whether a loaded cache is "in-flight" — i.e. we should resume.
static bool snapshotIsInFlight(const FlightCache &c) {
    // In-flight means phase is BOOST/COAST/DESCENT (not TRANSPORT/PAD/READY/RECOVERY)
    FlightPhase p = (FlightPhase)c.flightPhase;
    return (p == BOOST || p == COAST || p == DESCENT) && c.armed == 1;
}

// Compute staleness: how many ms ago snapshot was taken, vs. now.
// If snapshot epoch is 0 (no RTC) we fall back to comparing liftoffAgeMs only
// to current millis increment AND treat the cache as "best effort".
static unsigned long snapshotAgeMs(const FlightCache &c) {
    extern std::atomic<unsigned long long> systemBaseEpochMs;
    extern std::atomic<unsigned long>       systemBaseMillis;
    unsigned long long baseEp = systemBaseEpochMs.load(std::memory_order_relaxed);
    unsigned long      baseMs = systemBaseMillis.load(std::memory_order_relaxed);
    if (c.snapshotEpoch_ms == 0 || baseEp == 0) {
        // We have no wall clock reference. Fall back: if RTC kept power the
        // millis() increment is preserved through soft reset, so the snapshot
        // age is approximately (millis() - last SystemBaseMillis?). Best
        // effort: treat as "fresh" — we'll sweep from RTC where possible.
        return 0;  // assume fresh
    }
    uint64_t nowEpoch = baseEp + (millis() - baseMs);
    return (unsigned long)(nowEpoch - c.snapshotEpoch_ms);
}

// Fast mid-flight-reset detector — RTC slow-memory ONLY (microseconds, no
// flash/SD/wire traffic). Called first thing in setup(), BEFORE any hardware
// init, to decide whether to skip the power-settle delays and boot directly
// into resume. Returns true only when a valid in-flight snapshot exists on
// the always-kept RTC cache (which itself lives across brownouts/soft resets).
//
// NVS/SD are deliberately NOT touched here (flash wear + latency). The
// authoritative check still runs later via flightCacheLoadAndApply(), so we
// only risk an unnecessary "fast boot" (harmless — non-critical subsystems
// come up from DeferredInitTask in the background).
static bool flightCacheExpediteBoot() {
    if (!enableBrownoutRecovery) return false;
    if (rtcCache.magic    != BROWNOUT_CACHE_MAGIC ||
        rtcCache.version  != BROWNOUT_CACHE_VERSION) return false;
    if (!snapshotIsInFlight(rtcCache)) return false;

    // Staleness guard: only meaningful if the operator had synced the wall
    // clock (systemBaseEpochMs) — at boot the RAM copy is 0, so this matches
    // snapshotAgeMs() and treats RTC-continuous resumes as "fresh". The 60s
    // window still gets enforced by flightCacheLoadAndApply().
    extern std::atomic<unsigned long long> systemBaseEpochMs;
    extern std::atomic<unsigned long>       systemBaseMillis;
    unsigned long long baseEp = systemBaseEpochMs.load(std::memory_order_relaxed);
    unsigned long      baseMs = systemBaseMillis.load(std::memory_order_relaxed);
    if (baseEp > 0 && rtcCache.snapshotEpoch_ms != 0) {
        uint64_t nowEpoch = baseEp + (millis() - baseMs);
        if ((nowEpoch > rtcCache.snapshotEpoch_ms) &&
            (nowEpoch - rtcCache.snapshotEpoch_ms) >
                (uint64_t)(BROWNOUT_CACHE_VALIDITY_S * 1000ULL)) {
            return false;
        }
    }
    return true;
}

// Peek the last-commanded servo angles straight from the RTC cache (RTC slow
// memory only — microseconds, no flash/SD, safe before any hardware init).
// Used by initServos() on the expedited-resume path so the fins are driven to
// where they were before the reset instead of a hardcoded center value.
// Returns false if no valid in-flight snapshot exists to seed from.
static bool peekCachedServoAngles(float (&out)[8]) {
    if (rtcCache.magic   != BROWNOUT_CACHE_MAGIC ||
        rtcCache.version != BROWNOUT_CACHE_VERSION) return false;
    if (!snapshotIsInFlight(rtcCache)) return false;
    memcpy(out, rtcCache.servoAngles, sizeof(float[8]));
    return true;
}

bool flightCacheLoadAndApply() {
    if (!enableBrownoutRecovery) return false;

    FlightCache c;
    bool ok = false;
    const char *source = "(none)";

    if (loadFromNVS(c))  { ok = true; source = "NVS"; }
    else if (loadFromSD(c)) { ok = true; source = "SD"; }

    // RTC survives soft reset (which brownouts usually are); try last.
    if (!ok && rtcCache.magic == BROWNOUT_CACHE_MAGIC &&
        rtcCache.version == BROWNOUT_CACHE_VERSION) {
        memcpy(&c, &rtcCache, sizeof(c));
        ok = true; source = "RTC";
    }

    if (!ok) {
        write(LOG_SERIAL, LOG_INFO, "[BROWNOUT] No valid snapshot — cold boot");
        return false;
    }

    unsigned long age = snapshotAgeMs(c);
    unsigned long maxAgeMs = BROWNOUT_CACHE_VALIDITY_S * 1000UL;
    bool inFlight = snapshotIsInFlight(c);

    write(LOG_BOTH, LOG_INFO,
          "[BROWNOUT] Snapshot source=%s phase=%u armed=%u age=%lums inFlight=%s",
          source, (unsigned)c.flightPhase, (unsigned)c.armed, age,
          inFlight ? "YES" : "no");

    if (age > maxAgeMs) {
        write(LOG_BOTH, LOG_INFO,
              "[BROWNOUT] Snapshot stale (age %lums > %lus) — ignoring",
              age, BROWNOUT_CACHE_VALIDITY_S);
        return false;
    }
    if (!inFlight) {
        write(LOG_SERIAL, LOG_INFO,
              "[BROWNOUT] Snapshot was not in-flight — cold boot");
        return false;
    }

    // Restore in-flight state. Skip instrument arming gates — we have already
    // armed once before the brownout; the physical arm rail may or may not
    // still have power, but the operator chose to fly, so we resume.
    currentSystemMode.store((SystemMode)c.systemMode, std::memory_order_relaxed);
    currentPhase.store((FlightPhase)c.flightPhase, std::memory_order_relaxed);
    systemArmed.store(c.armed != 0, std::memory_order_relaxed);

    filter_alt       = c.filter_alt;
    V_z              = c.V_z;
    baseline_altitude = c.baseline_altitude;
    qnh_pressure      = c.qnh_pressure;

    // Liftoff time: age was captured pre-reboot. If RTC millis survived
    // (soft reset), millis() is continuous and liftoff_time_ms is unchanged.
    // If we hard-reset, millis wrapped to 0 and the cached liftoff_t is stale.
    // We can't always tell which happened, so we use the epoch-relative form
    // when available.
    extern std::atomic<unsigned long long> systemBaseEpochMs;
    extern std::atomic<unsigned long>      systemBaseMillis;
    unsigned long long baseEp = systemBaseEpochMs.load(std::memory_order_relaxed);
    unsigned long      baseMs = systemBaseMillis.load(std::memory_order_relaxed);
    if (baseEp > 0 && c.liftoffEpoch_ms > 0) {
        // liftoff time in millis() reference = baseMs + (liftoffEpoch - baseEp)
        unsigned long restored = baseMs + (unsigned long)(c.liftoffEpoch_ms - baseEp);
        liftoff_time_ms.store(restored, std::memory_order_relaxed);
    } else {
        // No wall clock — best-effort: keep cached value (works if soft reset)
        liftoff_time_ms.store(c.liftoffAgeMs, std::memory_order_relaxed);
    }

    flightCacheRestorePyros(c);
    flightCacheRestorePID(c);
    // enableBackupChute is static constexpr now (build-time-only); cannot be
    // restored from cache. Log if the cached value disagrees with the current
    // build setting so the operator knows the apparent state changed.
    if (c.backupEnabled != (enableBackupChute ? 1 : 0)) {
        write(LOG_BOTH, LOG_WARN,
              "[BROWNOUT] Cached backup_enabled=%u disagree with build flag=%d. Using BUILD flag.",
              (unsigned)c.backupEnabled, (int)enableBackupChute);
    }

    flightCacheValid    = true;
    flightCacheRestored = c;

    write(LOG_BOTH, LOG_WARN,
          "[BROWNOUT] RESUMING FLIGHT — phase=%u, V_z=%.2f, pyro=%u/%u/%u",
          (unsigned)c.flightPhase, V_z,
          (unsigned)pyroState[0].state, (unsigned)pyroState[1].state,
          (unsigned)pyroState[2].state);
    return true;
}

#endif // BROWNOUT_RECOVERY_H

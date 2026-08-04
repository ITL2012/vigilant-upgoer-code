#ifndef BUZZERS_H
#define BUZZERS_H

#include "globals.h"
#include <Arduino.h>

// ---- Buzzer Functions ----
// LEDC channel for buzzer (use channel 0, timer 0)
static constexpr int BUZZER_LEDC_CHANNEL = 0;
static bool buzzerLedsInitialized = false;

void initBuzzerLEDC() {
    if (buzzerLedsInitialized) return;
    // Initialize LEDC: use 12-bit resolution (max for 5kHz), 5000Hz base freq
    // ESP32-S3: max freq at 12-bit = 80MHz/4096 ≈ 19.5kHz, so 5kHz is fine
    if (ledcSetup(BUZZER_LEDC_CHANNEL, 5000, 12) == 0) {
        // Fallback: lower frequency if setup fails
        if (ledcSetup(BUZZER_LEDC_CHANNEL, 4000, 12) == 0) {
            ledcSetup(BUZZER_LEDC_CHANNEL, 2000, 11);
        }
    }
    // Attach buzzer pin to LEDC channel 0
    ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CHANNEL);
    buzzerLedsInitialized = true;
    write(LOG_BOTH, LOG_INFO, "[OK] Buzzer LEDC initialized");
}

void playTone(unsigned int freq, unsigned long duration_ms) {
    if (enableBuzzer != true) return;
    initBuzzerLEDC();
    tone(BUZZER_PIN, freq, duration_ms);
}

void armedAlarm() {
  static unsigned long lastBuzzerAction = 0;
  unsigned long currentMillis = millis();

  // Total cycle time = 300ms (100ms on + 200ms off) — non-blocking
  if (currentMillis - lastBuzzerAction >= 300) {
    playTone(1760, 100); // Plays for 100ms non-blocking
    lastBuzzerAction = currentMillis;
  }
}

void recoveryNoise() {
  // Instant safety cutoff if buzzer is disabled globally
  if (enableBuzzer != true) {
    noTone(BUZZER_PIN);
    return;
  }

  // Configurations
  const unsigned long TONE_DURATION  = 150;  // Length of each chirp pitch (ms)
  const unsigned long CYCLE_DURATION = 2700; // Total time for the whole pattern (1200ms sound + 1500ms silence)

  static unsigned long lastCycleStart = 0;
  unsigned long currentMillis = millis();

  // Reset our master clock cycle when the 2.7-second window finishes
  if (currentMillis - lastCycleStart >= CYCLE_DURATION) {
    lastCycleStart = currentMillis;
  }

  unsigned long elapsedInCycle = currentMillis - lastCycleStart;

  // The first 1200ms of the cycle plays 8 alternating chirps
  if (elapsedInCycle < 1200) {
    // Determine which chirp number we are currently on (0 through 7)
    int currentChirpIndex = elapsedInCycle / TONE_DURATION;

    // Even numbers play 2500Hz, Odd numbers play 4000Hz
    if (currentChirpIndex % 2 == 0) {
      playTone(2500, TONE_DURATION);
    } else {
      playTone(4000, TONE_DURATION);
    }
  }
  else {
    // The remaining 1500ms of the cycle is absolute silence
    noTone(BUZZER_PIN);
  }
}


// ============================================================================
// NON-BLOCKING CHIME SEQUENCER
// ============================================================================
// Chimes (startup/arm/disarm/mode-change) are multi-note patterns that used to
// call delay() between notes, blocking the control loop for ~1s. They are now
// scheduled sequences: the _Chime*() call installs the notes and returns
// immediately, and the notes play out in the background from buzzerUpdate(),
// which must be called from loop() each iteration.
static constexpr int MAX_CHIME_STEPS = 6;

struct ChimeStep {
    unsigned int  freq;   // Hz
    unsigned int  onMs;   // how long the note sounds
    unsigned int  gapMs;  // silence before the next note (or end)
};

static ChimeStep chimeSeq[MAX_CHIME_STEPS];
static int      chimeCount    = 0;
static int      chimeIndex    = 0;
static bool     chimeActive   = false;
static bool     chimeSounding = false;  // true = playing, false = in gap
static unsigned long chimePhaseStart = 0;

static void chimeStart(const ChimeStep steps[], int count) {
    if (enableBuzzer != true) return;
    initBuzzerLEDC();
    if (count > MAX_CHIME_STEPS) count = MAX_CHIME_STEPS;
    for (int i = 0; i < count; i++) chimeSeq[i] = steps[i];
    chimeCount    = count;
    chimeIndex    = 0;
    chimeSounding = true;
    chimeActive   = true;
    chimePhaseStart = millis();
    tone(BUZZER_PIN, chimeSeq[0].freq);   // start first note immediately
}

// Called once per loop() iteration. Advances the active chime through its
// notes/gaps with millis() timing only — never blocks.
void buzzerUpdate() {
    if (enableBuzzer != true) return;
    if (!chimeActive) return;

    unsigned long now = millis();
    const ChimeStep &s = chimeSeq[chimeIndex];

    if (chimeSounding) {
        // Note's sound window elapsed — silence (or end of sequence).
        if (now - chimePhaseStart >= s.onMs) {
            noTone(BUZZER_PIN);
            if (chimeIndex == chimeCount - 1 && s.gapMs == 0) {
                chimeActive = false;
                return;
            }
            chimeSounding  = false;
            chimePhaseStart = now;
        }
    } else {
        // Silence gap elapsed — advance to the next note.
        if (now - chimePhaseStart >= (unsigned long)s.gapMs) {
            chimeIndex++;
            if (chimeIndex >= chimeCount) {
                chimeActive = false;
                return;
            }
            chimeSounding  = true;
            chimePhaseStart = now;
            tone(BUZZER_PIN, chimeSeq[chimeIndex].freq);
        }
    }
}

// All chimes below are non-blocking: they schedule their note pattern and
// return immediately. The audible result is identical to the old delay()-based
// versions (same frequencies/durations/silences).

void startupChime() {
    static const ChimeStep steps[] = {
        {523,  100, 20},
        {659,  100, 20},
        {784,  150, 30},
        {1047, 220, 30},
    };
    chimeStart(steps, 4);
}

void errorBeep() {
    static const ChimeStep steps[] = {
        {200, 1000, 0},
    };
    chimeStart(steps, 1);
}

void armChime() {
    static const ChimeStep steps[] = {
        {880,  150, 30},
        {880,  150, 30},
        {1760, 400, 50},
    };
    chimeStart(steps, 3);
}

void disarmChime() {
    static const ChimeStep steps[] = {
        {1760, 150, 30},
        {880,  300, 50},
    };
    chimeStart(steps, 2);
}

void modeChangeChime() {
    static const ChimeStep steps[] = {
        {587,  80, 20},
        {880,  80, 20},
        {1175, 150, 30},
    };
    chimeStart(steps, 3);
}

// RECOVERY beacon - call once to start, then call recoveryNoise() each loop
static bool recoveryBeaconActive = false;

void recoveryBeaconStart() {
    recoveryBeaconActive = true;
    write(LOG_SERIAL, LOG_INFO, "[BUZZER] Recovery beacon started");
}

void recoveryBeaconStop() {
    recoveryBeaconActive = false;
    noTone(BUZZER_PIN);
    write(LOG_SERIAL, LOG_INFO, "[BUZZER] Recovery beacon stopped");
}

// Call this each loop iteration when in RECOVERY phase
void recoveryBeaconUpdate() {
    if (recoveryBeaconActive) {
        recoveryNoise();
    }
}
#endif // BUZZERS_H
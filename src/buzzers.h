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
  
  // Total cycle time = 100ms (on time) + 200ms (off time) = 300ms
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



void startupChime() {
    playTone(523, 100); delay(120);
    playTone(659, 100); delay(120);
    playTone(784, 150); delay(180);
    playTone(1047, 200); delay(250);
}

void errorBeep() {
    playTone(200, 1000);
}

void armChime() {
    playTone(880, 150); delay(180);
    playTone(880, 150); delay(180);
    playTone(1760, 400); delay(450);
}

void disarmChime() {
    playTone(1760, 150); delay(180);
    playTone(880, 300); delay(350);
}

void modeChangeChime() {
    playTone(587, 80); delay(100);
    playTone(880, 80); delay(100);
    playTone(1175, 150); delay(180);
}
#endif // BUZZERS_H
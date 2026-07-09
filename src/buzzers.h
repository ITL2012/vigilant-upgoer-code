#ifndef BUZZERS_H
#define BUZZERS_H

#include "globals.h"
#include <Arduino.h>

// ---- Buzzer Functions ----
// LEDC channel for buzzer (use channel 0, timer 0)
static constexpr int BUZZER_LEDC_CHANNEL = 0;
static bool buzzerLedsInitialized = false;

void armedAlarm() { // I dont know (but hope) it will stop when READY is off, but maybe not, thatll be fixed if needed
  // Localized variables that remember their state between calls
  static unsigned long lastBuzzerAction = 0;
  static bool buzzerIsOn = false;
  
  unsigned long currentMillis = millis();
  
  if (buzzerIsOn) {
    // If the buzzer has been on for 100ms, turn it off
    if (currentMillis - lastBuzzerAction >= 100) {
      noTone(BUZZER_PIN);
      buzzerIsOn = false;
      lastBuzzerAction = currentMillis;
    }
  } else {
    // If the buzzer has been off for 200ms, turn it back on
    if (currentMillis - lastBuzzerAction >= 200) {
      tone(BUZZER_PIN, 1760); // Sharp, high-urgency pitch
      buzzerIsOn = true;
      lastBuzzerAction = currentMillis;
    }
  }
}

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
    Serial.println("[OK] Buzzer LEDC initialized");
}

void playTone(unsigned int freq, unsigned long duration_ms) {
    if (enableBuzzer != true) return;
    initBuzzerLEDC();
    tone(BUZZER_PIN, freq, duration_ms);
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
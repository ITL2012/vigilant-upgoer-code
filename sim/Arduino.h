#ifndef ARDUINO_H
#define ARDUINO_H

#include <cstdint>
#include <algorithm>

inline float constrain(float x, float lo, float hi) { return std::max(lo, std::min(hi, x)); }
inline long constrain(long x, long lo, long hi) { return std::max(lo, std::min(hi, x)); }

uint32_t micros();
void sim_set_micros(uint32_t us);

#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0

inline void tone(int, unsigned int, unsigned long) {}
inline void noTone(int) {}
inline void delay(unsigned long) {}
inline void digitalWrite(int, int) {}
inline void pinMode(int, int) {}
inline int digitalRead(int) { return 0; }

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define RAD_TO_DEG (180.0f / M_PI)
#define DEG_TO_RAD (M_PI / 180.0f)

#define PROGMEM
#define PGM_P const char*

class String {
public:
    String() {}
    String(const char*) {}
    const char* c_str() const { return ""; }
    int toInt() const { return 0; }
    float toFloat() const { return 0.0f; }
};

#endif

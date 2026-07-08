#include "Arduino.h"

static uint32_t _sim_micros_state = 0;

uint32_t micros() { return _sim_micros_state; }
void sim_set_micros(uint32_t us) { _sim_micros_state = us; }

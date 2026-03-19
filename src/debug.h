#ifndef DEBUG_H_
#define DEBUG_H_

#include <stdint.h>
#include <stdbool.h>

void DebugInit();
void DebugSetLED(bool value);
void DebugBlinkLED(uint32_t delay);

#endif  // DEBUG_H_
#include "debug.h"

#include <hardware/gpio.h>
#include <pico/stdlib.h>

#define DEBUG_LED 12

void DebugInit() {
  gpio_init(DEBUG_LED);
  gpio_set_dir(DEBUG_LED, GPIO_OUT);
  gpio_put(DEBUG_LED, false);
}

void DebugSetLED(bool value) {
  gpio_put(DEBUG_LED, value);
}

void DebugBlinkLED(uint32_t delay) {
  gpio_put(DEBUG_LED, !gpio_get(DEBUG_LED));
  sleep_ms(delay);
  gpio_put(DEBUG_LED, !gpio_get(DEBUG_LED));
  sleep_ms(delay);
}
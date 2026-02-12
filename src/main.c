#include <hardware/gpio.h>
#include <pico/stdio.h>
#include "hub12.h"
#include "usb.h"

#define DEBUG_LED 12

int main(void) {
  gpio_init(DEBUG_LED);
  gpio_set_dir(DEBUG_LED, GPIO_OUT);
  gpio_put(DEBUG_LED, false);

  Hub12Initialize();
  USBInitialize();
  USBWaitForConfiguration();

  gpio_put(DEBUG_LED, true);

  while (true) {
    tight_loop_contents();
  }
}

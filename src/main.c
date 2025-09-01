#include <hardware/gpio.h>
#include <pico/stdio.h>
#include "hub12.h"
#include "usb.h"

int main(void) {
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  gpio_put(PICO_DEFAULT_LED_PIN, false);

  Hub12Initialize();
  USBInitialize();
  USBWaitForConfiguration();

  gpio_put(PICO_DEFAULT_LED_PIN, true);

  while (true) {
    tight_loop_contents();
  }
}

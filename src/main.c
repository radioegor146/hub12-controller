#include <string.h>

#include "debug.h"
#include "hub12.h"
#include "usb.h"

int main(void) {
  DebugInit();

  Hub12Initialize();

  USBInitialize();
  USBWaitForConfiguration();

  while (true) {
    DebugBlinkLED(200);
  }
}

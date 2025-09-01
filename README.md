# hub12-controller

Simple HUB12 LED matrices USB controller using RP2040
Currently it is ready to use for 224x192 (7x12) 32x16 monochrome (red) matrices 
It uses:
- PIO to control 12 simultaneous SPI lanes
- DMA to push data to PIO
- Low-level USB w/o any libraries to push data to buffer

Pins are set in [this file](src/hub12.h)
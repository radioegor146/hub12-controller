#ifndef HUB12_CONTROLLER_HUB12_H
#define HUB12_CONTROLLER_HUB12_H

#include <stdint.h>
#include <stdbool.h>

#define WIDTH 224
#define HEIGHT 192

#define BUFFER_SIZE (WIDTH * HEIGHT / 8)

#define MIN_DATA_PIN 0
#define MAX_DATA_PIN 11
#define CLOCK_PIN 12
#define LATCH_PIN 13
#define OUTPUT_ENABLE_PIN 22
#define A0_PIN 18
#define A1_PIN 19

#define USED_PIO pio0
#define USED_SM 0
#define PIO_CLOCK_DIVIDER 8

#define DMA_IRQ_TO_USE 0

#define LATCH_TOGGLE_CYCLES 100
#define PIO_END_WAIT_CYCLES 100

void Hub12Initialize();
uint8_t* Hub12GetAvailableBuffer();
void Hub12PushBuffer();

#endif  //HUB12_CONTROLLER_HUB12_H

# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

HUB12 LED matrix USB controller firmware for RP2040 (Raspberry Pi Pico). Controls a 224x192 pixel display (7x12 array of 32x16 monochrome red LED matrices) via USB.

## Build Commands

```bash
# Configure build (first time or after CMake changes)
cmake -B build -DPICO_SDK_PATH=$HOME/.pico-sdk/sdk/2.2.0

# Build firmware
cmake --build build

# Output: build/hub12_controller.uf2 (flash by copying to Pico in BOOTSEL mode)
```

## Architecture

### Hardware Interface Layers

1. **PIO Layer** (`hub12.pio`, `hub12.c`): Generates 12 simultaneous SPI data streams via RP2040's PIO hardware. The PIO program outputs 12 bits of data per clock cycle with hardware-controlled clock signal (sideset).

2. **DMA Layer** (`hub12.c`): Continuously streams framebuffer data to PIO without CPU intervention. Uses double buffering - while one buffer is being displayed, the other can be written to via USB.

3. **USB Layer** (`usb.c`): Low-level USB device implementation (no TinyUSB). Vendor-class device (VID: 0xE146, PID: 0x1337) with two bulk OUT endpoints:
   - EP1: Receives framebuffer data (5376 bytes for full frame)
   - EP2: Resets buffer position (sync mechanism)

### Data Flow

USB data → EP1 handler fills available buffer → `Hub12PushBuffer()` swaps buffers at frame boundary → DMA streams new buffer to PIO → PIO outputs to LED matrices

### Pin Configuration

Defined in `src/hub12.h`:
- Data pins: GPIO 0-11 (12 parallel SPI lanes)
- Clock: GPIO 23
- Latch: GPIO 24
- Output Enable: GPIO 25
- Row select: A0=GPIO 26, A1=GPIO 22

### Display Timing

The display uses 1:4 multiplexing (4 row groups). DMA interrupt handler cycles through rows and handles buffer swapping at frame boundaries to prevent tearing.

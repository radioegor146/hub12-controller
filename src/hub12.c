#include "hub12.h"

#include <hardware/dma.h>
#include <hardware/gpio.h>

#include "hub12.pio.h"

uint8_t buffer_1[BUFFER_SIZE] = {0};
uint8_t buffer_2[BUFFER_SIZE] = {0};

uint8_t* current_dma_buffer = buffer_1;
uint8_t* available_buffer = buffer_2;

int dma_channel = 0;
dma_channel_config dma_config = {};
uint32_t current_row = 0;
volatile bool swap_required = false;
volatile bool swap_ready = false;

static void Hub12SelectRow(uint8_t row) {
  gpio_put(A0_PIN, (row & 1) ? true : false);
  gpio_put(A1_PIN, (row & 2) ? true : false);
}

static void Hub12OutputEnable(bool enabled) {
  gpio_put(OUTPUT_ENABLE_PIN, enabled);
}

static void Hub12ToggleLatch() {
  gpio_put(LATCH_PIN, true);
  busy_wait_at_least_cycles(LATCH_TOGGLE_CYCLES);
  gpio_put(LATCH_PIN, false);
}

static void Hub12DMAFinished() {
  busy_wait_at_least_cycles(PIO_END_WAIT_CYCLES);
  Hub12OutputEnable(false);
  Hub12ToggleLatch();
  Hub12SelectRow(current_row);
  Hub12OutputEnable(true);

  current_row++;
  current_row %= 4;

  if (current_row == 0 && swap_required) {
    uint8_t* old_dma_buffer = current_dma_buffer;
    current_dma_buffer = available_buffer;
    available_buffer = old_dma_buffer;
    swap_required = false;
    swap_ready = true;
  }

  dma_channel_configure(dma_channel, &dma_config, &USED_PIO->txf[USED_SM],
                        current_dma_buffer + BUFFER_SIZE / 4 * current_row,
                        BUFFER_SIZE / 4 / 4, true);
}

static void Hub12DMAIRQHandler() {
  if (dma_channel >= 0 &&
      dma_irqn_get_channel_status(DMA_IRQ_TO_USE, dma_channel)) {
    dma_irqn_acknowledge_channel(DMA_IRQ_TO_USE, dma_channel);
    Hub12DMAFinished();
  }
}

static void Hub12PIODMAInitialize() {
  irq_set_exclusive_handler(dma_get_irq_num(DMA_IRQ_TO_USE),
                            Hub12DMAIRQHandler);
  irq_set_enabled(dma_get_irq_num(DMA_IRQ_TO_USE), true);
  irq_set_priority(dma_get_irq_num(DMA_IRQ_TO_USE), PICO_HIGHEST_IRQ_PRIORITY);

  dma_channel = dma_claim_unused_channel(false);
  if (dma_channel < 0) {
    return;
  }
  dma_config = dma_channel_get_default_config(dma_channel);
  channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_32);
  channel_config_set_read_increment(&dma_config, true);
  channel_config_set_write_increment(&dma_config, false);
  dma_irqn_set_channel_enabled(DMA_IRQ_TO_USE, dma_channel, true);
  channel_config_set_dreq(&dma_config, pio_get_dreq(USED_PIO, USED_SM, true));
}

static void Hub12PIOProgramInitialize() {
  uint32_t pio_program_offset = pio_add_program(USED_PIO, &hub12_program);

  uint32_t pin_count = MAX_DATA_PIN - MIN_DATA_PIN + 1;
  for (uint32_t i = MIN_DATA_PIN; i <= MAX_DATA_PIN; i++) {
    pio_gpio_init(USED_PIO, i);
  }
  pio_gpio_init(USED_PIO, CLOCK_PIN);
  pio_sm_set_consecutive_pindirs(USED_PIO, USED_SM, MIN_DATA_PIN, pin_count,
                                 true);
  pio_sm_set_consecutive_pindirs(USED_PIO, USED_SM, CLOCK_PIN, 1, true);
  pio_sm_config config = hub12_program_get_default_config(pio_program_offset);
  sm_config_set_out_pins(&config, MIN_DATA_PIN, pin_count);
  sm_config_set_sideset_pins(&config, CLOCK_PIN);
  sm_config_set_out_shift(&config, true, true, 32);
  sm_config_set_clkdiv(&config, PIO_CLOCK_DIVIDER);
  pio_sm_init(USED_PIO, USED_SM, pio_program_offset, &config);
  pio_sm_set_enabled(USED_PIO, USED_SM, true);
}

void Hub12Initialize() {
  gpio_init(LATCH_PIN);
  gpio_set_dir(LATCH_PIN, GPIO_OUT);
  gpio_set_outover(LATCH_PIN, GPIO_OVERRIDE_INVERT);
  gpio_put(LATCH_PIN, false);

  gpio_init(OUTPUT_ENABLE_PIN);
  gpio_set_dir(OUTPUT_ENABLE_PIN, GPIO_OUT);
  gpio_put(OUTPUT_ENABLE_PIN, false);

  gpio_init(A0_PIN);
  gpio_init(A1_PIN);
  gpio_set_dir(A0_PIN, GPIO_OUT);
  gpio_set_dir(A1_PIN, GPIO_OUT);
  gpio_put(A0_PIN, false);
  gpio_put(A1_PIN, false);

  Hub12PIOProgramInitialize();
  Hub12PIODMAInitialize();

  dma_channel_configure(dma_channel, &dma_config, &USED_PIO->txf[USED_SM],
                        current_dma_buffer + BUFFER_SIZE / 4 * current_row,
                        BUFFER_SIZE / 4 / 4, true);
}

uint8_t* Hub12GetAvailableBuffer() {
  return available_buffer;
}

void Hub12PushBuffer() {
  swap_required = true;
  while (!swap_ready) {
    tight_loop_contents();
  }
  swap_ready = false;
}
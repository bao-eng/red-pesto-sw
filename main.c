#include <stdio.h>
#include "pico/stdlib.h"

// https://www.reddit.com/r/raspberrypipico/comments/14q9wga/deep_sleep_on_c/
void dormant_sleep_until_interrupt()
{
  gpio_set_dormant_irq_enabled(9, IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_EDGE_LOW_BITS, true);
  xosc_dormant();
  gpio_acknowledge_irq(9, IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_EDGE_LOW_BITS);
  gpio_set_dormant_irq_enabled(9, IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_EDGE_LOW_BITS, false);

  clocks_init();
  stdio_init_all();
}

int main() {
  stdio_init_all();
  printf("Hello Dormant!\n");
  printf("XOSC going dormant until button is pressed\n");
  uart_default_tx_wait_blocking();

  dormant_sleep_until_interrupt();

  uint i = 0;
  while (1) {
      printf("XOSC awake %d\n", i++);
      sleep_ms(100);
  }
  return 0;
}
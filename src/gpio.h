#include "pico/stdlib.h"
#include "hardware/spi.h"

#include "board.h"

void red_pesto_gpio_init(gpio_irq_callback_t callback){
  spi_init(spi0, 500 * 1000);
  spi_set_format(spi0,  // SPI instance
                 8,     // Number of bits per transfer
                 1,     // Polarity (CPOL)
                 1,     // Phase (CPHA)
                 SPI_MSB_FIRST);
  gpio_set_function(SPI_MISO, GPIO_FUNC_SPI);
  gpio_set_function(SPI_CLK, GPIO_FUNC_SPI);
  gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);
  gpio_pull_up(SPI_CS);

  gpio_init(LED_GREEN);
  gpio_set_dir(LED_GREEN, GPIO_OUT);
  gpio_put(LED_GREEN, 0);

  // Chip select is active-low, so we'll initialise it to a driven-high state
  gpio_init(SPI_CS);
  gpio_set_dir(SPI_CS, GPIO_OUT);
  gpio_put(SPI_CS, 1);

  gpio_init(INT1);
  gpio_set_dir(INT1, GPIO_IN);
  gpio_pull_up(INT1);
  gpio_set_irq_enabled_with_callback(INT1, GPIO_IRQ_EDGE_RISE, true,
                                     callback);

  gpio_init(INT2);
  gpio_set_dir(INT2, GPIO_IN);
  gpio_pull_up(INT2);
  gpio_set_irq_enabled_with_callback(
      INT2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, callback);
}

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
  gpio_set_function(SPI_MISO_PIN, GPIO_FUNC_SPI);
  gpio_set_function(SPI_CLK_PIN, GPIO_FUNC_SPI);
  gpio_set_function(SPI_MOSI_PIN, GPIO_FUNC_SPI);
  gpio_pull_up(SPI_CS_PIN);

  gpio_init(LED_GREEN_PIN);
  gpio_set_dir(LED_GREEN_PIN, GPIO_OUT);
  gpio_put(LED_GREEN_PIN, 0);

  // Chip select is active-low, so we'll initialise it to a driven-high state
  gpio_init(SPI_CS_PIN);
  gpio_set_dir(SPI_CS_PIN, GPIO_OUT);
  gpio_put(SPI_CS_PIN, 1);

  gpio_init(INT1_PIN);
  gpio_set_dir(INT1_PIN, GPIO_IN);
  gpio_pull_up(INT1_PIN);
  gpio_set_irq_enabled_with_callback(INT1_PIN, GPIO_IRQ_EDGE_RISE, true,
                                     callback);

  gpio_init(INT2_PIN);
  gpio_set_dir(INT2_PIN, GPIO_IN);
  gpio_pull_up(INT2_PIN);
  gpio_set_irq_enabled_with_callback(
      INT2_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, callback);

  gpio_init(BQ_INT_PIN);
  gpio_set_dir(BQ_INT_PIN, GPIO_IN);
  gpio_pull_up(BQ_INT_PIN);
  gpio_set_irq_enabled_with_callback(BQ_INT_PIN, GPIO_IRQ_EDGE_RISE, true,
                                     callback);

  gpio_set_dir(CHRG_EN_PIN, GPIO_OUT);
  gpio_put(CHRG_EN_PIN, false);

  gpio_set_dir(BTN_PIN, GPIO_IN);
  gpio_set_irq_enabled_with_callback(BTN_PIN, GPIO_IRQ_EDGE_FALL, true,
                                      callback);
}

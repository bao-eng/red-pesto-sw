#include <stdio.h>
#include <string.h>

#include "src/board.h"
#include "src/lis2dh12.h"
#include "src/pwm.h"

void gpio_callback(uint gpio, uint32_t events) {
  static int int_count = 0;
  if (gpio == INT1 && events == GPIO_IRQ_EDGE_RISE) {
    gpio_put(LED, !gpio_get(LED));
    int16_t data_raw_acceleration[3];
    lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
  }
  if (gpio == INT2) {
    printf("INT2 :%d ", int_count);
    if (events == GPIO_IRQ_EDGE_FALL) {
      printf("sleep->wake\n");
      pwm_set_gpio_level(LED_PIN, 100);
    } else if (events == GPIO_IRQ_EDGE_RISE) {
      printf("wake->sleep\n");
      pwm_set_gpio_level(LED_PIN, 0);
    }
    int_count++;
  }
}

int main() {
  stdio_init_all();
  gpio_init(LED);
  gpio_set_dir(LED, GPIO_OUT);
  gpio_put(LED, 0);
  red_pesto_lis2dh12_init(gpio_callback);
  red_pesto_pwm_init();
  printf("LIS2DH12 demo\n");

  while (1) {
    tight_loop_contents();
  }
}

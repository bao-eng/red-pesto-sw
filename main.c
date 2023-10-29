#include <stdio.h>
#include <string.h>

#include "src/board.h"
#include "src/lis2dh12.h"
#include "src/pwm.h"
#include "src/gpio.h"

void gpio_callback(uint gpio, uint32_t events) {
  static int int_count = 0;
  if (gpio == INT1_PIN && events == GPIO_IRQ_EDGE_RISE) {
    gpio_put(LED_GREEN_PIN, !gpio_get(LED_GREEN_PIN));
    int16_t data_raw_acceleration[3];
    lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
  }
  if (gpio == INT2_PIN) {
    printf("INT2 :%d ", int_count);
    if (events == GPIO_IRQ_EDGE_FALL) {
      printf("sleep->wake\n");
      pwm_set_gpio_level(LED_POWER_PIN, 100);
    } else if (events == GPIO_IRQ_EDGE_RISE) {
      printf("wake->sleep\n");
      pwm_set_gpio_level(LED_POWER_PIN, 0);
    }
    int_count++;
  }
}

int main() {
  stdio_init_all();

  red_pesto_gpio_init(gpio_callback);
  red_pesto_lis2dh12_init();
  red_pesto_pwm_init();

  while (1) {
    tight_loop_contents();
  }
}

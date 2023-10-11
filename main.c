#include "pico/stdlib.h"
#include <stdio.h>
#include "pico/time.h"
#include "hardware/pwm.h"

#define LED_PIN 13
#define BTN_PIN 9

uint16_t pwm = 0;

void gpio_callback(uint gpio, uint32_t events) {
  static step =0;
  if (events == GPIO_IRQ_EDGE_FALL && gpio == BTN_PIN) {
    step++;
    if(step>8) step=0;
    pwm = step*800;
    pwm_set_gpio_level(LED_PIN, pwm);
  }
}
int main() {
    gpio_set_dir(BTN_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(BTN_PIN, GPIO_IRQ_EDGE_FALL, true,
                                     &gpio_callback);

    gpio_set_function(LED_PIN, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(LED_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_init(slice_num, &config, true);
    pwm_set_wrap(slice_num, 6400);
    pwm_set_gpio_level(LED_PIN, 0);

    while (1)
    {
        pwm_set_gpio_level(LED_PIN, 0);
        sleep_ms(1);
        pwm_set_gpio_level(LED_PIN, pwm);
        sleep_ms(1000);
        // tight_loop_contents();
    }
}
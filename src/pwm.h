#pragma once
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#include "board.h"

void red_pesto_pwm_init(){
  gpio_set_function(LED_POWER_PIN, GPIO_FUNC_PWM);

  uint slice_num = pwm_gpio_to_slice_num(LED_POWER_PIN);
  pwm_config config = pwm_get_default_config();
  pwm_init(slice_num, &config, true);
  pwm_set_wrap(slice_num, 6400);
  pwm_set_gpio_level(LED_POWER_PIN, 0);
}

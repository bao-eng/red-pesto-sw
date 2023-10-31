#pragma once
#include "hardware/adc.h"
#include "board.h"

void red_pesto_adc_init(){
  adc_init();
  adc_gpio_init(CC1_PIN);
  adc_gpio_init(CC2_PIN);
  adc_gpio_init(VBAT_PIN);
}

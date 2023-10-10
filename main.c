#include <stdio.h>

#include "bq256xx_charger.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

#define I2C_INST i2c0
#define I2C_SDA_PIN 24
#define I2C_SCL_PIN 25
#define CHRG_EN_PIN 10
#define STATUS_PIN 16
#define BQ_INT_PIN 11

#define CC1_PIN 26
#define CC2_PIN 27
#define CC1_ADC 0
#define CC2_ADC 1
#define VBAT_PIN 28
#define VBAT_ADC 2

#define BTN_PIN 9

// https://hackaday.com/2023/01/04/all-about-usb-c-resistors-and-emarkers/
#define VRD_USB_MIN 0.25f
#define VRD_USB_MAX 0.61f
#define VRD_15_MIN 0.7f
#define VRD_15_MAX 1.16f
#define VRD_30_MIN 1.31f
#define VRD_30_MAX 2.04f
#define VRA_MAX 0.15f

struct bq256xx_device bq25619e = {
    .i2c = I2C_INST,
    .dev_addr = 0x6A,
};

void gpio_callback(uint gpio, uint32_t events) {
  if (events == GPIO_IRQ_EDGE_RISE && gpio == BQ_INT_PIN) {
    busy_wait_ms(100);  // wait for the CC voltage to set
    const float conversion_factor = 3.3f / (1 << 12);
    adc_select_input(CC1_ADC);
    float cc1 = adc_read() * conversion_factor;
    adc_select_input(CC2_ADC);
    float cc2 = adc_read() * conversion_factor;
    float cc = cc1 > cc2 ? cc1 : cc2;
    if ((cc > VRD_USB_MIN) && (cc < VRD_USB_MAX)) {
      printf("vRd-USB (500mA)\n");
      bq256xx_set_input_curr_lim(&bq25619e, 500000);
    } else if ((cc > VRD_15_MIN) && (cc < VRD_15_MAX)) {
      printf("vRd-1.5\n");
      bq256xx_set_input_curr_lim(&bq25619e, 1500000);
    } else if ((cc > VRD_30_MIN) && (cc < VRD_30_MAX)) {
      printf("vRd-3.0\n");
      bq256xx_set_input_curr_lim(&bq25619e, 3000000);
    } else if (cc < VRA_MAX) {
      printf("vRa (disconnected)\n");
      bq256xx_set_input_curr_lim(&bq25619e, 500000);
    }
  }
  if (events == GPIO_IRQ_EDGE_FALL && gpio == BTN_PIN) {
    printf("BUTTON_INT!!!\n");
  }
}

int main() {
  stdio_init_all();

  adc_init();
  adc_gpio_init(CC1_PIN);
  adc_gpio_init(CC2_PIN);

  adc_gpio_init(VBAT_PIN);

  gpio_init(STATUS_PIN);
  gpio_set_dir(STATUS_PIN, GPIO_OUT);

  gpio_init(BQ_INT_PIN);
  gpio_set_dir(BQ_INT_PIN, GPIO_IN);
  gpio_pull_up(BQ_INT_PIN);
  gpio_set_irq_enabled_with_callback(BQ_INT_PIN, GPIO_IRQ_EDGE_RISE, true,
                                     &gpio_callback);

  gpio_set_dir(CHRG_EN_PIN, GPIO_OUT);
  gpio_put(CHRG_EN_PIN, false);

  gpio_set_dir(BTN_PIN, GPIO_IN);
  gpio_set_irq_enabled_with_callback(BTN_PIN, GPIO_IRQ_EDGE_FALL, true,
                                     &gpio_callback);

  i2c_init(I2C_INST, 100 * 1000);
  gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA_PIN);
  gpio_pull_up(I2C_SCL_PIN);

  bq256xx_charger_reset(&bq25619e);
  bq256xx_hw_init(&bq25619e);

  struct bq256xx_state state1;
  bq256xx_get_state(&bq25619e, &state1);

  while (1) {
    const float conversion_factor = 6.4f / (1 << 12);
    adc_select_input(VBAT_ADC);
    printf("VBAT: %.2f V\n", adc_read() * conversion_factor);
    sleep_ms(1000);
  }

  return 0;
}

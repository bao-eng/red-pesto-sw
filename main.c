#include <stdio.h>
#include <string.h>

#include "src/board.h"
#include "src/lis2dh12.h"
#include "src/pwm.h"
#include "src/gpio.h"
#include "src/adc.h"
#include "src/i2c.h"
#include "src/bq256xx_charger.h"

#define VRD_USB_MIN 0.25f
#define VRD_USB_MAX 0.61f
#define VRD_15_MIN 0.7f
#define VRD_15_MAX 1.16f
#define VRD_30_MIN 1.31f
#define VRD_30_MAX 2.04f
#define VRA_MAX 0.15f

bool acc_drdy_flag;
bool acc_wake_flag;
bool acc_sleep_flag;
bool bq_int_flag;
bool btn_int_flag;

void gpio_callback(uint gpio, uint32_t events) {
  if (gpio == INT1_PIN && events == GPIO_IRQ_EDGE_RISE) {
    acc_drdy_flag = true;
  }
  if (gpio == INT2_PIN) {
    if (events == GPIO_IRQ_EDGE_FALL) {
      acc_wake_flag = true;
    } else if (events == GPIO_IRQ_EDGE_RISE) {
      acc_sleep_flag = true;
    }
  }
  if (events == GPIO_IRQ_EDGE_RISE && gpio == BQ_INT_PIN) {
    bq_int_flag = true;
  }
  if (events == GPIO_IRQ_EDGE_FALL && gpio == BTN_PIN) {
    btn_int_flag = true;
  }
}

struct bq256xx_device bq25619e = {
  .i2c = i2c0,
  .dev_addr = 0x6A,
};

int main() {
  stdio_init_all();

  red_pesto_gpio_init(gpio_callback);
  red_pesto_lis2dh12_init();
  red_pesto_pwm_init();
  red_pesto_adc_init();
  red_pesto_i2c_init();

  bq256xx_charger_reset(&bq25619e);
  bq256xx_hw_init(&bq25619e);

  while (1) {
    if(acc_drdy_flag){
      gpio_put(LED_GREEN_PIN, !gpio_get(LED_GREEN_PIN));
      int16_t data_raw_acceleration[3];
      lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
      acc_drdy_flag = false;
    }
    if(acc_wake_flag){
      printf("sleep->wake\n");
      pwm_set_gpio_level(LED_POWER_PIN, 100);
      acc_wake_flag = false;
    }
    if(acc_sleep_flag){
      printf("wake->sleep\n");
      pwm_set_gpio_level(LED_POWER_PIN, 0);
      acc_sleep_flag = false;
    }
    if(bq_int_flag){
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
      bq_int_flag = false;
    }
    if(btn_int_flag){
      printf("BUTTON_INT!!!\n");
      btn_int_flag = false;
    }
  }
}

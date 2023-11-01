#include <stdio.h>
#include <string.h>

#include "src/board.h"
#include "src/common.h"
#include "src/lis2dh12.h"
#include "src/pwm.h"
#include "src/gpio.h"
#include "src/adc.h"
#include "src/i2c.h"
#include "src/bq25619e.h"
#include "src/cli.h"
#include "src/veml7700.h"

volatile bool acc_drdy_flag;
volatile bool acc_wake_flag;
volatile bool acc_sleep_flag;
volatile bool bq_int_flag;
volatile bool btn_int_flag;
volatile bool process_cli_flag;

bool timer_callback(repeating_timer_t *rt);
void gpio_callback(uint gpio, uint32_t events);

int main() {
  stdio_init_all();

  red_pesto_gpio_init(gpio_callback);
  red_pesto_lis2dh12_init();
  red_pesto_pwm_init();
  red_pesto_adc_init();
  red_pesto_i2c_init();
  red_pesto_bq25619e_init();
  red_pesto_init_cli();
  red_pesto_veml7700_init();

  repeating_timer_t timer;
  add_repeating_timer_ms(100, timer_callback, NULL, &timer);

  while (1) {
    if(acc_drdy_flag){
      gpio_put(LED_GREEN_PIN, !gpio_get(LED_GREEN_PIN));
      int16_t data_raw_acceleration[3];
      lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
      float acceleration_mg[3];
      for (int i = 0; i < 3; i++) {
        acceleration_mg[i] = lis2dh12_from_fs2_nm_to_mg(
                                    data_raw_acceleration[i]);
      }
      // this printf is the bottle neck for the accelerometer ODR (cli_printf ~2x times slower), usb stdio 10x slower than default 115200 uart
      // 50Hz - not ok; 25Hz - ok
      // printf("%4.2f %4.2f %4.2f\n", acceleration_mg[0]/1000.0f, acceleration_mg[1]/1000.0f, acceleration_mg[2]/1000.0f);
      acc_drdy_flag = false;
    }
    if(acc_wake_flag){
      DEBUG_PRINT("sleep->wake");
      pwm_set_gpio_level(LED_POWER_PIN, 100);
      acc_wake_flag = false;
    }
    if(acc_sleep_flag){
      DEBUG_PRINT("wake->sleep");
      pwm_set_gpio_level(LED_POWER_PIN, 0);
      acc_sleep_flag = false;
    }
    if(bq_int_flag){
      bq_int_handler();
      bq_int_flag = false;
    }
    if(btn_int_flag){
      DEBUG_PRINT("BUTTON_INT!!!");
      btn_int_flag = false;
    }
    if(process_cli_flag){
      int tmp = getchar_timeout_us(0);
      if(tmp != PICO_ERROR_TIMEOUT){
        char c = (char)tmp;
        EmbeddedCli *cli = getCliPointer();
        embeddedCliReceiveChar(cli, c);
        embeddedCliProcess(cli);
      }
      process_cli_flag = false;
    }
  }
}

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
  if (gpio == BQ_INT_PIN && events == GPIO_IRQ_EDGE_RISE) {
    bq_int_flag = true;
  }
  if (gpio == BTN_PIN && events == GPIO_IRQ_EDGE_FALL) {
    btn_int_flag = true;
  }
}

bool timer_callback(repeating_timer_t *rt) {
  process_cli_flag = true;
  return true;
}

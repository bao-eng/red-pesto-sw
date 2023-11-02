#include <stdio.h>
#include <string.h>

#include "src/board.h"
#include "src/lis2dh12.h"
#include "src/pwm.h"
#include "src/gpio.h"
#include "src/adc.h"
#include "src/i2c.h"
#include "src/bq25619e.h"
#include "src/cli.h"
#include "src/cli_bindings.h"
#include "src/veml7700.h"

#define MAX_LED_PWM 400.0f
#define MIN_LED_PWM 100.0f
#define MAX_ALS 30000.0f

volatile bool acc_drdy_flag;
volatile bool acc_wake_flag;
volatile bool acc_sleep_flag;
volatile bool bq_int_flag;
volatile bool btn_pressed_int_flag;
volatile bool btn_released_int_flag;
volatile bool process_cli_flag;
volatile bool veml_read_flag;
volatile bool led_on;

bool cli_timer_callback(repeating_timer_t *rt);
bool veml_timer_callback(repeating_timer_t *rt);
int64_t sleep_alarm_callback(alarm_id_t id, void *user_data);
int64_t button_alarm_callback(alarm_id_t id, void *user_data);
void gpio_callback(uint gpio, uint32_t events);
uint16_t get_led_pwm_adjusted(uint16_t als);

alarm_id_t sleep_alarm_id, button_alarm_id;

int main() {
  stdio_init_all();
  red_pesto_init_cli();
  red_pesto_cli_bindings(getCliPointer());
  red_pesto_gpio_init(gpio_callback);
  gpio_put(LED_GREEN_PIN, 1);
  sleep_ms(500);
  gpio_put(LED_GREEN_PIN, 0);
  red_pesto_lis2dh12_init();
  red_pesto_pwm_init();
  red_pesto_adc_init();
  red_pesto_i2c_init();
  red_pesto_bq25619e_init();

  red_pesto_veml7700_init();

  repeating_timer_t cli_timer;
  add_repeating_timer_ms(100, cli_timer_callback, NULL, &cli_timer);

  repeating_timer_t veml_timer;
  add_repeating_timer_ms(100, veml_timer_callback, NULL, &veml_timer);

  uint16_t als_val = 0;
  // uint32_t t_pressed, t_released;

  while (1) {
    if(acc_drdy_flag){
      // gpio_put(LED_GREEN_PIN, !gpio_get(LED_GREEN_PIN));
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
      if(sleep_alarm_id) cancel_alarm(sleep_alarm_id);
      led_on = true;
      pwm_set_gpio_level(LED_POWER_PIN, get_led_pwm_adjusted(als_val));
      acc_wake_flag = false;
    }
    if(acc_sleep_flag){
      DEBUG_PRINT("wake->sleep");
      sleep_alarm_id = add_alarm_in_ms(10000, sleep_alarm_callback, NULL, false);
      acc_sleep_flag = false;
    }
    if(bq_int_flag){
      bq_int_handler();
      bq_int_flag = false;
    }
    if(btn_pressed_int_flag){
      DEBUG_PRINT("button pressed!");
      button_alarm_id = add_alarm_in_ms(10000, button_alarm_callback, NULL, false);
      btn_pressed_int_flag = false;
    }
    if(btn_released_int_flag){
      DEBUG_PRINT("button released!");
      if(button_alarm_id) cancel_alarm(button_alarm_id);
      btn_released_int_flag = false;
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
    if(veml_read_flag){
      als_val = red_pesto_veml7700_get_average();
      uint16_t pwm_level = 0;
      if(led_on){
        pwm_level = get_led_pwm_adjusted(als_val);
        pwm_set_gpio_level(LED_POWER_PIN, pwm_level);
      }
      // printf("%d %d\n", als_val, pwm_level);
      veml_read_flag = 0;
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
  if (gpio == BTN_PIN) {
    if(events == GPIO_IRQ_EDGE_FALL) {
      btn_pressed_int_flag = true;
    }else if(events == GPIO_IRQ_EDGE_RISE) {
      btn_released_int_flag = true;
    }
  }
}

bool cli_timer_callback(repeating_timer_t *rt) {
  process_cli_flag = true;
  return true;
}

bool veml_timer_callback(repeating_timer_t *rt) {
  veml_read_flag = true;
  return true;
}

uint16_t get_led_pwm_adjusted(uint16_t als){
  uint16_t res = MIN_LED_PWM + (((MAX_LED_PWM-MIN_LED_PWM))/MAX_ALS)*(float)als;
  if (res > MAX_LED_PWM) res = MAX_LED_PWM;
  return res;
}

int64_t sleep_alarm_callback(alarm_id_t id, void *user_data) {
  led_on = false;
  pwm_set_gpio_level(LED_POWER_PIN, 0);
  return 0;
}

int64_t button_alarm_callback(alarm_id_t id, void *user_data) {
  DEBUG_PRINT("Entering shipping mode");
  gpio_put(LED_RED_PIN, 1);
  busy_wait_ms(2000);
  bq_enter_ship_mode();
  return 0;
}

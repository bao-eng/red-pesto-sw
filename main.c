#include <stdio.h>

#include "VEML7700.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

#define I2C0_SDA_PIN 20
#define I2C0_SCL_PIN 21

#define I2C1_SDA_PIN 14
#define I2C1_SCL_PIN 15

int main() {
  stdio_init_all();
  i2c_init(i2c0, 100 * 1000);
  gpio_set_function(I2C0_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(I2C0_SCL_PIN, GPIO_FUNC_I2C);
  // Make the I2C pins available to picotool
  bi_decl(bi_2pins_with_func(I2C0_SDA_PIN, I2C0_SCL_PIN, GPIO_FUNC_I2C));

  i2c_init(i2c1, 100 * 1000);
  gpio_set_function(I2C1_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(I2C1_SCL_PIN, GPIO_FUNC_I2C);

  veml7700_init(i2c0);
  veml7700_init(i2c1);
  while (1) {
    printf("%d | %d\n", veml7700_read_als_data(i2c1),
           veml7700_read_als_data(i2c0));
    sleep_ms(100);
  }
}

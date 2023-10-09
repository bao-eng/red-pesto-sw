#include <stdio.h>
#include <string.h>

#include "hardware/spi.h"
#include "lis2dh12-pid/lis2dh12_reg.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

#define SPI_MOSI 19
#define SPI_MISO 20
#define SPI_CLK 18
#define SPI_CS 21
#define INT1 17
#define INT2 16
#define LED 25

#define READ_BIT 0x80

#define BOOT_TIME 5  // ms

/* Self test limits converted from 10bit right-aligned to 16bit left-aligned. */
#define MIN_ST_LIMIT_LSb 17 * 64
#define MAX_ST_LIMIT_LSb 360 * 64

/* Private variables ---------------------------------------------------------*/
static int16_t data_raw_acceleration[3];
static float acceleration_st_mg[3];
static float acceleration_mg[3];
static uint8_t tx_buffer[1000];
static float test_val_mg[3];
static float max_st_limit_mg;
static float min_st_limit_mg;

stmdev_ctx_t dev_ctx; /** xxxxxxx is the used part number **/

static inline void cs_select() {
  gpio_put(SPI_CS, 0);  // Active low
}

static inline void cs_deselect() { gpio_put(SPI_CS, 1); }

int32_t platform_write(void *handle, uint8_t Reg, const uint8_t *Bufp,
                       uint16_t len) {
  cs_select();
  uint8_t reg = Reg & 0x7f;
  reg |= 0x40;
  spi_write_blocking(handle, &reg, 1);
  spi_write_blocking(handle, Bufp, len);
  cs_deselect();
  return 0;
}

int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len) {
  cs_select();
  uint8_t reg = Reg;
  reg |= 0xC0;
  spi_write_blocking(handle, &reg, 1);
  spi_read_blocking(handle, 0, Bufp, len);
  cs_deselect();
  return 0;
}
void platform_delay(uint32_t millisec) { busy_wait_ms(millisec); }

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
    } else if (events == GPIO_IRQ_EDGE_RISE) {
      printf("wake->sleep\n");
    }
    int_count++;
  }
}

int main() {
  stdio_init_all();
  printf("LIS2DH12 demo\n");
  spi_init(spi0, 500 * 1000);
  spi_set_format(spi0,  // SPI instance
                 8,     // Number of bits per transfer
                 1,     // Polarity (CPOL)
                 1,     // Phase (CPHA)
                 SPI_MSB_FIRST);
  gpio_set_function(SPI_MISO, GPIO_FUNC_SPI);
  gpio_set_function(SPI_CLK, GPIO_FUNC_SPI);
  gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);
  gpio_pull_up(SPI_CS);

  gpio_init(LED);
  gpio_set_dir(LED, GPIO_OUT);
  gpio_put(LED, 0);

  // Chip select is active-low, so we'll initialise it to a driven-high state
  gpio_init(SPI_CS);
  gpio_set_dir(SPI_CS, GPIO_OUT);
  gpio_put(SPI_CS, 1);

  gpio_init(INT1);
  gpio_set_dir(INT1, GPIO_IN);
  gpio_pull_up(INT1);
  gpio_set_irq_enabled_with_callback(INT1, GPIO_IRQ_EDGE_RISE, true,
                                     &gpio_callback);

  gpio_init(INT2);
  gpio_set_dir(INT2, GPIO_IN);
  gpio_pull_up(INT2);
  gpio_set_irq_enabled_with_callback(
      INT2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

  lis2dh12_reg_t reg;
  uint8_t i, j;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = spi0;

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  lis2dh12_device_id_get(&dev_ctx, &reg.byte);
  lis2dh12_device_id_get(&dev_ctx, &reg.byte);
  if (reg.byte == LIS2DH12_ID) {
    printf("LIS2DH12 detected\n");
  } else {
    printf("LIS2DH12 not found!\n");
  }

  /* Enable Block Data Update. */
  lis2dh12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set full scale to 2g. */
  lis2dh12_full_scale_set(&dev_ctx, LIS2DH12_2g);
  /* Set device in normal mode. */
  lis2dh12_operating_mode_set(&dev_ctx, LIS2DH12_NM_10bit);
  /* Set Output Data Rate to 1Hz. */
  lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_ODR_50Hz);

  lis2dh12_ctrl_reg3_t ctrl_reg3;
  ctrl_reg3.i1_zyxda = 1;
  lis2dh12_pin_int1_config_set(&dev_ctx, &ctrl_reg3);
  int16_t data_raw_acceleration[3];
  lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);

  // lis2dh12_int1_cfg_t int1_cfg;
  // int1_cfg.aoi = 0;
  // int1_cfg._6d = 0;
  // int1_cfg.xhie = 1;
  // int1_cfg.xlie = 0;
  // int1_cfg.yhie = 1;
  // int1_cfg.ylie = 0;
  // int1_cfg.zhie = 1;
  // int1_cfg.zlie = 0;
  // lis2dh12_int1_gen_conf_set(&dev_ctx, &int1_cfg);
  // lis2dh12_int1_gen_threshold_set(&dev_ctx, 50);
  // lis2dh12_int1_gen_duration_set(&dev_ctx, 1);
  // lis2dh12_ctrl_reg6_t ctrl_reg6;
  // ctrl_reg6.i2_ia1 = 1;
  // lis2dh12_pin_int2_config_set(&dev_ctx, &ctrl_reg6);
  lis2dh12_hp_t high_pass = LIS2DH12_DISC_FROM_INT_GENERATOR;
  lis2dh12_high_pass_int_conf_set(&dev_ctx, high_pass);

  lis2dh12_ctrl_reg6_t ctrl_reg6;
  ctrl_reg6.i2_act = 1;
  ctrl_reg6.int_polarity = 0;
  lis2dh12_pin_int2_config_set(&dev_ctx, &ctrl_reg6);
  lis2dh12_act_threshold_set(&dev_ctx, 10);
  lis2dh12_act_timeout_set(&dev_ctx, 10);

  while (1) {
    tight_loop_contents();
  }
}

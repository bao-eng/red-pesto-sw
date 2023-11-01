#pragma once
#include "pico/stdlib.h"
#include "hardware/spi.h"

#include "board.h"
#include "lis2dh12-pid/lis2dh12_reg.h"

#define BOOT_TIME 5  // ms

/* Self test limits converted from 10bit right-aligned to 16bit left-aligned. */
#define MIN_ST_LIMIT_LSb 17 * 64
#define MAX_ST_LIMIT_LSb 360 * 64

stmdev_ctx_t dev_ctx; /** xxxxxxx is the used part number **/

static inline void cs_select() {
  gpio_put(SPI_CS_PIN, 0);  // Active low
}

static inline void cs_deselect() { gpio_put(SPI_CS_PIN, 1); }

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

void red_pesto_lis2dh12_init(){
  lis2dh12_reg_t reg;
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

  lis2dh12_ctrl_reg3_t ctrl_reg3;
  ctrl_reg3.i1_zyxda = 1;
  lis2dh12_pin_int1_config_set(&dev_ctx, &ctrl_reg3);

  lis2dh12_hp_t high_pass = LIS2DH12_DISC_FROM_INT_GENERATOR;
  lis2dh12_high_pass_int_conf_set(&dev_ctx, high_pass);

  lis2dh12_ctrl_reg6_t ctrl_reg6;
  ctrl_reg6.i2_act = 1;
  ctrl_reg6.int_polarity = 0;
  lis2dh12_pin_int2_config_set(&dev_ctx, &ctrl_reg6);
  lis2dh12_act_threshold_set(&dev_ctx, 10);
  lis2dh12_act_timeout_set(&dev_ctx, 10);

  /* Enable Block Data Update. */
  lis2dh12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set full scale to 2g. */
  lis2dh12_full_scale_set(&dev_ctx, LIS2DH12_2g);
  /* Set device in normal mode. */
  lis2dh12_operating_mode_set(&dev_ctx, LIS2DH12_NM_10bit);
  /* Set Output Data Rate to 1Hz. */
  lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_ODR_50Hz);

  int16_t data_raw_acceleration[3];
  lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
}

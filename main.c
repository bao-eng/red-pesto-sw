/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

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
  // Make the SPI pins available to picotool
  // bi_decl(bi_3pins_with_func(PICO_DEFAULT_SPI_RX_PIN,
  // PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI));

  // Chip select is active-low, so we'll initialise it to a driven-high state
  gpio_init(SPI_CS);
  gpio_set_dir(SPI_CS, GPIO_OUT);
  gpio_put(SPI_CS, 1);
  // Make the CS pin available to picotool
  // bi_decl(bi_1pin_with_name(PICO_DEFAULT_SPI_CSN_PIN, "SPI CS"));

  lis2dh12_reg_t reg;
  stmdev_ctx_t dev_ctx; /** xxxxxxx is the used part number **/
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
  /* Wait stable output */
  platform_delay(90);

  /* Check if new value available */
  do {
    lis2dh12_status_get(&dev_ctx, &reg.status_reg);
  } while (!reg.status_reg.zyxda);

  /* Read dummy data and discard it */
  lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
  /* Read 5 sample and get the average vale for each axis */
  memset(acceleration_mg, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lis2dh12_status_get(&dev_ctx, &reg.status_reg);
    } while (!reg.status_reg.zyxda);

    /* Read data and accumulate the mg value */
    lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);

    for (j = 0; j < 3; j++) {
      acceleration_mg[j] +=
          lis2dh12_from_fs2_nm_to_mg(data_raw_acceleration[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    acceleration_mg[i] /= 5.0f;
  }

  /* Enable Self Test positive (or negative) */
  lis2dh12_self_test_set(&dev_ctx, LIS2DH12_ST_POSITIVE);
  // lis2dh12_self_test_set(&dev_ctx, LIS2DH12_ST_NEGATIVE);
  /* Wait stable output */
  platform_delay(90);

  /* Check if new value available */
  do {
    lis2dh12_status_get(&dev_ctx, &reg.status_reg);
  } while (!reg.status_reg.zyxda);

  /* Read dummy data and discard it */
  lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
  /* Read 5 sample and get the average vale for each axis */
  memset(acceleration_st_mg, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lis2dh12_status_get(&dev_ctx, &reg.status_reg);
    } while (!reg.status_reg.zyxda);

    /* Read data and accumulate the mg value */
    lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);

    for (j = 0; j < 3; j++) {
      acceleration_st_mg[j] +=
          lis2dh12_from_fs2_nm_to_mg(data_raw_acceleration[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    acceleration_st_mg[i] /= 5.0f;
  }

  /* Calculate the mg values for self test */
  for (i = 0; i < 3; i++) {
    test_val_mg[i] = fabs((acceleration_st_mg[i] - acceleration_mg[i]));
  }

  min_st_limit_mg = lis2dh12_from_fs2_nm_to_mg(MIN_ST_LIMIT_LSb);
  max_st_limit_mg = lis2dh12_from_fs2_nm_to_mg(MAX_ST_LIMIT_LSb);

  /* Check self test limit */
  for (i = 0; i < 3; i++) {
    if ((min_st_limit_mg < test_val_mg[i]) &&
        (test_val_mg[i] < max_st_limit_mg)) {
      printf(
          "Axis[%d]: lmt min %4.2f mg - lmt max %4.2f mg - val %4.2f mg - "
          "PASS\r\n",
          i, min_st_limit_mg, max_st_limit_mg, test_val_mg[i]);
    }

    else {
      printf(
          "Axis[%d]: lmt min %4.2f mg - lmt max %4.2f mg - val %4.2f mg - "
          "FAIL\r\n",
          i, min_st_limit_mg, max_st_limit_mg, test_val_mg[i]);
    }
  }

  /* Disable Self Test */
  lis2dh12_self_test_set(&dev_ctx, LIS2DH12_ST_DISABLE);
  /* Disable sensor. */
  lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_POWER_DOWN);
}
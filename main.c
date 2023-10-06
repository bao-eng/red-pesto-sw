/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"

#include "lis2dh12-pid/lis2dh12_reg.h"

#define READ_BIT 0x80

#define    BOOT_TIME            5 //ms

/* Self test limits converted from 10bit right-aligned to 16bit left-aligned. */
#define    MIN_ST_LIMIT_LSb     17*64
#define    MAX_ST_LIMIT_LSb    360*64

/* Private variables ---------------------------------------------------------*/
static int16_t data_raw_acceleration[3];
static float acceleration_st_mg[3];
static float acceleration_mg[3];
static uint8_t tx_buffer[1000];
static float test_val_mg[3];
static float max_st_limit_mg;
static float min_st_limit_mg;

static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
    asm volatile("nop \n nop \n nop");
}

static void write_register(uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = reg & 0x7f;  // remove read bit as this is a write
    buf[1] = data;
    cs_select();
    spi_write_blocking(spi_default, buf, 2);
    cs_deselect();
    sleep_ms(10);
}

static void read_registers(uint8_t reg, uint8_t *buf, uint16_t len) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.
    reg |= READ_BIT;
    cs_select();
    spi_write_blocking(spi_default, &reg, 1);
    sleep_ms(10);
    spi_read_blocking(spi_default, 0, buf, len);
    cs_deselect();
    sleep_ms(10);
}

int32_t platform_write(void *handle, uint8_t Reg, const uint8_t *Bufp, uint16_t len){
    cs_select();
    spi_write_blocking(spi_default, &Reg, 1);
    spi_write_blocking(spi_default, Bufp, len);
    cs_deselect();
    sleep_ms(10);
    return 0;
}
int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len){
    cs_select();
    spi_write_blocking(spi_default, &Reg, 1);
    spi_read_blocking(spi_default, 0, Bufp, len);
    cs_deselect();
    return 0;
}
void platform_delay(uint32_t millisec){
    busy_wait_ms(millisec);
}

int main() {
    stdio_init_all();

    spi_init(spi_default, 500 * 1000);
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    // Make the SPI pins available to picotool
    bi_decl(bi_3pins_with_func(PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI));

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
    gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
    // Make the CS pin available to picotool
    bi_decl(bi_1pin_with_name(PICO_DEFAULT_SPI_CSN_PIN, "SPI CS"));

    lis2dh12_reg_t reg;
    stmdev_ctx_t dev_ctx; /** xxxxxxx is the used part number **/
    uint8_t i, j;
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = platform_delay;

    /* Wait sensor boot time */
    platform_delay(BOOT_TIME);
    /* Check device ID */
    lis2dh12_device_id_get(&dev_ctx, &reg.byte);

    if (reg.byte = LIS2DH12_ID)
    {
        printf("LIS2DH12 detected\n");
    }else {
        printf("LIS2DH12 not found!\n");
        while (1);
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
        acceleration_mg[j] += lis2dh12_from_fs2_nm_to_mg(
                                data_raw_acceleration[j]);
    }
    }

    /* Calculate the mg average values */
    for (i = 0; i < 3; i++) {
    acceleration_mg[i] /= 5.0f;
    }

    /* Enable Self Test positive (or negative) */
    lis2dh12_self_test_set(&dev_ctx, LIS2DH12_ST_POSITIVE);
    //lis2dh12_self_test_set(&dev_ctx, LIS2DH12_ST_NEGATIVE);
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
        acceleration_st_mg[j] += lis2dh12_from_fs2_nm_to_mg(
                                    data_raw_acceleration[j]);
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
    if (( min_st_limit_mg < test_val_mg[i] ) &&
        ( test_val_mg[i] < max_st_limit_mg)) {
        printf("Axis[%d]: lmt min %4.2f mg - lmt max %4.2f mg - val %4.2f mg - PASS\r\n",
                i, min_st_limit_mg, max_st_limit_mg, test_val_mg[i]);
    }

    else {
        printf("Axis[%d]: lmt min %4.2f mg - lmt max %4.2f mg - val %4.2f mg - FAIL\r\n",
                i, min_st_limit_mg, max_st_limit_mg, test_val_mg[i]);
    }

    // tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    /* Disable Self Test */
    lis2dh12_self_test_set(&dev_ctx, LIS2DH12_ST_DISABLE);
    /* Disable sensor. */
    lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_POWER_DOWN);

    // while (1) {

    //     sleep_ms(1000);
    // }
}
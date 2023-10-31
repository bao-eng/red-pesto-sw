#pragma once
#include "hardware/i2c.h"

#define BIT(nr) (1 << (nr))

#define GENMASK(h, l) ~(~0 << (h - l + 1)) << l

#define VELM7700_DEV_ADR 0x10

// Gain selection
#define ALS_GAIN_MASK GENMASK(12, 11)
#define ALS_GAIN_BIT_SHIFT 11
#define ALS_GAIN_1 0b00
#define ALS_GAIN_2 0b01
#define ALS_GAIN_0_25 0b10
#define ALS_GAIN_0_125 0b11

// ALS integration time setting
#define ALS_IT_MASK GENMASK(9, 6)
#define ALS_IT_BIT_SHIFT 6
#define ALS_IT_25MS 0b1100
#define ALS_IT_50MS 0b1000
#define ALS_IT_100MS 0b0000
#define ALS_IT_200MS 0b0001
#define ALS_IT_400MS 0b0010
#define ALS_IT_800MS 0b0011

// ALS persistence protect number setting
#define ALS_PERS_MASK GENMASK(5, 4)
#define ALS_PERS_BIT_SHIFT 4
#define ALS_PERS_1 0b00
#define ALS_PERS_2 0b01
#define ALS_PERS_4 0b10
#define ALS_PERS_8 0b11

// ALS interrupt enable setting
#define ALS_INT_EN_MASK BIT(1)
#define ALS_INT_EN_BIT_SHIFT 1
#define ALS_INT_EN_DISABLE 0
#define ALS_INT_EN_ENABLE 1

// ALS shut down setting
#define ALS_SD_MASK BIT(0)
#define ALS_SD_BIT_SHIFT 0
#define ALS_SD_POWERON 0
#define ALS_SD_SHUTDOWN 1

void veml7700_reg_write(i2c_inst_t *i2c, uint8_t reg_addr, uint16_t data) {
  i2c_write_blocking(i2c, VELM7700_DEV_ADR, &reg_addr, 1, true);
  i2c_write_blocking(i2c, VELM7700_DEV_ADR, (uint8_t *)&data, 2, false);
}

uint16_t veml7700_reg_read(i2c_inst_t *i2c, uint8_t reg_addr) {
  uint16_t buf;
  i2c_write_blocking(i2c, VELM7700_DEV_ADR, &reg_addr, 1, true);
  i2c_read_blocking(i2c, VELM7700_DEV_ADR, (uint8_t *)&buf, 2, false);
  return buf;
}

void veml7700_init(i2c_inst_t *i2c) {
  veml7700_reg_write(i2c, 0x00,
                     0 | ALS_GAIN_1 << ALS_GAIN_BIT_SHIFT |
                         ALS_IT_100MS << ALS_IT_BIT_SHIFT |
                         ALS_PERS_1 << ALS_PERS_BIT_SHIFT |
                         ALS_INT_EN_DISABLE << ALS_INT_EN_BIT_SHIFT |
                         ALS_SD_POWERON << ALS_SD_BIT_SHIFT);
}

uint16_t veml7700_read_als_data(i2c_inst_t *i2c) {
  return veml7700_reg_read(i2c, 0x04);
}

void red_pesto_veml7700_init(){
  veml7700_init(i2c0);
  veml7700_init(i2c1);
}

/**
 * Copyright (C) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "bma4_defs.h"
#include "common.h"

/******************************************************************************/
/*!                       Macro definitions                                   */

/*! Read write length varies based on user requirement */
#define BMA4_READ_WRITE_LEN  UINT8_C(46)

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;

i2c_inst_t *i2c_inst;


/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function
 */
BMA4_INTF_RET_TYPE bma4_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    // return coines_read_i2c(COINES_I2C_BUS_0, dev_address, reg_addr, reg_data, (uint16_t)len);
    // volatile uint8_t reg_addr1 = reg_addr;

    size_t nbytes = 0;
    i2c_write_blocking(i2c_inst, dev_addr, &reg_addr, 1, true);
    nbytes = i2c_read_blocking(i2c_inst, dev_addr, reg_data, len, false);
    // return BMA4_INTF_RET_SUCCESS;
    if(nbytes == len)
    {
        return BMA4_INTF_RET_SUCCESS;
    }else
    {
        return 1;
    }
}

/*!
 * I2C write function map to COINES platform
 */
BMA4_INTF_RET_TYPE bma4_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    // return coines_write_i2c(COINES_I2C_BUS_0, dev_address, reg_addr, (uint8_t *)reg_data, (uint16_t)len);
    size_t nbytes = 0;
    i2c_write_blocking(i2c_inst, dev_addr, &reg_addr, 1, true);
    nbytes = i2c_write_blocking(i2c_inst, dev_addr, reg_data, len, false);
    // return BMA4_INTF_RET_SUCCESS;
    if(nbytes == len)
    {
        return BMA4_INTF_RET_SUCCESS;
    }else
    {
        return 1;
    }
}

/*!
 * SPI read function map to COINES platform
 */
BMA4_INTF_RET_TYPE bma4_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    // uint8_t dev_address = *(uint8_t*)intf_ptr;

    // (void)intf_ptr;

    // return coines_read_spi(COINES_SPI_BUS_0, dev_address, reg_addr, reg_data, (uint16_t)len);
}

/*!
 * SPI write function map to COINES platform
 */
BMA4_INTF_RET_TYPE bma4_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    // uint8_t dev_address = *(uint8_t*)intf_ptr;

    // (void)intf_ptr;

    // return coines_write_spi(COINES_SPI_BUS_0, dev_address, reg_addr, (uint8_t *)reg_data, (uint16_t)len);
}

/*!
 * Delay function
 */
void bma4_delay_us(uint32_t period, void *intf_ptr)
{
    sleep_us(period);
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 *  Also to initialize coines platform
 */
int8_t bma4_i2c_interface_init(struct bma4_dev *bma, i2c_inst_t *i2c, enum bma4_variant variant)
{
    int8_t rslt = BMA4_OK;

    /* To initialize the user I2C function */
    i2c_inst = i2c;
    dev_addr = BMA4_I2C_ADDR_PRIMARY;
    bma->intf = BMA4_I2C_INTF;
    bma->bus_read = bma4_i2c_read;
    bma->bus_write = bma4_i2c_write;

    /* Assign variant information */
    bma->variant = variant;

    /* Assign device address to interface pointer */
    bma->intf_ptr = &dev_addr;

    /* Configure delay in microseconds */
    bma->delay_us = bma4_delay_us;

    /* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
    bma->read_write_len = BMA4_READ_WRITE_LEN;

    /* Set Performance mode status */
    bma->perf_mode_status = BMA4_DISABLE;

    sleep_ms(100);

    return rslt;
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bma4_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BMA4_OK)
    {
        printf("%s\t", api_name);
        if (rslt == BMA4_E_NULL_PTR)
        {
            printf("Error [%d] : Null pointer\r\n", rslt);
        }
        else if (rslt == BMA4_E_COM_FAIL)
        {
            printf("Error [%d] : Communication failure\r\n", rslt);
        }
        else if (rslt == BMA4_E_CONFIG_STREAM_ERROR)
        {
            printf("Error [%d] : Invalid configuration stream\r\n", rslt);
        }
        else if (rslt == BMA4_E_SELF_TEST_FAIL)
        {
            printf("Error [%d] : Self test failed\r\n", rslt);
        }
        else if (rslt == BMA4_E_INVALID_SENSOR)
        {
            printf("Error [%d] : Device not found\r\n", rslt);
        }
        else if (rslt == BMA4_E_OUT_OF_RANGE)
        {
            printf("Error [%d] : Out of Range\r\n", rslt);
        }
        else if (rslt == BMA4_E_AVG_MODE_INVALID_CONF)
        {
            printf("Error [%d] : Invalid bandwidth and ODR combination in Accel Averaging mode\r\n", rslt);
        }
        else
        {
            /* For more error codes refer "*_defs.h" */
            printf("Error [%d] : Unknown error code\r\n", rslt);
        }
    }
}

/*!
 *  @brief Deinitializes coines platform
 *
 *  @return void.
 */
void bma4_coines_deinit(void)
{
}

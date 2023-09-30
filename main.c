#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#include "BMA456-Sensor-API/bma456.h"
// #include "bma456-rp2040/common.h"

#define I2C_INST i2c0
#define I2C_SDA_PIN 20
#define I2C_SCL_PIN 21

// // #define I2C_INST i2c1
// // #define I2C_SDA_PIN 14
// // #define I2C_SCL_PIN 15

// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

void loop_wait(){
    uint32_t val = 0xDEADBEEF;
    while(val==0xDEADBEEF);
}

// dev->bus_read(dev->dev_addr, addr, temp_buff, temp_len);
uint16_t bma4_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    // return coines_read_i2c(COINES_I2C_BUS_0, dev_address, reg_addr, reg_data, (uint16_t)len);
    // volatile uint8_t reg_addr1 = reg_addr;

    size_t nbytes = 0;
    i2c_write_blocking(I2C_INST, dev_addr, &reg_addr, 1, true);
    nbytes = i2c_read_blocking(I2C_INST, dev_addr, reg_data, len, false);
    // return BMA4_INTF_RET_SUCCESS;
    if(nbytes == len)
    {
        return 0;
    }else
    {
        return 1;
    }
}

/*!
 * I2C write function map to COINES platform
 */
uint16_t bma4_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    // return coines_write_i2c(COINES_I2C_BUS_0, dev_address, reg_addr, (uint8_t *)reg_data, (uint16_t)len);
    size_t nbytes = 0;
    i2c_write_blocking(I2C_INST, dev_addr, &reg_addr, 1, true);
    nbytes = i2c_write_blocking(I2C_INST, dev_addr, reg_data, len, false);
    // return BMA4_INTF_RET_SUCCESS;
    if(nbytes == len)
    {
        return 0;
    }else
    {
        return 1;
    }
}

void bma4_delay_us(long unsigned int period)
{
    // sleep_us(period);
    busy_wait_us(period);
}

int main() {
    // Enable UART so we can print status output
    stdio_init_all();
    // This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
    i2c_init(I2C_INST, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    // gpio_pull_up(I2C_SDA_PIN);
    // gpio_pull_up(I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(I2C_SDA_PIN, I2C_SCL_PIN, GPIO_FUNC_I2C));

    loop_wait();
    // uint8_t buf = 0;
    // uint8_t regaddr = 0x00;
    // i2c_write_blocking(I2C_INST, 0x18, &regaddr, 1, true);
    // int numbytes = 0;
    // numbytes = i2c_read_blocking(I2C_INST, 0x18, &buf, 1, false);
    // printf("buf=%x\n", buf);
    // printf("numbytes=%d\n", numbytes);

    uint16_t rslt = BMA4_OK;
    uint8_t init_seq_status = 0;
    
    /* Declare an instance of the BMA456 device */
    struct bma4_dev dev;
    
    /* Modify the parameters */
    dev.dev_addr        = BMA4_I2C_ADDR_PRIMARY;
    dev.interface       = BMA4_I2C_INTERFACE;
    dev.bus_read        = bma4_i2c_read;
    dev.bus_write       = bma4_i2c_write;
    dev.delay           = bma4_delay_us;
    dev.read_write_len  = 8000;
    dev.resolution      = 12;
    dev.feature_len     = BMA456_FEATURE_SIZE;
    
    /* a. Reading the chip id. */
    rslt |= bma456_init(&dev);
    /* b. Performing initialization sequence. 
       c. Checking the correct status of the initialization sequence.
    */
    rslt |= bma456_write_config_file(&dev);
    
    return rslt;

}

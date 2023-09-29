#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#include "BMA456-Sensor-API/bma456_an.h"
#include "bma456-rp2040/common.h"

#define I2C_INST i2c0
#define I2C_SDA_PIN 20
#define I2C_SCL_PIN 21

// // #define I2C_INST i2c1
// // #define I2C_SDA_PIN 14
// // #define I2C_SCL_PIN 15

/******************************************************************************/
/*!                Macro definition                                           */

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH       (9.80665f)

/*! Macro that holds the total number of accel x,y and z axes sample counts to be printed */
#define ACCEL_SAMPLE_COUNT  UINT8_C(100)

/******************************************************************************/
/*!           Static Function Declaration                                     */

/*!
 *  @brief This internal API converts raw sensor values(LSB) to meters per seconds square.
 *
 *  @param[in] val       : Raw sensor value.
 *  @param[in] g_range   : Accel Range selected (2G, 4G, 8G, 16G).
 *  @param[in] bit_width : Resolution of the sensor.
 *
 *  @return Accel values in meters per second square.
 *
 */
static float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width);

// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

void loop_wait(){
    uint32_t val = 0xDEADBEEF;
    while(val==0xDEADBEEF);
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

    /* Variable to store the status of API */
    int8_t rslt;

    /* Sensor initialization configuration */
    struct bma4_dev bma = { 0 };

    /* Variable to store accel data ready interrupt status */
    uint16_t int_status = 0;

    /* Variable that holds the accelerometer sample count */
    uint8_t n_data = 1;

    struct bma4_accel sens_data = { 0 };
    float x = 0, y = 0, z = 0;
    struct bma4_accel_config accel_conf = { 0 };

    rslt = bma4_i2c_interface_init(&bma, I2C_INST, BMA45X_VARIANT);
    bma4_error_codes_print_result("bma4_interface_init", rslt);

    /* Sensor initialization */
    rslt = bma456_an_init(&bma);
    bma4_error_codes_print_result("bma456_an_init status", rslt);

    /* Upload the configuration file to enable the features of the sensor. */
    rslt = bma456_an_write_config_file(&bma);
    bma4_error_codes_print_result("bma456_an_write_config status", rslt);


    /* Accelerometer configuration settings */
    /* Output data Rate */
    accel_conf.odr = BMA4_OUTPUT_DATA_RATE_50HZ;

    /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G) */
    accel_conf.range = BMA4_ACCEL_RANGE_2G;

    /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
     * if it is set to 2, then 2^(bandwidth parameter) samples
     * are averaged, resulting in 4 averaged samples
     * Note1 : For more information, refer the datasheet.
     * Note2 : A higher number of averaged samples will result in a less noisier signal, but
     * this has an adverse effect on the power consumed.
     */
    accel_conf.bandwidth = BMA4_ACCEL_NORMAL_AVG4;

    /* Enable the filter performance mode where averaging of samples
     * will be done based on above set bandwidth and ODR.
     * There are two modes
     *  0 -> Averaging samples (Default)
     *  1 -> No averaging
     * For more info on No Averaging mode refer datasheet.
     */
    accel_conf.perf_mode = BMA4_CIC_AVG_MODE;

    /* Set the accel configurations */
    rslt = bma4_set_accel_config(&accel_conf, &bma);
    bma4_error_codes_print_result("bma4_set_accel_config status", rslt);

    /* NOTE : Enable accel after set of configurations */
    rslt = bma4_set_accel_enable(1, &bma);
    bma4_error_codes_print_result("bma4_set_accel_enable status", rslt);

    /* Mapping data ready interrupt with interrupt pin 1 to get interrupt status once getting new accel data */
    rslt = bma456_an_map_interrupt(BMA4_INTR1_MAP, BMA4_DATA_RDY_INT, BMA4_ENABLE, &bma);
    bma4_error_codes_print_result("bma456_an_map_interrupt status", rslt);

    printf("Data, Acc_Raw_X, Acc_Raw_Y, Acc_Raw_Z, Acc_ms2_X, Acc_ms2_Y, Acc_ms2_Z\n");

    for (;;)
    {
        /* Read interrupt status */
        rslt = bma456_an_read_int_status(&int_status, &bma);
        bma4_error_codes_print_result("bma456_an_read_int_status", rslt);

        /* Filtering only the accel data ready interrupt */
        if ((rslt == BMA4_OK) && (int_status & BMA4_ACCEL_DATA_RDY_INT))
        {
            /* Read the accel x, y, z data */
            rslt = bma4_read_accel_xyz(&sens_data, &bma);
            bma4_error_codes_print_result("bma4_read_accel_xyz status", rslt);

            if (rslt == BMA4_OK)
            {

                /* Converting lsb to meter per second squared for 16 bit resolution at 2G range */
                x = lsb_to_ms2(sens_data.x, (float)2, bma.resolution);
                y = lsb_to_ms2(sens_data.y, (float)2, bma.resolution);
                z = lsb_to_ms2(sens_data.z, (float)2, bma.resolution);

                /* Print the data in m/s2 */
                printf("%d, %d, %d, %d, %4.2f, %4.2f, %4.2f\n", n_data, sens_data.x, sens_data.y, sens_data.z, x, y, z);
            }

            /* Increment the count that determines the number of samples to be printed */
            n_data++;

            /* When the count reaches more than ACCEL_SAMPLE_COUNT, break and exit the loop */
            if (n_data > ACCEL_SAMPLE_COUNT)
            {
                break;
            }
        }
    }
}

/*!
 * @brief This internal API converts raw sensor values(LSB) to meters per seconds square.
 */
static float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

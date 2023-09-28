// /**
//  * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
//  *
//  * SPDX-License-Identifier: BSD-3-Clause
//  */

// #include "pico/stdlib.h"

// int main() {
//     const uint LED_PIN = 2;
//     gpio_init(LED_PIN);
//     gpio_set_dir(LED_PIN, GPIO_OUT);
//     while (true) {
//         gpio_put(LED_PIN, 1);
//         sleep_ms(250);
//         gpio_put(LED_PIN, 0);
//         sleep_ms(250);
//     }
// }


/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Fade an LED between low and high brightness. An interrupt handler updates
// the PWM slice's output level each time the counter wraps.

// #include "pico/stdlib.h"
// #include <stdio.h>
// #include "pico/time.h"
// #include "hardware/irq.h"
// #include "hardware/pwm.h"

// #define LED_PIN 13

// int main() {
//     // Tell the LED pin that the PWM is in charge of its value.
//     gpio_set_function(LED_PIN, GPIO_FUNC_PWM);
//     // Figure out which slice we just connected to the LED pin
//     uint slice_num = pwm_gpio_to_slice_num(LED_PIN);

//     // Get some sensible defaults for the slice configuration. By default, the
//     // counter is allowed to wrap over its maximum range (0 to 2**16-1)
//     pwm_config config = pwm_get_default_config();
//     // Set divider, reduces counter clock to sysclock/this value
//     (&config, 4.f);
//     // Load the configuration into our PWM slice, and set it running.
//     pwm_init(slice_num, &config, true);
//     pwm_set_gpio_level(LED_PIN, 0);

//     // Everything after this point happens in the PWM interrupt handler, so we
//     // can twiddle our thumbs
//     bool go_up = true;
//     uint16_t duty = 0;
//     while (1){
//         if(go_up){
//             duty++;
//             if(duty >=50){
//                 go_up=false;
//             }
//         }else{
//             duty--;
//             if(duty == 0){
//                 go_up=true;
//             }
//         }
//         pwm_set_gpio_level(LED_PIN, duty*duty);
//         sleep_ms(100);
//         // tight_loop_contents();
//     }
// }


/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Sweep through all 7-bit I2C addresses, to see if any slaves are present on
// the I2C bus. Print out a table that looks like this:
//
// I2C Bus Scan
//   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
// 0
// 1       @
// 2
// 3             @
// 4
// 5
// 6
// 7
//
// E.g. if slave addresses 0x12 and 0x34 were acknowledged.

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

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

int main() {
    // Enable UART so we can print status output
    stdio_init_all();
    // This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
    i2c_init(I2C_INST, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(I2C_SDA_PIN, I2C_SCL_PIN, GPIO_FUNC_I2C));

    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }

        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.

        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        if (reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = i2c_read_blocking(I2C_INST, addr, &rxdata, 1, false);

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
    printf("Done.\n");
    return 0;
}

#include <stdio.h>
#include "pico/stdlib.h"

int main(){
    //Initialise I/O
    stdio_init_all(); 

    // initialise GPIO (Green LED connected to pin 25)
    gpio_init(2);
    gpio_set_dir(2, GPIO_OUT);

    //Main Loop 
    while(1){
        gpio_put(2, 1); // Set pin 25 to high
        printf("LED ON!\n");
        sleep_ms(1000); // 0.5s delay

        gpio_put(2, 0); // Set pin 25 to low
        printf("LED OFF!\n");
        sleep_ms(1000); // 0.5s delay
    }
}

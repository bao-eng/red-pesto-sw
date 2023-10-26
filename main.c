#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"

#define EMBEDDED_CLI_IMPL
#include "embedded_cli.h"

static void writeCharToCli(EmbeddedCli *embeddedCli, char c) {
    putchar_raw(c);
}

static void reboot2bootloader(EmbeddedCli *cli, char *args, void *context) {
    reset_usb_boot(0, 0);
}

int main(){
    stdio_init_all(); 

    EmbeddedCliConfig *config = embeddedCliDefaultConfig();
    config->maxBindingCount = 16;
    EmbeddedCli *cli = embeddedCliNewDefault();
    void writeChar(EmbeddedCli *embeddedCli, char c);
    cli->writeChar = writeCharToCli;
    char c = (char)getchar_timeout_us(100);
    embeddedCliReceiveChar(cli, c);
    CliCommandBinding binding =
    {
        "reset_usb_boot",
        "Reboot the device into BOOTSEL mode",
        false,
        NULL,
        reboot2bootloader
    };
    embeddedCliAddBinding(cli, binding);

    while(1){
      char c = (char)getchar_timeout_us(0);
      embeddedCliReceiveChar(cli, c);
      embeddedCliProcess(cli);
      sleep_ms(100);
    }
}

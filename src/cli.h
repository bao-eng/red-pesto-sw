#include "pico/stdlib.h"
#include "pico/bootrom.h"

#define EMBEDDED_CLI_IMPL
#include "embedded_cli.h"

static EmbeddedCli *cli;

static void writeCharToCli(EmbeddedCli *embeddedCli, char c) {
    putchar_raw(c);
}

static void reboot2bootloader(EmbeddedCli *cli, char *args, void *context) {
    reset_usb_boot(0, 0);
}

void red_pesto_init_cli(){
    cli = embeddedCliNewDefault();
    void writeChar(EmbeddedCli *embeddedCli, char c);
    cli->writeChar = writeCharToCli;
    char c = (char)getchar_timeout_us(0);
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
}

EmbeddedCli *getCliPointer() {
    return cli;
}

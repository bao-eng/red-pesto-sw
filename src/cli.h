#pragma once
#include <stdarg.h>

#include "pico/stdlib.h"
#include "pico/bootrom.h"

#define EMBEDDED_CLI_IMPL
#include "embedded_cli.h"

#define CLI_PRINT_BUFFER_SIZE 512

static EmbeddedCli *cli;

static void writeCharToCli(EmbeddedCli *embeddedCli, char c) {
    putchar_raw(c);
}

static void reboot2bootloader(EmbeddedCli *cli, char *args, void *context) {
    reset_usb_boot(0, 0);
}

static void ship_mode(EmbeddedCli *cli, char *args, void *context) {
    bq_enter_ship_mode();
}

void red_pesto_init_cli(){
    cli = embeddedCliNewDefault();
    void writeChar(EmbeddedCli *embeddedCli, char c);
    cli->writeChar = writeCharToCli;
    char c = (char)getchar_timeout_us(0);
    embeddedCliReceiveChar(cli, c);

    CliCommandBinding dfu_binding =
    {
        "usb_boot",
        "Reboot the device into BOOTSEL mode",
        false,
        NULL,
        reboot2bootloader
    };
    embeddedCliAddBinding(cli, dfu_binding);

    CliCommandBinding ship_binding =
    {
        "ship_mode",
        "Put bq25619e charger into ship mode",
        false,
        NULL,
        ship_mode
    };
    embeddedCliAddBinding(cli, ship_binding);
}

EmbeddedCli *getCliPointer() {
    return cli;
}

// Function to encapsulate the 'embeddedCliPrint()' call with print formatting arguments (act like printf(), but keeps cursor at correct location).
// The 'embeddedCliPrint()' function does already add a linebreak ('\r\n') to the end of the print statement, so no need to add it yourself.
void cli_printf(const char *format, ...) {
    // Create a buffer to store the formatted string
    char buffer[CLI_PRINT_BUFFER_SIZE];

    // Format the string using snprintf
    va_list args;
    va_start(args, format);
    int length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    // Check if string fitted in buffer else print error to stderr
    if (length < 0) {
        printf("Error formatting the string\r\n");
        return;
    }

    // Call embeddedCliPrint with the formatted string
    embeddedCliPrint(getCliPointer(), buffer);
}

#pragma once
#include <stdarg.h>

#define EMBEDDED_CLI_IMPL
#include "embedded_cli.h"

#define CLI_PRINT_BUFFER_SIZE 512

static EmbeddedCli *cli;

void red_pesto_init_cli();
EmbeddedCli *getCliPointer();
void cli_printf(const char *format, ...);
static void writeCharToCli(EmbeddedCli *embeddedCli, char c);

#ifdef ENABLE_DEBUG_OUTPUT
#  define DEBUG_PRINT(msg, ...) cli_printf("[DEBUG] [%6.6f] " msg, to_us_since_boot(get_absolute_time())/1000000.0f, ##__VA_ARGS__);
#else
#  define DEBUG_PRINT(msg, ...)
#endif

void red_pesto_init_cli(){
    cli = embeddedCliNewDefault();
    void writeChar(EmbeddedCli *embeddedCli, char c);
    cli->writeChar = writeCharToCli;
    char c = (char)getchar_timeout_us(0);
    embeddedCliReceiveChar(cli, c);
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

static void writeCharToCli(EmbeddedCli *embeddedCli, char c) {
    putchar_raw(c);
}

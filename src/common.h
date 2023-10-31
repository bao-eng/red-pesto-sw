#pragma once
#include "cli.h"

#ifdef ENABLE_DEBUG_OUTPUT
#  define DEBUG_PRINT(msg, ...) cli_printf("[DEBUG] [%6.6f] " msg, to_us_since_boot(get_absolute_time())/1000000.0f, ##__VA_ARGS__);
#else
#  define DEBUG_PRINT(msg, ...) 
#endif

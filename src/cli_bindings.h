#pragma once
#include "pico/stdlib.h"
#include "pico/bootrom.h"

#include "cli.h"
#include "veml7700.h"
#include "bq25619e.h"

static void reboot2bootloader(EmbeddedCli *cli, char *args, void *context);
static void ship_mode(EmbeddedCli *cli, char *args, void *context);
static void get_als(EmbeddedCli *cli, char *args, void *context);
void red_pesto_cli_bindings(EmbeddedCli *embeddedCli);

static void reboot2bootloader(EmbeddedCli *cli, char *args, void *context) {
    reset_usb_boot(0, 0);
}

static void ship_mode(EmbeddedCli *cli, char *args, void *context) {
    bq_enter_ship_mode();
}

static void get_als(EmbeddedCli *cli, char *args, void *context) {
    cli_printf("%d", red_pesto_veml7700_get_average());
}

void red_pesto_cli_bindings(EmbeddedCli *embeddedCli){
  CliCommandBinding dfu_binding =
  {
      "usb_boot",
      "Reboot the device into BOOTSEL mode",
      false,
      NULL,
      reboot2bootloader
  };

  CliCommandBinding ship_binding =
  {
      "ship_mode",
      "Put bq25619e charger into ship mode",
      false,
      NULL,
      ship_mode
  };

  CliCommandBinding als_binding =
  {
      "get_als_data",
      "Get VEML7700 sensor data",
      false,
      NULL,
      get_als
  };
  embeddedCliAddBinding(embeddedCli, dfu_binding);
  embeddedCliAddBinding(embeddedCli, ship_binding);
  embeddedCliAddBinding(embeddedCli, als_binding);
}

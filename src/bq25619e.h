#pragma once
#include "hardware/i2c.h"
#include "bq256xx_charger.h"
#include "cli.h"

#define VRD_USB_MIN 0.25f
#define VRD_USB_MAX 0.61f
#define VRD_15_MIN 0.7f
#define VRD_15_MAX 1.16f
#define VRD_30_MIN 1.31f
#define VRD_30_MAX 2.04f
#define VRA_MAX 0.15f

struct bq256xx_device bq25619e = {
  .i2c = i2c0,
  .dev_addr = 0x6A,
};

void red_pesto_bq25619e_init(){
  bq256xx_charger_reset(&bq25619e);
  bq256xx_hw_init(&bq25619e);
  bq256xx_get_state(&bq25619e, &bq25619e.state);
}

void bq_int_handler(){
  struct bq256xx_state tmp_state;
  bq256xx_get_state(&bq25619e, &tmp_state);
  if(tmp_state.online){
    if(!bq25619e.state.online){
      DEBUG_PRINT("Charging cable connected");
      const float conversion_factor =  3.3f / (1 << 12);
      adc_select_input(CC1_ADC);
      float cc1 = adc_read() * conversion_factor;
      adc_select_input(CC2_ADC);
      float cc2 = adc_read() * conversion_factor;
      float cc = cc1 > cc2 ? cc1 : cc2;
      if ((cc > VRD_USB_MIN) && (cc < VRD_USB_MAX)) {
        DEBUG_PRINT("vRd-USB (500mA)");
        bq256xx_set_input_curr_lim(&bq25619e, 500000);
      } else if ((cc > VRD_15_MIN) && (cc < VRD_15_MAX)) {
        DEBUG_PRINT("vRd-1.5");
        bq256xx_set_input_curr_lim(&bq25619e, 1500000);
      } else if ((cc > VRD_30_MIN) && (cc < VRD_30_MAX)) {
        DEBUG_PRINT("vRd-3.0");
        bq256xx_set_input_curr_lim(&bq25619e, 3000000);
      }
    }
    switch (tmp_state.chrg_stat)
    {
      case BQ256XX_CHRG_STAT_NOT_CHRGING:
        DEBUG_PRINT("Not charging");
        break;
      case BQ256XX_CHRG_STAT_PRECHRGING:
        DEBUG_PRINT("Precharging");
        break;
      case BQ256XX_CHRG_STAT_FAST_CHRGING:
        DEBUG_PRINT("Fast charging");
        break;
      case BQ256XX_CHRG_STAT_CHRG_TERM:
        DEBUG_PRINT("Charging termination");
        break;
    }
    const float conversion_factor = 6.4f / (1 << 12);
    adc_select_input(VBAT_ADC);
    DEBUG_PRINT("VBAT: %.2f V\n", adc_read() * conversion_factor);
  }else{
    if(bq25619e.state.online){
      DEBUG_PRINT("Charging cable disconnected");
    }
  }
  bq25619e.state = tmp_state;
}

void bq_enter_ship_mode(){
  bq256xx_set_ship_mode(&bq25619e);
}

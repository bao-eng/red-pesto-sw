// SPDX-License-Identifier: GPL-2.0
// BQ256XX Battery Charger Driver
// Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/

#include "hardware/i2c.h"

#define BIT(nr) (1 << (nr))

#define GENMASK(h, l) ~(~0 << (h - l + 1)) << l

#define I2C_NAME_SIZE	20
#define	EINVAL		22	/* Invalid argument */
 #define MAX(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

 #define MIN(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

#define CLAMP(x, lower, upper) (MIN(upper, MAX(x, lower)))
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

#define BQ256XX_MANUFACTURER "Texas Instruments"

#define BQ256XX_INPUT_CURRENT_LIMIT		0x00
#define BQ256XX_CHARGER_CONTROL_0		0x01
#define BQ256XX_CHARGE_CURRENT_LIMIT		0x02
#define BQ256XX_PRECHG_AND_TERM_CURR_LIM	0x03
#define BQ256XX_BATTERY_VOLTAGE_LIMIT		0x04
#define BQ256XX_CHARGER_CONTROL_1		0x05
#define BQ256XX_CHARGER_CONTROL_2		0x06
#define BQ256XX_CHARGER_CONTROL_3		0x07
#define BQ256XX_CHARGER_STATUS_0		0x08
#define BQ256XX_CHARGER_STATUS_1		0x09
#define BQ256XX_CHARGER_STATUS_2		0x0a
#define BQ256XX_PART_INFORMATION		0x0b
#define BQ256XX_CHARGER_CONTROL_4		0x0c

#define BQ256XX_IINDPM_MASK		GENMASK(4, 0)
#define BQ256XX_IINDPM_STEP_uA		100000
#define BQ256XX_IINDPM_OFFSET_uA	100000
#define BQ256XX_IINDPM_MIN_uA		100000
#define BQ256XX_IINDPM_MAX_uA		3200000
#define BQ256XX_IINDPM_DEF_uA		2400000

#define BQ256XX_TS_IGNORE		BIT(6)
#define BQ256XX_TS_IGNORE_SHIFT		6

#define BQ256XX_VINDPM_MASK		GENMASK(3, 0)
#define BQ256XX_VINDPM_STEP_uV		100000
#define BQ256XX_VINDPM_OFFSET_uV	3900000
#define BQ256XX_VINDPM_MIN_uV		3900000
#define BQ256XX_VINDPM_MAX_uV		5400000
#define BQ256XX_VINDPM_DEF_uV		4500000

#define BQ256XX_VBATREG_MASK		GENMASK(7, 3)
#define BQ2560X_VBATREG_STEP_uV		32000
#define BQ2560X_VBATREG_OFFSET_uV	3856000
#define BQ2560X_VBATREG_MIN_uV		3856000
#define BQ2560X_VBATREG_MAX_uV		4624000
#define BQ2560X_VBATREG_DEF_uV		4208000
#define BQ25601D_VBATREG_OFFSET_uV	3847000
#define BQ25601D_VBATREG_MIN_uV		3847000
#define BQ25601D_VBATREG_MAX_uV		4615000
#define BQ25601D_VBATREG_DEF_uV		4199000
#define BQ2561X_VBATREG_STEP_uV		10000
#define BQ25611D_VBATREG_MIN_uV		3494000
#define BQ25611D_VBATREG_MAX_uV		4510000
#define BQ25611D_VBATREG_DEF_uV		4190000
#define BQ25618_VBATREG_MIN_uV		3504000
#define BQ25618_VBATREG_MAX_uV		4500000
#define BQ25618_VBATREG_DEF_uV		4200000
#define BQ256XX_VBATREG_BIT_SHIFT	3
#define BQ2561X_VBATREG_THRESH		0x8
#define BQ25611D_VBATREG_THRESH_uV	4290000
#define BQ25618_VBATREG_THRESH_uV	4300000

#define BQ256XX_CHG_CONFIG_MASK		BIT(4)
#define BQ256XX_CHG_CONFIG_BIT_SHIFT	4

#define BQ256XX_ITERM_MASK		GENMASK(3, 0)
#define BQ256XX_ITERM_STEP_uA		60000
#define BQ256XX_ITERM_OFFSET_uA		60000
#define BQ256XX_ITERM_MIN_uA		60000
#define BQ256XX_ITERM_MAX_uA		780000
#define BQ256XX_ITERM_DEF_uA		180000
#define BQ25618_ITERM_STEP_uA		20000
#define BQ25618_ITERM_OFFSET_uA		20000
#define BQ25618_ITERM_MIN_uA		20000
#define BQ25618_ITERM_MAX_uA		260000
#define BQ25618_ITERM_DEF_uA		60000

#define BQ256XX_IPRECHG_MASK		GENMASK(7, 4)
#define BQ256XX_IPRECHG_STEP_uA		60000
#define BQ256XX_IPRECHG_OFFSET_uA	60000
#define BQ256XX_IPRECHG_MIN_uA		60000
#define BQ256XX_IPRECHG_MAX_uA		780000
#define BQ256XX_IPRECHG_DEF_uA		180000
#define BQ25618_IPRECHG_STEP_uA		20000
#define BQ25618_IPRECHG_OFFSET_uA	20000
#define BQ25618_IPRECHG_MIN_uA		20000
#define BQ25618_IPRECHG_MAX_uA		260000
#define BQ25618_IPRECHG_DEF_uA		40000
#define BQ256XX_IPRECHG_BIT_SHIFT	4

#define BQ256XX_ICHG_MASK		GENMASK(5, 0)
#define BQ256XX_ICHG_STEP_uA		60000
#define BQ256XX_ICHG_MIN_uA		0
#define BQ256XX_ICHG_MAX_uA		3000000
#define BQ2560X_ICHG_DEF_uA		2040000
#define BQ25611D_ICHG_DEF_uA		1020000
#define BQ25618_ICHG_STEP_uA		20000
#define BQ25618_ICHG_MIN_uA		0
#define BQ25618_ICHG_MAX_uA		1500000
#define BQ25618_ICHG_DEF_uA		340000
#define BQ25618_ICHG_THRESH		0x3c
#define BQ25618_ICHG_THRESH_uA		1180000

#define BQ256XX_VBUS_STAT_MASK		GENMASK(7, 5)
#define BQ256XX_VBUS_STAT_NO_INPUT	0
#define BQ256XX_VBUS_STAT_USB_SDP	BIT(5)
#define BQ256XX_VBUS_STAT_USB_CDP	BIT(6)
#define BQ256XX_VBUS_STAT_USB_DCP	(BIT(6) | BIT(5))
#define BQ256XX_VBUS_STAT_USB_OTG	(BIT(7) | BIT(6) | BIT(5))

#define BQ256XX_CHRG_STAT_MASK		GENMASK(4, 3)
#define BQ256XX_CHRG_STAT_NOT_CHRGING	0
#define BQ256XX_CHRG_STAT_PRECHRGING	BIT(3)
#define BQ256XX_CHRG_STAT_FAST_CHRGING	BIT(4)
#define BQ256XX_CHRG_STAT_CHRG_TERM	(BIT(4) | BIT(3))

#define BQ256XX_PG_STAT_MASK		BIT(2)
#define BQ256XX_WDT_FAULT_MASK		BIT(7)
#define BQ256XX_CHRG_FAULT_MASK		GENMASK(5, 4)
#define BQ256XX_CHRG_FAULT_NORMAL	0
#define BQ256XX_CHRG_FAULT_INPUT	BIT(4)
#define BQ256XX_CHRG_FAULT_THERM	BIT(5)
#define BQ256XX_CHRG_FAULT_CST_EXPIRE	(BIT(5) | BIT(4))
#define BQ256XX_BAT_FAULT_MASK		BIT(3)
#define BQ256XX_NTC_FAULT_MASK		GENMASK(2, 0)
#define BQ256XX_NTC_FAULT_WARM		BIT(1)
#define BQ256XX_NTC_FAULT_COOL		(BIT(1) | BIT(0))
#define BQ256XX_NTC_FAULT_COLD		(BIT(2) | BIT(0))
#define BQ256XX_NTC_FAULT_HOT		(BIT(2) | BIT(1))

#define BQ256XX_NUM_WD_VAL	4
#define BQ256XX_WATCHDOG_MASK	GENMASK(5, 4)
#define BQ256XX_WATCHDOG_MAX	1600000
#define BQ256XX_WATCHDOG_DIS	0
#define BQ256XX_WDT_BIT_SHIFT	4

#define BQ256XX_REG_RST		BIT(7)

struct reg_default {
	unsigned int reg;
	unsigned int def;
};

/**
 * struct bq256xx_init_data -
 * @ichg: fast charge current
 * @iindpm: input current limit
 * @vbatreg: charge voltage
 * @iterm: termination current
 * @iprechg: precharge current
 * @vindpm: input voltage limit
 * @ichg_max: maximum fast charge current
 * @vbatreg_max: maximum charge voltage
 * @ts_ignore: TS_IGNORE flag
 */
struct bq256xx_init_data {
	uint32_t ichg;
	uint32_t iindpm;
	uint32_t vbatreg;
	uint32_t iterm;
	uint32_t iprechg;
	uint32_t vindpm;
	uint32_t ichg_max;
	uint32_t vbatreg_max;
	bool ts_ignore;
};

/**
 * struct bq256xx_state -
 * @vbus_stat: VBUS status according to BQ256XX_CHARGER_STATUS_0
 * @chrg_stat: charging status according to BQ256XX_CHARGER_STATUS_0
 * @online: PG status according to BQ256XX_CHARGER_STATUS_0
 *
 * @wdt_fault: watchdog fault according to BQ256XX_CHARGER_STATUS_1
 * @bat_fault: battery fault according to BQ256XX_CHARGER_STATUS_1
 * @chrg_fault: charging fault according to BQ256XX_CHARGER_STATUS_1
 * @ntc_fault: TS fault according to BQ256XX_CHARGER_STATUS_1
 */
struct bq256xx_state {
	uint8_t vbus_stat;
	uint8_t chrg_stat;
	bool online;

	uint8_t wdt_fault;
	uint8_t bat_fault;
	uint8_t chrg_fault;
	uint8_t ntc_fault;
};

enum bq256xx_id {
	BQ25600,
	BQ25600D,
	BQ25601,
	BQ25601D,
	BQ25618,
	BQ25619,
	BQ25611D,
};

/**
 * struct bq256xx_device -
 * @client: i2c client structure
 * @regmap: register map structure
 * @dev: device structure
 * @charger: power supply registered for the charger
 * @battery: power supply registered for the battery
 * @lock: mutex lock structure
 *
 * @usb2_phy: usb_phy identifier
 * @usb3_phy: usb_phy identifier
 * @usb_nb: notifier block
 * @usb_work: usb work queue
 * @usb_event: usb_event code
 *
 * @model_name: i2c name string
 *
 * @init_data: initialization data
 * @chip_info: device variant information
 * @state: device status and faults
 * @watchdog_timer: watchdog timer value in milliseconds
 */
struct bq256xx_device {
	i2c_inst_t *i2c;
	uint8_t dev_addr;
	char model_name[I2C_NAME_SIZE];
	struct bq256xx_init_data init_data;
	const struct bq256xx_chip_info *chip_info;
	struct bq256xx_state state;
	int watchdog_timer;
};

/**
 * struct bq256xx_chip_info -
 * @model_id: device instance
 *
 * @bq256xx_regmap_config: regmap configuration struct
 * @bq256xx_get_ichg: pointer to instance specific get_ichg function
 * @bq256xx_get_iindpm: pointer to instance specific get_iindpm function
 * @bq256xx_get_vbatreg: pointer to instance specific get_vbatreg function
 * @bq256xx_get_iterm: pointer to instance specific get_iterm function
 * @bq256xx_get_iprechg: pointer to instance specific get_iprechg function
 * @bq256xx_get_vindpm: pointer to instance specific get_vindpm function
 *
 * @bq256xx_set_ichg: pointer to instance specific set_ichg function
 * @bq256xx_set_iindpm: pointer to instance specific set_iindpm function
 * @bq256xx_set_vbatreg: pointer to instance specific set_vbatreg function
 * @bq256xx_set_iterm: pointer to instance specific set_iterm function
 * @bq256xx_set_iprechg: pointer to instance specific set_iprechg function
 * @bq256xx_set_vindpm: pointer to instance specific set_vindpm function
 * @bq256xx_set_charge_type: pointer to instance specific set_charge_type function
 * @bq256xx_set_ts_ignore: pointer to instance specific set_ts_ignore function
 *
 * @bq256xx_def_ichg: default ichg value in microamps
 * @bq256xx_def_iindpm: default iindpm value in microamps
 * @bq256xx_def_vbatreg: default vbatreg value in microvolts
 * @bq256xx_def_iterm: default iterm value in microamps
 * @bq256xx_def_iprechg: default iprechg value in microamps
 * @bq256xx_def_vindpm: default vindpm value in microvolts
 *
 * @bq256xx_max_ichg: maximum charge current in microamps
 * @bq256xx_max_vbatreg: maximum battery regulation voltage in microvolts
 *
 * @has_usb_detect: indicates whether device has BC1.2 detection
 */
typedef struct bq256xx_chip_info {
	int model_id;

	const struct regmap_config *bq256xx_regmap_config;

	int (*bq256xx_get_ichg)(struct bq256xx_device *bq);
	int (*bq256xx_get_iindpm)(struct bq256xx_device *bq);
	int (*bq256xx_get_vbatreg)(struct bq256xx_device *bq);
	int (*bq256xx_get_iterm)(struct bq256xx_device *bq);
	int (*bq256xx_get_iprechg)(struct bq256xx_device *bq);
	int (*bq256xx_get_vindpm)(struct bq256xx_device *bq);

	int (*bq256xx_set_ichg)(struct bq256xx_device *bq, int ichg);
	int (*bq256xx_set_iindpm)(struct bq256xx_device *bq, int iindpm);
	int (*bq256xx_set_vbatreg)(struct bq256xx_device *bq, int vbatreg);
	int (*bq256xx_set_iterm)(struct bq256xx_device *bq, int iterm);
	int (*bq256xx_set_iprechg)(struct bq256xx_device *bq, int iprechg);
	int (*bq256xx_set_vindpm)(struct bq256xx_device *bq, int vindpm);
	int (*bq256xx_set_charge_type)(struct bq256xx_device *bq, int type);
	int (*bq256xx_set_ts_ignore)(struct bq256xx_device *bq, bool ts_ignore);

	int bq256xx_def_ichg;
	int bq256xx_def_iindpm;
	int bq256xx_def_vbatreg;
	int bq256xx_def_iterm;
	int bq256xx_def_iprechg;
	int bq256xx_def_vindpm;

	int bq256xx_max_ichg;
	int bq256xx_max_vbatreg;

	bool has_usb_detect;
};

static int bq256xx_watchdog_time[BQ256XX_NUM_WD_VAL] = {
	0, 40000, 80000, 1600000
};

static const int bq25611d_vbatreg_values[] = {
	3494000, 3590000, 3686000, 3790000, 3894000, 3990000, 4090000, 4140000,
	4190000
};

static const int bq25618_619_vbatreg_values[] = {
	3504000, 3600000, 3696000, 3800000, 3904000, 4000000, 4100000, 4150000,
	4200000
};

static const int bq25618_619_ichg_values[] = {
	1290000, 1360000, 1430000, 1500000
};

int regmap_read(i2c_inst_t *i2cinst, uint8_t dev_addr, uint8_t reg_addr,
                unsigned int *val) {
  i2c_write_blocking(i2cinst, dev_addr, &reg_addr, 1, true);
  uint8_t buf;
  size_t nbytes = i2c_read_blocking(i2cinst, dev_addr, &buf, 1, false);
  *val = buf;
  if (nbytes == 1) return 0;
  return -EINVAL;
}

int regmap_update_bits(i2c_inst_t *i2cinst, uint8_t dev_addr, uint8_t reg_addr,
                       uint8_t mask, uint8_t val) {
  unsigned int orig;
  uint8_t tmp;
  regmap_read(i2cinst, dev_addr, reg_addr, &orig);
  tmp = orig & ~mask;
  tmp |= val & mask;
  if (tmp != orig) {
		uint8_t buf[2];
		buf[0] = reg_addr;
		buf[1] = tmp;
    size_t nbytes = i2c_write_blocking(i2cinst, dev_addr, buf, 2, false);
    if (nbytes == 1) return 0;
    return -EINVAL;
  }
  return 0;
}

static int bq256xx_array_parse(int array_size, int val, const int array[])
{
	int i = 0;

	if (val < array[i])
		return i - 1;

	if (val >= array[array_size - 1])
		return array_size - 1;

	for (i = 1; i < array_size; i++) {
		if (val == array[i])
			return i;

		if (val > array[i - 1] && val < array[i]) {
			if (val < array[i])
				return i - 1;
			else
				return i;
		}
	}
	return -EINVAL;
}

static struct reg_default bq2560x_reg_defs[] = {
	{BQ256XX_INPUT_CURRENT_LIMIT, 0x17},
	{BQ256XX_CHARGER_CONTROL_0, 0x1a},
	{BQ256XX_CHARGE_CURRENT_LIMIT, 0xa2},
	{BQ256XX_PRECHG_AND_TERM_CURR_LIM, 0x22},
	{BQ256XX_BATTERY_VOLTAGE_LIMIT, 0x58},
	{BQ256XX_CHARGER_CONTROL_1, 0x9f},
	{BQ256XX_CHARGER_CONTROL_2, 0x66},
	{BQ256XX_CHARGER_CONTROL_3, 0x4c},
};

static struct reg_default bq25611d_reg_defs[] = {
	{BQ256XX_INPUT_CURRENT_LIMIT, 0x17},
	{BQ256XX_CHARGER_CONTROL_0, 0x1a},
	{BQ256XX_CHARGE_CURRENT_LIMIT, 0x91},
	{BQ256XX_PRECHG_AND_TERM_CURR_LIM, 0x12},
	{BQ256XX_BATTERY_VOLTAGE_LIMIT, 0x40},
	{BQ256XX_CHARGER_CONTROL_1, 0x9e},
	{BQ256XX_CHARGER_CONTROL_2, 0xe6},
	{BQ256XX_CHARGER_CONTROL_3, 0x4c},
	{BQ256XX_PART_INFORMATION, 0x54},
	{BQ256XX_CHARGER_CONTROL_4, 0x75},
};

static struct reg_default bq25618_619_reg_defs[] = {
	{BQ256XX_INPUT_CURRENT_LIMIT, 0x17},
	{BQ256XX_CHARGER_CONTROL_0, 0x1a},
	{BQ256XX_CHARGE_CURRENT_LIMIT, 0x91},
	{BQ256XX_PRECHG_AND_TERM_CURR_LIM, 0x12},
	{BQ256XX_BATTERY_VOLTAGE_LIMIT, 0x40},
	{BQ256XX_CHARGER_CONTROL_1, 0x9e},
	{BQ256XX_CHARGER_CONTROL_2, 0xe6},
	{BQ256XX_CHARGER_CONTROL_3, 0x4c},
	{BQ256XX_PART_INFORMATION, 0x2c},
	{BQ256XX_CHARGER_CONTROL_4, 0x75},
};

static int bq256xx_get_state(struct bq256xx_device *bq,
				struct bq256xx_state *state)
{
	unsigned int charger_status_0;
	unsigned int charger_status_1;
	int ret;

	ret = regmap_read(bq->i2c, bq->dev_addr, BQ256XX_CHARGER_STATUS_0,
						&charger_status_0);
	if (ret)
		return ret;

	ret = regmap_read(bq->i2c, bq->dev_addr, BQ256XX_CHARGER_STATUS_1,
						&charger_status_1);
	if (ret)
		return ret;

	state->vbus_stat = charger_status_0 & BQ256XX_VBUS_STAT_MASK;
	state->chrg_stat = charger_status_0 & BQ256XX_CHRG_STAT_MASK;
	state->online = charger_status_0 & BQ256XX_PG_STAT_MASK;

	state->wdt_fault = charger_status_1 & BQ256XX_WDT_FAULT_MASK;
	state->bat_fault = charger_status_1 & BQ256XX_BAT_FAULT_MASK;
	state->chrg_fault = charger_status_1 & BQ256XX_CHRG_FAULT_MASK;
	state->ntc_fault = charger_status_1 & BQ256XX_NTC_FAULT_MASK;

	return 0;
}

static int bq256xx_set_charge_type(struct bq256xx_device *bq, int type)
{
	int chg_config = 0;

	return regmap_update_bits(bq->i2c, bq->dev_addr, BQ256XX_CHARGER_CONTROL_0,
				BQ256XX_CHG_CONFIG_MASK,
				(chg_config ? 1 : 0) << BQ256XX_CHG_CONFIG_BIT_SHIFT);
}

static int bq256xx_get_ichg_curr(struct bq256xx_device *bq)
{
	unsigned int charge_current_limit;
	unsigned int ichg_reg_code;
	int ret;

	ret = regmap_read(bq->i2c, bq->dev_addr, BQ256XX_CHARGE_CURRENT_LIMIT,
						&charge_current_limit);
	if (ret)
		return ret;

	ichg_reg_code = charge_current_limit & BQ256XX_ICHG_MASK;

	return ichg_reg_code * BQ256XX_ICHG_STEP_uA;
}

static int bq25618_619_get_ichg_curr(struct bq256xx_device *bq)
{
	unsigned int charge_current_limit;
	unsigned int ichg_reg_code;
	int ret;

	ret = regmap_read(bq->i2c, bq->dev_addr, BQ256XX_CHARGE_CURRENT_LIMIT,
						&charge_current_limit);
	if (ret)
		return ret;

	ichg_reg_code = charge_current_limit & BQ256XX_ICHG_MASK;

	if (ichg_reg_code < BQ25618_ICHG_THRESH)
		return ichg_reg_code * BQ25618_ICHG_STEP_uA;

	return bq25618_619_ichg_values[ichg_reg_code - BQ25618_ICHG_THRESH];
}

static int bq256xx_set_ichg_curr(struct bq256xx_device *bq, int ichg)
{
	unsigned int ichg_reg_code;
	int ichg_max = bq->init_data.ichg_max;

	ichg = CLAMP(ichg, BQ256XX_ICHG_MIN_uA, ichg_max);
	ichg_reg_code = ichg / BQ256XX_ICHG_STEP_uA;

	return regmap_update_bits(bq->i2c, bq->dev_addr, BQ256XX_CHARGE_CURRENT_LIMIT,
					BQ256XX_ICHG_MASK, ichg_reg_code);
}

static int bq25618_619_set_ichg_curr(struct bq256xx_device *bq, int ichg)
{
	int array_size = ARRAY_SIZE(bq25618_619_ichg_values);
	unsigned int ichg_reg_code;
	int ichg_max = bq->init_data.ichg_max;

	ichg = CLAMP(ichg, BQ25618_ICHG_MIN_uA, ichg_max);

	if (ichg <= BQ25618_ICHG_THRESH_uA) {
		ichg_reg_code = ichg / BQ25618_ICHG_STEP_uA;
	} else {
		ichg_reg_code = bq256xx_array_parse(array_size, ichg,
			bq25618_619_ichg_values) + BQ25618_ICHG_THRESH;
	}

	return regmap_update_bits(bq->i2c, bq->dev_addr, BQ256XX_CHARGE_CURRENT_LIMIT,
					BQ256XX_ICHG_MASK, ichg_reg_code);
}

static int bq25618_619_get_chrg_volt(struct bq256xx_device *bq)
{
	unsigned int battery_volt_lim;
	unsigned int vbatreg_reg_code;
	int ret;

	ret = regmap_read(bq->i2c, bq->dev_addr, BQ256XX_BATTERY_VOLTAGE_LIMIT,
							&battery_volt_lim);

	if (ret)
		return ret;

	vbatreg_reg_code = (battery_volt_lim & BQ256XX_VBATREG_MASK) >>
						BQ256XX_VBATREG_BIT_SHIFT;

	if (vbatreg_reg_code > BQ2561X_VBATREG_THRESH)
		return ((vbatreg_reg_code - BQ2561X_VBATREG_THRESH) *
					BQ2561X_VBATREG_STEP_uV) +
					BQ25618_VBATREG_THRESH_uV;

	return bq25618_619_vbatreg_values[vbatreg_reg_code];
}

static int bq25611d_get_chrg_volt(struct bq256xx_device *bq)
{
	unsigned int battery_volt_lim;
	unsigned int vbatreg_reg_code;
	int ret;

	ret = regmap_read(bq->i2c, bq->dev_addr, BQ256XX_BATTERY_VOLTAGE_LIMIT,
							&battery_volt_lim);
	if (ret)
		return ret;

	vbatreg_reg_code = (battery_volt_lim & BQ256XX_VBATREG_MASK) >>
						BQ256XX_VBATREG_BIT_SHIFT;

	if (vbatreg_reg_code > BQ2561X_VBATREG_THRESH)
		return ((vbatreg_reg_code - BQ2561X_VBATREG_THRESH) *
					BQ2561X_VBATREG_STEP_uV) +
					BQ25611D_VBATREG_THRESH_uV;

	return bq25611d_vbatreg_values[vbatreg_reg_code];
}

static int bq2560x_get_chrg_volt(struct bq256xx_device *bq)
{
	unsigned int battery_volt_lim;
	unsigned int vbatreg_reg_code;
	int ret;

	ret = regmap_read(bq->i2c, bq->dev_addr, BQ256XX_BATTERY_VOLTAGE_LIMIT,
							&battery_volt_lim);
	if (ret)
		return ret;

	vbatreg_reg_code = (battery_volt_lim & BQ256XX_VBATREG_MASK) >>
						BQ256XX_VBATREG_BIT_SHIFT;

	return (vbatreg_reg_code * BQ2560X_VBATREG_STEP_uV)
					+ BQ2560X_VBATREG_OFFSET_uV;
}

static int bq25601d_get_chrg_volt(struct bq256xx_device *bq)
{
	unsigned int battery_volt_lim;
	unsigned int vbatreg_reg_code;
	int ret;

	ret = regmap_read(bq->i2c, bq->dev_addr, BQ256XX_BATTERY_VOLTAGE_LIMIT,
							&battery_volt_lim);
	if (ret)
		return ret;

	vbatreg_reg_code = (battery_volt_lim & BQ256XX_VBATREG_MASK) >>
						BQ256XX_VBATREG_BIT_SHIFT;

	return (vbatreg_reg_code * BQ2560X_VBATREG_STEP_uV)
					+ BQ25601D_VBATREG_OFFSET_uV;
}

static int bq25618_619_set_chrg_volt(struct bq256xx_device *bq, int vbatreg)
{
	int array_size = ARRAY_SIZE(bq25618_619_vbatreg_values);
	unsigned int vbatreg_reg_code;
	int vbatreg_max = bq->init_data.vbatreg_max;

	vbatreg = CLAMP(vbatreg, BQ25618_VBATREG_MIN_uV, vbatreg_max);

	if (vbatreg > BQ25618_VBATREG_THRESH_uV)
		vbatreg_reg_code = ((vbatreg -
		BQ25618_VBATREG_THRESH_uV) /
		(BQ2561X_VBATREG_STEP_uV)) + BQ2561X_VBATREG_THRESH;
	else {
		vbatreg_reg_code = bq256xx_array_parse(array_size, vbatreg,
						bq25618_619_vbatreg_values);
	}

	return regmap_update_bits(bq->i2c, bq->dev_addr, BQ256XX_BATTERY_VOLTAGE_LIMIT,
				BQ256XX_VBATREG_MASK, vbatreg_reg_code <<
						BQ256XX_VBATREG_BIT_SHIFT);
}

static int bq25611d_set_chrg_volt(struct bq256xx_device *bq, int vbatreg)
{
	int array_size = ARRAY_SIZE(bq25611d_vbatreg_values);
	unsigned int vbatreg_reg_code;
	int vbatreg_max = bq->init_data.vbatreg_max;

	vbatreg = CLAMP(vbatreg, BQ25611D_VBATREG_MIN_uV, vbatreg_max);

	if (vbatreg > BQ25611D_VBATREG_THRESH_uV)
		vbatreg_reg_code = ((vbatreg -
		BQ25611D_VBATREG_THRESH_uV) /
		(BQ2561X_VBATREG_STEP_uV)) + BQ2561X_VBATREG_THRESH;
	else {
		vbatreg_reg_code = bq256xx_array_parse(array_size, vbatreg,
						bq25611d_vbatreg_values);
	}

	return regmap_update_bits(bq->i2c, bq->dev_addr, BQ256XX_BATTERY_VOLTAGE_LIMIT,
				BQ256XX_VBATREG_MASK, vbatreg_reg_code <<
						BQ256XX_VBATREG_BIT_SHIFT);
}

static int bq2560x_set_chrg_volt(struct bq256xx_device *bq, int vbatreg)
{
	unsigned int vbatreg_reg_code;
	int vbatreg_max = bq->init_data.vbatreg_max;

	vbatreg = CLAMP(vbatreg, BQ2560X_VBATREG_MIN_uV, vbatreg_max);

	vbatreg_reg_code = (vbatreg - BQ2560X_VBATREG_OFFSET_uV) /
						BQ2560X_VBATREG_STEP_uV;

	return regmap_update_bits(bq->i2c, bq->dev_addr, BQ256XX_BATTERY_VOLTAGE_LIMIT,
				BQ256XX_VBATREG_MASK, vbatreg_reg_code <<
						BQ256XX_VBATREG_BIT_SHIFT);
}

static int bq25601d_set_chrg_volt(struct bq256xx_device *bq, int vbatreg)
{
	unsigned int vbatreg_reg_code;
	int vbatreg_max = bq->init_data.vbatreg_max;

	vbatreg = CLAMP(vbatreg, BQ25601D_VBATREG_MIN_uV, vbatreg_max);

	vbatreg_reg_code = (vbatreg - BQ25601D_VBATREG_OFFSET_uV) /
						BQ2560X_VBATREG_STEP_uV;

	return regmap_update_bits(bq->i2c, bq->dev_addr, BQ256XX_BATTERY_VOLTAGE_LIMIT,
				BQ256XX_VBATREG_MASK, vbatreg_reg_code <<
						BQ256XX_VBATREG_BIT_SHIFT);
}

static int bq256xx_set_ts_ignore(struct bq256xx_device *bq, bool ts_ignore)
{
	return regmap_update_bits(bq->i2c, bq->dev_addr, BQ256XX_INPUT_CURRENT_LIMIT,
				BQ256XX_TS_IGNORE, (ts_ignore ? 1 : 0) << BQ256XX_TS_IGNORE_SHIFT);
}

static int bq256xx_get_prechrg_curr(struct bq256xx_device *bq)
{
	unsigned int prechg_and_term_curr_lim;
	unsigned int iprechg_reg_code;
	int ret;

	ret = regmap_read(bq->i2c, bq->dev_addr, BQ256XX_PRECHG_AND_TERM_CURR_LIM,
						&prechg_and_term_curr_lim);
	if (ret)
		return ret;

	iprechg_reg_code = (prechg_and_term_curr_lim & BQ256XX_IPRECHG_MASK)
						>> BQ256XX_IPRECHG_BIT_SHIFT;

	return (iprechg_reg_code * BQ256XX_IPRECHG_STEP_uA) +
						BQ256XX_IPRECHG_OFFSET_uA;
}

static int bq256xx_set_prechrg_curr(struct bq256xx_device *bq, int iprechg)
{
	unsigned int iprechg_reg_code;

	iprechg = CLAMP(iprechg, BQ256XX_IPRECHG_MIN_uA,
						BQ256XX_IPRECHG_MAX_uA);

	iprechg_reg_code = ((iprechg - BQ256XX_IPRECHG_OFFSET_uA) /
			BQ256XX_IPRECHG_STEP_uA) << BQ256XX_IPRECHG_BIT_SHIFT;

	return regmap_update_bits(bq->i2c, bq->dev_addr, BQ256XX_PRECHG_AND_TERM_CURR_LIM,
				BQ256XX_IPRECHG_MASK, iprechg_reg_code);
}

static int bq25618_619_get_prechrg_curr(struct bq256xx_device *bq)
{
	unsigned int prechg_and_term_curr_lim;
	unsigned int iprechg_reg_code;
	int ret;

	ret = regmap_read(bq->i2c, bq->dev_addr, BQ256XX_PRECHG_AND_TERM_CURR_LIM,
						&prechg_and_term_curr_lim);
	if (ret)
		return ret;

	iprechg_reg_code = (prechg_and_term_curr_lim & BQ256XX_IPRECHG_MASK)
						>> BQ256XX_IPRECHG_BIT_SHIFT;

	return (iprechg_reg_code * BQ25618_IPRECHG_STEP_uA) +
						BQ25618_IPRECHG_OFFSET_uA;
}

static int bq25618_619_set_prechrg_curr(struct bq256xx_device *bq, int iprechg)
{
	unsigned int iprechg_reg_code;

	iprechg = CLAMP(iprechg, BQ25618_IPRECHG_MIN_uA,
						BQ25618_IPRECHG_MAX_uA);

	iprechg_reg_code = ((iprechg - BQ25618_IPRECHG_OFFSET_uA) /
			BQ25618_IPRECHG_STEP_uA) << BQ256XX_IPRECHG_BIT_SHIFT;

	return regmap_update_bits(bq->i2c, bq->dev_addr, BQ256XX_PRECHG_AND_TERM_CURR_LIM,
				BQ256XX_IPRECHG_MASK, iprechg_reg_code);
}

static int bq256xx_get_term_curr(struct bq256xx_device *bq)
{
	unsigned int prechg_and_term_curr_lim;
	unsigned int iterm_reg_code;
	int ret;

	ret = regmap_read(bq->i2c, bq->dev_addr, BQ256XX_PRECHG_AND_TERM_CURR_LIM,
						&prechg_and_term_curr_lim);
	if (ret)
		return ret;

	iterm_reg_code = prechg_and_term_curr_lim & BQ256XX_ITERM_MASK;

	return (iterm_reg_code * BQ256XX_ITERM_STEP_uA) +
						BQ256XX_ITERM_OFFSET_uA;
}

static int bq256xx_set_term_curr(struct bq256xx_device *bq, int iterm)
{
	unsigned int iterm_reg_code;

	iterm = CLAMP(iterm, BQ256XX_ITERM_MIN_uA, BQ256XX_ITERM_MAX_uA);

	iterm_reg_code = (iterm - BQ256XX_ITERM_OFFSET_uA) /
							BQ256XX_ITERM_STEP_uA;

	return regmap_update_bits(bq->i2c, bq->dev_addr, BQ256XX_PRECHG_AND_TERM_CURR_LIM,
				BQ256XX_ITERM_MASK, iterm_reg_code);
}

static int bq25618_619_get_term_curr(struct bq256xx_device *bq)
{
	unsigned int prechg_and_term_curr_lim;
	unsigned int iterm_reg_code;
	int ret;

	ret = regmap_read(bq->i2c, bq->dev_addr, BQ256XX_PRECHG_AND_TERM_CURR_LIM,
						&prechg_and_term_curr_lim);
	if (ret)
		return ret;

	iterm_reg_code = prechg_and_term_curr_lim & BQ256XX_ITERM_MASK;

	return (iterm_reg_code * BQ25618_ITERM_STEP_uA) +
						BQ25618_ITERM_OFFSET_uA;
}

static int bq25618_619_set_term_curr(struct bq256xx_device *bq, int iterm)
{
	unsigned int iterm_reg_code;

	iterm = CLAMP(iterm, BQ25618_ITERM_MIN_uA, BQ25618_ITERM_MAX_uA);

	iterm_reg_code = (iterm - BQ25618_ITERM_OFFSET_uA) /
							BQ25618_ITERM_STEP_uA;

	return regmap_update_bits(bq->i2c, bq->dev_addr, BQ256XX_PRECHG_AND_TERM_CURR_LIM,
				BQ256XX_ITERM_MASK, iterm_reg_code);
}

static int bq256xx_get_input_volt_lim(struct bq256xx_device *bq)
{
	unsigned int charger_control_2;
	unsigned int vindpm_reg_code;
	int ret;

	ret = regmap_read(bq->i2c, bq->dev_addr, BQ256XX_CHARGER_CONTROL_2,
						&charger_control_2);
	if (ret)
		return ret;

	vindpm_reg_code = charger_control_2 & BQ256XX_VINDPM_MASK;

	return (vindpm_reg_code * BQ256XX_VINDPM_STEP_uV) +
						BQ256XX_VINDPM_OFFSET_uV;
}

static int bq256xx_set_input_volt_lim(struct bq256xx_device *bq, int vindpm)
{
	unsigned int vindpm_reg_code;

	vindpm = CLAMP(vindpm, BQ256XX_VINDPM_MIN_uV, BQ256XX_VINDPM_MAX_uV);

	vindpm_reg_code = (vindpm - BQ256XX_VINDPM_OFFSET_uV) /
						BQ256XX_VINDPM_STEP_uV;

	return regmap_update_bits(bq->i2c, bq->dev_addr, BQ256XX_CHARGER_CONTROL_2,
					BQ256XX_VINDPM_MASK, vindpm_reg_code);
}

static int bq256xx_get_input_curr_lim(struct bq256xx_device *bq)
{
	unsigned int input_current_limit;
	unsigned int iindpm_reg_code;
	int ret;

	ret = regmap_read(bq->i2c, bq->dev_addr, BQ256XX_INPUT_CURRENT_LIMIT,
						&input_current_limit);
	if (ret)
		return ret;

	iindpm_reg_code = input_current_limit & BQ256XX_IINDPM_MASK;

	return (iindpm_reg_code * BQ256XX_IINDPM_STEP_uA) +
						BQ256XX_IINDPM_OFFSET_uA;
}

static int bq256xx_set_input_curr_lim(struct bq256xx_device *bq, int iindpm)
{
	unsigned int iindpm_reg_code;

	iindpm = CLAMP(iindpm, BQ256XX_IINDPM_MIN_uA, BQ256XX_IINDPM_MAX_uA);

	iindpm_reg_code = (iindpm - BQ256XX_IINDPM_OFFSET_uA) /
							BQ256XX_IINDPM_STEP_uA;

	return regmap_update_bits(bq->i2c, bq->dev_addr, BQ256XX_INPUT_CURRENT_LIMIT,
					BQ256XX_IINDPM_MASK, iindpm_reg_code);
}

static void bq256xx_charger_reset(struct bq256xx_device *bq)
{
	regmap_update_bits(bq->i2c, bq->dev_addr, BQ256XX_PART_INFORMATION,
					BQ256XX_REG_RST, BQ256XX_REG_RST);
}

static int bq256xx_hw_init(struct bq256xx_device *bq)
{
	int wd_reg_val = BQ256XX_WATCHDOG_DIS;
	int ret = 0;
	int i;

	for (i = 0; i < BQ256XX_NUM_WD_VAL; i++) {
		if (bq->watchdog_timer == bq256xx_watchdog_time[i]) {
			wd_reg_val = i;
			break;
		}
		if (bq->watchdog_timer > bq256xx_watchdog_time[i] &&
		    bq->watchdog_timer < bq256xx_watchdog_time[i + 1])
			wd_reg_val = i;
	}
	ret = regmap_update_bits(bq->i2c, bq->dev_addr, BQ256XX_CHARGER_CONTROL_1,
				 BQ256XX_WATCHDOG_MASK, wd_reg_val <<
						BQ256XX_WDT_BIT_SHIFT);

	// ret = bq->chip_info->bq256xx_set_vindpm(bq, bq->init_data.vindpm);
	// if (ret)
	// 	return ret;

	// ret = bq->chip_info->bq256xx_set_iindpm(bq, bq->init_data.iindpm);
	// if (ret)
	// 	return ret;

	// ret = bq->chip_info->bq256xx_set_ichg(bq,
	// 			bq->chip_info->bq256xx_def_ichg);
	// if (ret)
	// 	return ret;

	// ret = bq->chip_info->bq256xx_set_iprechg(bq,
	// 			bat_info->precharge_current_ua);
	// if (ret)
	// 	return ret;

	// ret = bq->chip_info->bq256xx_set_vbatreg(bq,
	// 			bq->chip_info->bq256xx_def_vbatreg);
	// if (ret)
	// 	return ret;

	// ret = bq->chip_info->bq256xx_set_iterm(bq,
	// 			bat_info->charge_term_current_ua);
	// if (ret)
	// 	return ret;

	// if (bq->chip_info->bq256xx_set_ts_ignore) {
	// 	ret = bq->chip_info->bq256xx_set_ts_ignore(bq, bq->init_data.ts_ignore);
	// 	if (ret)
	// 		return ret;
	// }

	return 0;
}

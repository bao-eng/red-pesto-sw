/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/adc/voltage_divider.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifiers*/
#define LED_GREEN_NODE DT_ALIAS(led_green)
#define LED_RED_NODE DT_ALIAS(led_red)
#define VEML7700_LEFT_NODE DT_ALIAS(veml7700_left)
#define VEML7700_RIGHT_NODE DT_ALIAS(veml7700_right)
#define LIS2DH_NODE DT_NODELABEL(lis2dh)
#define PWM_LED_NODE DT_ALIAS(pwm_led0)
#define VBAT_DIVIDER_NODE DT_NODELABEL(vbatt)
#define SW0_NODE DT_ALIAS(sw0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(LED_GREEN_NODE, gpios);
static const struct gpio_dt_spec led_red = GPIO_DT_SPEC_GET(LED_RED_NODE, gpios);
const struct device *const veml7700_left = DEVICE_DT_GET(VEML7700_LEFT_NODE);
const struct device *const veml7700_right = DEVICE_DT_GET(VEML7700_RIGHT_NODE);
const struct device *const lis2dh = DEVICE_DT_GET(LIS2DH_NODE);
static const struct pwm_dt_spec pwm_led0 = PWM_DT_SPEC_GET(PWM_LED_NODE);
static const struct voltage_divider_dt_spec adc_vbat = VOLTAGE_DIVIDER_DT_SPEC_GET(VBAT_DIVIDER_NODE);
static const struct adc_dt_spec adc_cc1 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);
static const struct adc_dt_spec adc_cc2 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static struct gpio_callback button_cb_data;

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
}

int meas_adc_v(const struct adc_dt_spec *spec, int32_t *v)
{
	int ret;
	int32_t sample_buffer = 0;

	/* Structure defining an ADC sampling sequence */
	struct adc_sequence sequence = {
		.buffer = &sample_buffer,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(sample_buffer),
	};
	adc_sequence_init_dt(spec, &sequence);

	ret = adc_read_dt(spec, &sequence);
	if (ret != 0) {
		return ret;
	}
	*v = sample_buffer;
	ret = adc_raw_to_millivolts_dt(spec, v);
	if (ret != 0) {
		return ret;
	}
	return 0;
}

int meas_divider_v(const struct voltage_divider_dt_spec *spec, int32_t *v)
{
	int ret;
	meas_adc_v(&spec->port, v);
	ret = voltage_divider_scale_dt(spec, v);
	if (ret != 0) {
		return ret;
	}
	return 0;
}

int main(void)
{
	int ret;

	ret = pwm_set_dt(&pwm_led0, PWM_KHZ(20), PWM_KHZ(20)/20U);
		if (ret < 0) {
		return 0;
	}

	if (!gpio_is_ready_dt(&led_green) || !gpio_is_ready_dt(&led_red)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_ACTIVE);
	ret = gpio_pin_configure_dt(&led_red, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);

	ret = adc_channel_setup_dt(&adc_vbat.port);
	ret = adc_channel_setup_dt(&adc_cc1);
	ret = adc_channel_setup_dt(&adc_cc2);

	struct sensor_value val;
	struct sensor_value accel[3];
	int32_t bat_volt, cc1_volt, cc2_volt;
	while (1) {
		meas_divider_v(&adc_vbat, &bat_volt);
		printk("VBAT: %" PRId32 "mV\n", bat_volt);
		meas_adc_v(&adc_cc1, &cc1_volt);
		printk("CC1: %" PRId32 "mV\n", cc1_volt);
		meas_adc_v(&adc_cc2, &cc2_volt);
		printk("CC2: %" PRId32 "mV\n", cc2_volt);
		ret = sensor_sample_fetch(veml7700_left);
		ret = sensor_channel_get(veml7700_left, SENSOR_CHAN_LIGHT, &val);
		printk("VEML770-Left: %d\n", val.val1);
		ret = sensor_sample_fetch(veml7700_right);
		ret = sensor_channel_get(veml7700_right, SENSOR_CHAN_LIGHT, &val);
		printk("VEML770-Right: %d\n", val.val1);
		ret = sensor_sample_fetch(lis2dh);
		ret = sensor_channel_get(lis2dh, SENSOR_CHAN_ACCEL_XYZ, accel);
		printk("LIS2DH: %.2f %.2f %.2f\n", accel[0].val1 + accel[0].val2*0.000001,
			accel[1].val1 + accel[1].val2*0.000001,
			accel[2].val1 + accel[2].val2*0.000001);
		ret = gpio_pin_toggle_dt(&led_green);
		ret = gpio_pin_toggle_dt(&led_red);
		if (ret < 0) {
			return 0;
		}
		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}

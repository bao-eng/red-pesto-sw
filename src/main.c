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

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED_GREEN_NODE DT_ALIAS(led_green)
#define LED_RED_NODE DT_ALIAS(led_red)
#define VEML7700_LEFT_NODE DT_ALIAS(veml7700_left)
#define VEML7700_RIGHT_NODE DT_ALIAS(veml7700_right)
#define LIS2DH_NODE DT_NODELABEL(lis2dh)
#define PWM_LED_NODE DT_ALIAS(pwm_led0)

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

	struct sensor_value val;
	struct sensor_value accel[3];
	while (1) {
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

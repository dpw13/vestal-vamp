/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include "adc.h"
#include "dac.h"
#include "display.h"

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   750

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* TODO:
ADC init
DMA
DAC init
SPI to ioexp
*/

int main(void)
{
	int ret;
	bool led_state = true;

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	adc_init();
	dac_init();
	adc_start();
	display_init();
	display_text(1, "Hello World");

	k_msleep(100);

	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}

		led_state = !led_state;
		//printf("LED state: %s\n", led_state ? "ON" : "OFF");
		display_bar(3, 50);
		adc_sample();
		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}

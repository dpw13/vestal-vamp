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
#include "timer.h"
#include "fft_dma.h"

/* 1000 msec = 1 sec */
//#define SLEEP_TIME_MS   750
#define SLEEP_TIME_MS   5000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* TODO:
SPI to ioexp
FFT/IFFT
granular control
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

	/* Data init */
	fft_init();

	adc_init();
	dac_init();
	timer_init();
	display_init();
	display_text(1, "Hello World");

	adc_start();
	timer_start();

	k_msleep(100);
	//timer_stop();

	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}

		led_state = !led_state;
		//printf("LED state: %s\n", led_state ? "ON" : "OFF");
		display_bar(3, 50);
		timer_stats();
		adc_stats();
		dac_stats();
		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}

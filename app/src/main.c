/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "filter.h"
#include "adc.h"
#include "dac.h"
#include "display.h"
#include "io.h"
#include "timer.h"
#include "filter.h"
#include "fft_dma.h"
#include "ifft_dma.h"

#include "settings.h"
#include "ui.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define SLEEP_TIME_MS 250

/* The devicetree node identifier for the "led1" alias. */
#define LED_NODE DT_ALIAS(led1)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

/* TODO:
SPI to ioexp debug
FFT/IFFT re-enable time and pitch shift
granular control
FFT/IFFT optimization
*/

void ui_display(uint16_t frame);

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

	settings_init();
	io_init();

	/* Data init */
	fft_init();
	ifft_init();

	adc_init();
	dac_init();
	timer_init();
	display_init();

	k_msleep(100);

	filter_start();
	adc_start();
	timer_start();

	// k_msleep(100);
	// timer_stop();

	uint16_t i = 0;
	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}

		led_state = !led_state;

		ui_display(i);
		i++;

#if 0
		while (1) {
			volatile uint32_t *pin = (volatile uint32_t *)(0x90000000);
			volatile uint32_t *pout = (volatile uint32_t *)(0x90000010);
			for (int j=0;j<256;j++) {
				pout[j] = j;
				pin[j] = pout[256 - j];
			}
			k_msleep(1);
		}
#endif
		io_work();
		// timer_stats();
		// adc_stats();
		// dac_stats();
		//filter_stats();
		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}

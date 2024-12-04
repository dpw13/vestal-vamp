/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include "filter.h"
#include "adc.h"
#include "dac.h"
#include "display.h"
#include "io.h"
#include "timer.h"
#include "fft_dma.h"
#include "ifft_dma.h"

#include "settings.h"

#define SLEEP_TIME_MS 500

/* The devicetree node identifier for the "led1" alias. */
#define LED_NODE DT_ALIAS(led1)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

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

	settings_init();
	io_init();

	/* Data init */
	fft_init();
	ifft_init();

	adc_init();
	dac_init();
	timer_init();
	display_init();
	display_text(0, " Vestal");
	display_text(1, "     Vamp");
	// display_text(1, "+++Line 1");
	// display_text(2, "---Line 2");
	// display_text(3, "***Line 3");

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

		// printf("LED state: %s\n", led_state ? "ON" : "OFF");
		// snprintf(buf, 8, "%d", i);
		// display_text(1, buf);
		display_bar(2, (i & 0x100) ? 256 - (i & 0xFF) : (i & 0xFF));
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
		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}

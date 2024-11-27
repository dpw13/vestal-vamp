/*
 * Copyright (c) 2024 Argentum Systems Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>

#include <stdio.h>
#include <string.h>

LOG_MODULE_REGISTER(io_ser, LOG_LEVEL_INF);

struct patch_info {
	const uint8_t * const name;

	const struct device *rx_dev;
	struct ring_buf *rx_ring_buf;
	bool rx_error;
	bool rx_overflow;

	const struct device *tx_dev;
};

#define DEV_IOEXP   DEVICE_DT_GET(DT_CHOSEN(ioexp_uart))

#define RING_BUF_SIZE 64

RING_BUF_DECLARE(rb_ioexp, RING_BUF_SIZE);
struct patch_info patch_o2c = {
	.name = "o2c",

	.rx_dev = DEV_IOEXP,
	.rx_ring_buf = &rb_ioexp,
	.rx_error = false,
	.rx_overflow = false,
};

static void uart_cb(const struct device *dev, void *ctx)
{
	struct patch_info *patch = (struct patch_info *)ctx;
	int ret;
	uint8_t *buf;
	uint32_t len;

	while (uart_irq_update(patch->rx_dev) > 0) {
		ret = uart_irq_rx_ready(patch->rx_dev);
		if (ret < 0) {
			patch->rx_error = true;
		}
		if (ret <= 0) {
			break;
		}

		len = ring_buf_put_claim(patch->rx_ring_buf, &buf, RING_BUF_SIZE);
		if (len == 0) {
			/* no space for Rx, disable the IRQ */
			uart_irq_rx_disable(patch->rx_dev);
			patch->rx_overflow = true;
			break;
		}

		ret = uart_fifo_read(patch->rx_dev, buf, len);
		if (ret < 0) {
			patch->rx_error = true;
		}
		if (ret <= 0) {
			break;
		}
		len = ret;

		ret = ring_buf_put_finish(patch->rx_ring_buf, len);
		if (ret != 0) {
			patch->rx_error = true;
			break;
		}
	}
}

static void passthrough(struct patch_info *patch)
{
	int ret;
	uint8_t *buf;
	uint32_t len;

	if (patch->rx_error) {
		LOG_ERR("<<%s: Rx Error!>>", patch->name);
		patch->rx_error = false;
	}

	if (patch->rx_overflow) {
		LOG_ERR("<<%s: Rx Overflow!>>", patch->name);
		patch->rx_overflow = false;
	}

	len = ring_buf_get_claim(patch->rx_ring_buf, &buf, RING_BUF_SIZE);
	if (len == 0) {
		goto done;
	}

        LOG_INF("%s", buf);

	ret = ring_buf_get_finish(patch->rx_ring_buf, len);
	if (ret < 0) {
		goto error;
	}

done:
	uart_irq_rx_enable(patch->rx_dev);
	return;

error:
	LOG_ERR("<<%s: Tx Error!>>", patch->name);
}

int io_serial_init(void) {
	LOG_INF("IO expander device %s", patch_o2c.rx_dev->name);

	return uart_irq_callback_user_data_set(patch_o2c.rx_dev, uart_cb, (void *)&patch_o2c);

        return 0;
}

int io_serial_work(void) {
        passthrough(&patch_o2c);

	return 0;
}

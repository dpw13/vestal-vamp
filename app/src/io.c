/*
 * Copyright (c) 2024 Argentum Systems Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>

#include <stdio.h>
#include <string.h>

LOG_MODULE_REGISTER(io_exp, LOG_LEVEL_INF);

struct patch_info {
	const uint8_t *const name;

	const struct device *rx_dev;
	struct ring_buf *rx_ring_buf;
	bool rx_error;
	bool rx_overflow;

	const struct device *tx_dev;
};

#define DEV_IOEXP_UART DEVICE_DT_GET(DT_CHOSEN(ioexp_uart))
#define DEV_IOEXP_SPI  DT_CHOSEN(ioexp_spi)

static const struct device *const io_spi_dev = DEVICE_DT_GET(DT_BUS(DEV_IOEXP_SPI));
static struct spi_config io_spi_cfg =
	SPI_CONFIG_DT(DEV_IOEXP_SPI, SPI_OP_MODE_MASTER | SPI_MODE_CPHA | SPI_WORD_SET(8), 0);

#define RING_BUF_SIZE 512

RING_BUF_DECLARE(rb_ioexp, RING_BUF_SIZE);
struct patch_info patch_o2c = {
	.name = "o2c",

	.rx_dev = DEV_IOEXP_UART,
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
	buf[len] = 0;

	LOG_INF("serial: %s", buf);

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

#define SPI_MAX_MSG_LEN 17

/* The SPI message is padded with a leading zero. We want to ensure that
 * the actual data is aligned, so create a struct with an appropriate
 * alignment.
 */
struct spi_msg {
	uint8_t pad[3];
	uint8_t blank;
	uint32_t data[4];
};

static struct spi_msg rxmsg;

static struct spi_buf rx = {
	.buf = (uint8_t *)&rxmsg.blank,
	.len = SPI_MAX_MSG_LEN,
};
const static struct spi_buf_set rx_bufs = {
	.buffers = &rx,
	.count = 1,
};

static uint8_t txmsg[SPI_MAX_MSG_LEN] = {0xA5};
static struct spi_buf tx = {
	.buf = &txmsg[0],
	.len = SPI_MAX_MSG_LEN,
};
const static struct spi_buf_set tx_bufs = {
	.buffers = &tx,
	.count = 1,
};

static int get_io(void)
{
	int ret = spi_transceive(io_spi_dev, &io_spi_cfg, &tx_bufs, &rx_bufs);

	if (ret < 0) {
		LOG_ERR("SPI transceive error: %d", ret);
	} else {
		LOG_INF("%08x %08x %08x %08x", rxmsg.data[0], rxmsg.data[1], rxmsg.data[2], rxmsg.data[3]);
	}

	return ret;
}

int io_init(void)
{
	LOG_INF("IO expander serial %s", patch_o2c.rx_dev->name);
	LOG_INF("IO expander spi %s", io_spi_dev->name);

	return uart_irq_callback_user_data_set(patch_o2c.rx_dev, uart_cb, (void *)&patch_o2c);

	return 0;
}

int io_work(void)
{
	passthrough(&patch_o2c);
	get_io();

	return 0;
}

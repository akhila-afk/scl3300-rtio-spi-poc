/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include <errno.h>
#include <stdint.h>
#include <string.h>

LOG_MODULE_REGISTER(scl3300_poc, LOG_LEVEL_INF);

#define SCL3300_FRAME_SIZE 4U

#define SCL3300_CMD_WHOAMI   0x40000091U
#define SCL3300_CMD_SW_RESET 0xB4002098U
#define SCL3300_CMD_MODE1    0xB400001FU
#define SCL3300_CMD_ANG_CTRL 0xB0001F6FU
#define SCL3300_CMD_STATUS   0x180000E5U

enum scl3300_init_state {
	STATE_WHOAMI_FRAME,
	STATE_RESET_FRAME,
	STATE_MODE1_FRAME,
	STATE_ANGLE_FRAME,
	STATE_STATUS1_FRAME,
	STATE_STATUS2_FRAME,
	STATE_STATUS3_FRAME,
	STATE_DONE,
};

RTIO_DEFINE(scl3300_rtio, 2, 2);

SPI_DT_IODEV_DEFINE(scl3300_spi_iodev,
		    DT_NODELABEL(scl3300),
		    SPI_WORD_SET(8) | SPI_TRANSFER_MSB);

static uint8_t g_tx_buf[SCL3300_FRAME_SIZE];
static uint8_t g_rx_buf[SCL3300_FRAME_SIZE];
static enum scl3300_init_state g_init_state;

static void scl3300_prepare_command(uint32_t command)
{
	sys_put_be32(command, g_tx_buf);
}

static int scl3300_decode(const uint8_t *data, size_t len)
{
	uint8_t rs;

	if (len < SCL3300_FRAME_SIZE) {
		return -EINVAL;
	}

	LOG_INF("SCL3300 RX: %02x %02x %02x %02x",
		data[0], data[1], data[2], data[3]);

	return 0;
}

static int scl3300_spi_submit(struct rtio *ctx)
{
	struct rtio_sqe *sqe;

	/* Required minimum inter-frame delay */
	k_busy_wait(10);

	sqe = rtio_sqe_acquire(ctx);
	if (sqe == NULL) {
		LOG_ERR("No RTIO SQE available");
		return -ENOMEM;
	}

	rtio_sqe_prep_transceive(sqe,
				 &scl3300_spi_iodev,
				 RTIO_PRIO_NORM,
				 g_tx_buf,
				 g_rx_buf,
				 sizeof(g_tx_buf),
				 NULL);

	rtio_submit(ctx, 0);

	return 0;
}

static int scl3300_send_command(struct rtio *ctx, uint32_t command)
{
	scl3300_prepare_command(command);
	return scl3300_spi_submit(ctx);
}

static void scl3300_cqe_callback(struct rtio *r, void *user_data)
{
	struct rtio_cqe *cqe;
	int ret = 0;

	ARG_UNUSED(user_data);

	cqe = rtio_cqe_consume(r);
	if (cqe == NULL) {
		return;
	}

	if (cqe->result < 0) {
		LOG_ERR("SPI transfer failed: %d", cqe->result);
		rtio_cqe_release(r, cqe);
		return;
	}

	rtio_cqe_release(r, cqe);

	switch (g_init_state) {
	case STATE_WHOAMI_FRAME:
		g_init_state = STATE_RESET_FRAME;
		ret = scl3300_send_command(r, SCL3300_CMD_SW_RESET);
		break;

	case STATE_RESET_FRAME:
		LOG_INF("WHOAMI response:");
		(void)scl3300_decode(g_rx_buf, sizeof(g_rx_buf));

		k_busy_wait(1000);

		g_init_state = STATE_MODE1_FRAME;
		ret = scl3300_send_command(r, SCL3300_CMD_MODE1);
		break;

	case STATE_MODE1_FRAME:
		g_init_state = STATE_ANGLE_FRAME;
		ret = scl3300_send_command(r, SCL3300_CMD_ANG_CTRL);
		break;

	case STATE_ANGLE_FRAME:
		k_busy_wait(25000);

		g_init_state = STATE_STATUS1_FRAME;
		ret = scl3300_send_command(r, SCL3300_CMD_STATUS);
		break;

	case STATE_STATUS1_FRAME:
		g_init_state = STATE_STATUS2_FRAME;
		ret = scl3300_send_command(r, SCL3300_CMD_STATUS);
		break;

	case STATE_STATUS2_FRAME:
		LOG_INF("STATUS response before clear verification:");
		(void)scl3300_decode(g_rx_buf, sizeof(g_rx_buf));

		g_init_state = STATE_STATUS3_FRAME;
		ret = scl3300_send_command(r, SCL3300_CMD_STATUS);
		break;

	case STATE_STATUS3_FRAME:
		LOG_INF("STATUS response after clear:");
		(void)scl3300_decode(g_rx_buf, sizeof(g_rx_buf));

		g_init_state = STATE_DONE;
		LOG_INF("Initialization sequence completed");
		break;

	case STATE_DONE:
	default:
		break;
	}

	if (ret < 0) {
		LOG_ERR("Failed to submit next command: %d", ret);
	}
}

int main(void)
{
	int ret;

	LOG_INF("SCL3300 WHOAMI + initialization POC");

	ret = rtio_set_cqe_callback(&scl3300_rtio,
				    scl3300_cqe_callback,
				    NULL);
	if (ret < 0) {
		LOG_ERR("Failed to register RTIO callback: %d", ret);
		return ret;
	}

	/* Start by reading the device ID */
	g_init_state = STATE_WHOAMI_FRAME;

	ret = scl3300_send_command(&scl3300_rtio, SCL3300_CMD_WHOAMI);
	if (ret < 0) {
		LOG_ERR("WHOAMI submission failed: %d", ret);
		return ret;
	}

	while (1) {
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
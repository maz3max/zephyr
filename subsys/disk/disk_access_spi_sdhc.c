/*
 * Copyright (c) 2017 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>

LOG_MODULE_REGISTER(sdhc_spi, CONFIG_DISK_LOG_LEVEL);

#include <disk/disk_access.h>
#include <drivers/gpio.h>
#include <sys/byteorder.h>
#include <drivers/spi.h>
#include <sys/crc.h>
#include "disk_access_sdhc.h"

/* Clock speed used during initialisation */
#define SDHC_SPI_INITIAL_SPEED 400000
/* Clock speed used after initialisation */
#define SDHC_SPI_SPEED 4000000

#ifndef DT_INST_0_ZEPHYR_MMC_SPI_SLOT_LABEL
#warning NO SDHC slot specified on board
#else
struct sdhc_spi_data {
	struct device *spi;
	struct spi_config cfg;
	struct device *cs;
	u32_t pin;

	u32_t sector_count;
	u8_t status;
	int trace_dir;
};

DEVICE_DECLARE(sdhc_spi_0);

/* Traces card traffic for LOG_LEVEL_DBG */
static int sdhc_spi_trace(struct sdhc_spi_data *data, int dir, int err,
		      const u8_t *buf, int len)
{
#if LOG_LEVEL >= LOG_LEVEL_DBG
	if (err != 0) {
		printk("(err=%d)", err);
		data->trace_dir = 0;
	}

	if (dir != data->trace_dir) {
		data->trace_dir = dir;

		printk("\n");

		if (dir == 1) {
			printk(">>");
		} else if (dir == -1) {
			printk("<<");
		}
	}

	for (; len != 0; len--) {
		printk(" %x", *buf++);
	}
#endif
	return err;
}

/* Asserts or deasserts chip select */
static void sdhc_spi_set_cs(struct sdhc_spi_data *data, int value)
{
	gpio_pin_write(data->cs, data->pin, value);
}

/* Receives a fixed number of bytes */
static int sdhc_spi_rx_bytes(struct sdhc_spi_data *data, u8_t *buf, int len)
{
	struct spi_buf tx_bufs[] = {
		{
			.buf = (u8_t *)sdhc_ones,
			.len = len
		}
	};

	const struct spi_buf_set tx = {
		.buffers = tx_bufs,
		.count = 1,
	};

	struct spi_buf rx_bufs[] = {
		{
			.buf = buf,
			.len = len
		}
	};

	const struct spi_buf_set rx = {
		.buffers = rx_bufs,
		.count = 1,
	};

	return sdhc_spi_trace(data, -1,
			  spi_transceive(data->spi, &data->cfg, &tx, &rx),
			  buf, len);
}

/* Receives and returns a single byte */
static int sdhc_spi_rx_u8(struct sdhc_spi_data *data)
{
	u8_t buf[1];
	int err = sdhc_spi_rx_bytes(data, buf, sizeof(buf));

	if (err != 0) {
		return err;
	}

	return buf[0];
}

/* Transmits a block of bytes */
static int sdhc_spi_tx(struct sdhc_spi_data *data, const u8_t *buf, int len)
{
	struct spi_buf spi_bufs[] = {
		{
			.buf = (u8_t *)buf,
			.len = len
		}
	};

	const struct spi_buf_set tx = {
		.buffers = spi_bufs,
		.count = 1
	};

	return sdhc_spi_trace(data, 1,
			spi_write(data->spi, &data->cfg, &tx), buf,
			len);
}

/* Transmits the command and payload */
static int sdhc_spi_tx_cmd(struct sdhc_spi_data *data, u8_t cmd, u32_t payload)
{
	u8_t buf[SDHC_CMD_SIZE];

	LOG_DBG("cmd%d payload=%u", cmd, payload);
	sdhc_spi_trace(data, 0, 0, NULL, 0);

	/* Encode the command */
	buf[0] = SDHC_TX | (cmd & ~SDHC_START);
	sys_put_be32(payload, &buf[1]);
	buf[SDHC_CMD_BODY_SIZE] = crc7_be(0, buf, SDHC_CMD_BODY_SIZE);

	return sdhc_spi_tx(data, buf, sizeof(buf));
}

/* Reads until anything but `discard` is received */
static int sdhc_spi_skip(struct sdhc_spi_data *data, int discard)
{
	int err;
	struct sdhc_retry retry;

	sdhc_retry_init(&retry, SDHC_READY_TIMEOUT, 0);

	do {
		err = sdhc_spi_rx_u8(data);
		if (err != discard) {
			return err;
		}
	} while (sdhc_retry_ok(&retry));

	LOG_WRN("Timeout while waiting for !%d", discard);
	return -ETIMEDOUT;
}

/* Reads until the first byte in a response is received */
static int sdhc_spi_skip_until_start(struct sdhc_spi_data *data)
{
	struct sdhc_retry retry;
	int status;

	sdhc_retry_init(&retry, SDHC_READY_TIMEOUT, 0);

	do {
		status = sdhc_spi_rx_u8(data);
		if (status < 0) {
			return status;
		}

		if ((status & SDHC_START) == 0) {
			return status;
		}
	} while (sdhc_retry_ok(&retry));

	return -ETIMEDOUT;
}

/* Reads until the bus goes high */
static int sdhc_spi_skip_until_ready(struct sdhc_spi_data *data)
{
	struct sdhc_retry retry;
	int status;

	sdhc_retry_init(&retry, SDHC_READY_TIMEOUT, 0);

	do {
		status = sdhc_spi_rx_u8(data);
		if (status < 0) {
			return status;
		}

		if (status == 0) {
			/* Card is still busy */
			continue;
		}

		if (status == 0xFF) {
			return 0;
		}

		/* Got something else.	Some cards release MISO part
		 * way through the transfer.  Read another and see if
		 * MISO went high.
		 */
		status = sdhc_spi_rx_u8(data);
		if (status < 0) {
			return status;
		}

		if (status == 0xFF) {
			return 0;
		}

		return -EPROTO;
	} while (sdhc_retry_ok(&retry));

	return -ETIMEDOUT;
}

/* Sends a command and returns the received R1 status code */
static int sdhc_spi_cmd_r1_raw(struct sdhc_spi_data *data,
	u8_t cmd, u32_t payload)
{
	int err;

	err = sdhc_spi_tx_cmd(data, cmd, payload);
	if (err != 0) {
		return err;
	}

	err = sdhc_spi_skip_until_start(data);

	/* Ensure there's a idle byte between commands */
	sdhc_spi_rx_u8(data);

	return err;
}

/* Sends a command and returns the mapped error code */
static int sdhc_spi_cmd_r1(struct sdhc_spi_data *data,
	u8_t cmd, uint32_t payload)
{
	return sdhc_map_r1_status(sdhc_spi_cmd_r1_raw(data, cmd, payload));
}

/* Sends a command in idle mode returns the mapped error code */
static int sdhc_spi_cmd_r1_idle(struct sdhc_spi_data *data, u8_t cmd,
			    uint32_t payload)
{
	return sdhc_map_r1_idle_status(sdhc_spi_cmd_r1_raw(data, cmd, payload));
}

/* Sends a command and returns the received multi-byte R2 status code */
static int sdhc_spi_cmd_r2(struct sdhc_spi_data *data,
	u8_t cmd, uint32_t payload)
{
	int err;
	int r1;
	int r2;

	err = sdhc_spi_tx_cmd(data, cmd, payload);
	if (err != 0) {
		return err;
	}

	r1 = sdhc_map_r1_status(sdhc_spi_skip_until_start(data));
	/* Always read the rest of the reply */
	r2 = sdhc_spi_rx_u8(data);

	/* Ensure there's a idle byte between commands */
	sdhc_spi_rx_u8(data);

	if (r1 < 0) {
		return r1;
	}

	return r2;
}

/* Sends a command and returns the received multi-byte status code */
static int sdhc_spi_cmd_r37_raw(struct sdhc_spi_data *data,
	u8_t cmd, u32_t payload, u32_t *reply)
{
	int err;
	int status;
	u8_t buf[sizeof(*reply)];

	err = sdhc_spi_tx_cmd(data, cmd, payload);
	if (err != 0) {
		return err;
	}

	status = sdhc_spi_skip_until_start(data);

	/* Always read the rest of the reply */
	err = sdhc_spi_rx_bytes(data, buf, sizeof(buf));
	*reply = sys_get_be32(buf);

	/* Ensure there's a idle byte between commands */
	sdhc_spi_rx_u8(data);

	if (err != 0) {
		return err;
	}

	return status;
}

/* Sends a command in idle mode returns the mapped error code */
static int sdhc_spi_cmd_r7_idle(struct sdhc_spi_data *data,
	u8_t cmd, u32_t payload, u32_t *reply)
{
	return sdhc_map_r1_idle_status(
		sdhc_spi_cmd_r37_raw(data, cmd, payload, reply));
}

/* Sends a command and returns the received multi-byte R3 error code */
static int sdhc_spi_cmd_r3(struct sdhc_spi_data *data,
	u8_t cmd, uint32_t payload, u32_t *reply)
{
	return sdhc_map_r1_status(
		sdhc_spi_cmd_r37_raw(data, cmd, payload, reply));
}

/* Receives a SDHC data block */
static int sdhc_spi_rx_block(struct sdhc_spi_data *data,
	u8_t *buf, int len)
{
	int err;
	int token;
	int i;
	/* Note the one extra byte to ensure there's an idle byte
	 * between commands.
	 */
	u8_t crc[SDHC_CRC16_SIZE + 1];

	token = sdhc_spi_skip(data, 0xFF);
	if (token < 0) {
		return token;
	}

	if (token != SDHC_TOKEN_SINGLE) {
		/* No start token */
		return -EIO;
	}

	/* Read the data in batches */
	for (i = 0; i < len; i += sizeof(sdhc_ones)) {
		int remain = MIN(sizeof(sdhc_ones), len - i);

		struct spi_buf tx_bufs[] = {
			{
				.buf = (u8_t *)sdhc_ones,
				.len = remain
			}
		};

		const struct spi_buf_set tx = {
			.buffers = tx_bufs,
			.count = 1,
		};

		struct spi_buf rx_bufs[] = {
			{
				.buf = &buf[i],
				.len = remain
			}
		};

		const struct spi_buf_set rx = {
			.buffers = rx_bufs,
			.count = 1,
		};

		err = sdhc_spi_trace(data, -1,
				spi_transceive(data->spi, &data->cfg,
				&tx, &rx),
				&buf[i], remain);
		if (err != 0) {
			return err;
		}
	}

	err = sdhc_spi_rx_bytes(data, crc, sizeof(crc));
	if (err != 0) {
		return err;
	}

	if (sys_get_be16(crc) != crc16_itu_t(0, buf, len)) {
		/* Bad CRC */
		return -EILSEQ;
	}

	return 0;
}

/* Transmits a SDHC data block */
static int sdhc_spi_tx_block(struct sdhc_spi_data *data,
	u8_t *send, int len)
{
	u8_t buf[SDHC_CRC16_SIZE];
	int err;

	/* Start the block */
	buf[0] = SDHC_TOKEN_SINGLE;
	err = sdhc_spi_tx(data, buf, 1);
	if (err != 0) {
		return err;
	}

	/* Write the payload */
	err = sdhc_spi_tx(data, send, len);
	if (err != 0) {
		return err;
	}

	/* Build and write the trailing CRC */
	sys_put_be16(crc16_itu_t(0, send, len), buf);

	err = sdhc_spi_tx(data, buf, sizeof(buf));
	if (err != 0) {
		return err;
	}

	return sdhc_map_data_status(sdhc_spi_rx_u8(data));
}

static int sdhc_spi_recover(struct sdhc_spi_data *data)
{
	/* TODO(nzmichaelh): implement */
	return sdhc_spi_cmd_r1(data, SDHC_SEND_STATUS, 0);
}

/* Attempts to return the card to idle mode */
static int sdhc_spi_go_idle(struct sdhc_spi_data *data)
{
	sdhc_spi_set_cs(data, 1);

	/* Write the initial >= 74 clocks */
	sdhc_spi_tx(data, sdhc_ones, 10);

	sdhc_spi_set_cs(data, 0);

	return sdhc_spi_cmd_r1_idle(data, SDHC_GO_IDLE_STATE, 0);
}

/* Checks the supported host volatage and basic protocol */
static int sdhc_spi_check_card(struct sdhc_spi_data *data)
{
	u32_t cond;
	int err;

	/* Check that the current voltage is supported */
	err = sdhc_spi_cmd_r7_idle(data, SDHC_SEND_IF_COND,
			       SDHC_VHS_3V3 | SDHC_CHECK, &cond);
	if (err != 0) {
		return err;
	}

	if ((cond & 0xFF) != SDHC_CHECK) {
		/* Card returned a different check pattern */
		return -ENOENT;
	}

	if ((cond & SDHC_VHS_MASK) != SDHC_VHS_3V3) {
		/* Card doesn't support this voltage */
		return -ENOTSUP;
	}

	return 0;
}

/* Detect and initialise the card */
static int sdhc_spi_detect(struct sdhc_spi_data *data)
{
	int err;
	u32_t ocr;
	struct sdhc_retry retry;
	u8_t structure;
	u32_t csize;
	u8_t buf[SDHC_CSD_SIZE];

	data->cfg.frequency = SDHC_SPI_INITIAL_SPEED;
	data->status = DISK_STATUS_UNINIT;

	sdhc_retry_init(&retry, SDHC_INIT_TIMEOUT, SDHC_RETRY_DELAY);

	/* Synchronise with the card by sending it to idle */
	do {
		err = sdhc_spi_go_idle(data);
		if (err == 0) {
			err = sdhc_spi_check_card(data);
			if (err == 0) {
				break;
			}
		}

		if (!sdhc_retry_ok(&retry)) {
			return -ENOENT;
		}
	} while (true);

	/* Enable CRC mode */
	err = sdhc_spi_cmd_r1_idle(data, SDHC_CRC_ON_OFF, 1);
	if (err != 0) {
		return err;
	}

	/* Wait for the card to leave idle state */
	do {
		sdhc_spi_cmd_r1_raw(data, SDHC_APP_CMD, 0);

		err = sdhc_spi_cmd_r1(data, SDHC_SEND_OP_COND, SDHC_HCS);
		if (err == 0) {
			break;
		}
	} while (sdhc_retry_ok(&retry));

	if (err != 0) {
		/* Card never exited idle */
		return -ETIMEDOUT;
	}

	/* Read OCR and confirm this is a SDHC card */
	err = sdhc_spi_cmd_r3(data, SDHC_READ_OCR, 0, &ocr);
	if (err != 0) {
		return err;
	}

	if ((ocr & SDHC_CCS) == 0U) {
		/* A 'SDSC' card */
		return -ENOTSUP;
	}

	/* Read the CSD */
	err = sdhc_spi_cmd_r1(data, SDHC_SEND_CSD, 0);
	if (err != 0) {
		return err;
	}

	err = sdhc_spi_rx_block(data, buf, sizeof(buf));
	if (err != 0) {
		return err;
	}

	/* Bits 126..127 are the structure version */
	structure = (buf[0] >> 6);
	if (structure != SDHC_CSD_V2) {
		/* Unsupported CSD format */
		return -ENOTSUP;
	}

	/* Bits 48..69 are the capacity of the card in 512 KiB units, minus 1.
	 */
	csize = sys_get_be32(&buf[6]) & ((1 << 22) - 1);
	if (csize < 4112) {
		/* Invalid capacity according to section 5.3.3 */
		return -ENOTSUP;
	}

	data->sector_count = (csize + 1) *
		(512 * 1024 / SDMMC_DEFAULT_BLOCK_SIZE);

	LOG_INF("Found a ~%u MiB SDHC card.",
		data->sector_count / (1024 * 1024 / SDMMC_DEFAULT_BLOCK_SIZE));

	/* Read the CID */
	err = sdhc_spi_cmd_r1(data, SDHC_SEND_CID, 0);
	if (err != 0) {
		return err;
	}

	err = sdhc_spi_rx_block(data, buf, sizeof(buf));
	if (err != 0) {
		return err;
	}

	LOG_INF("Manufacturer ID=%d OEM='%c%c' Name='%c%c%c%c%c' "
		"Revision=0x%x Serial=0x%x",
		buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6],
		buf[7], buf[8], sys_get_be32(&buf[9]));

	/* Initilisation complete */
	data->cfg.frequency = SDHC_SPI_SPEED;
	data->status = DISK_STATUS_OK;

	return 0;
}

static int sdhc_spi_read(struct sdhc_spi_data *data,
	u8_t *buf, u32_t sector, u32_t count)
{
	int err;

	err = sdhc_map_disk_status(data->status);
	if (err != 0) {
		return err;
	}

	sdhc_spi_set_cs(data, 0);

	/* Send the start read command */
	err = sdhc_spi_cmd_r1(data, SDHC_READ_MULTIPLE_BLOCK, sector);
	if (err != 0) {
		goto error;
	}

	/* Read the sectors */
	for (; count != 0U; count--) {
		err = sdhc_spi_rx_block(data, buf, SDMMC_DEFAULT_BLOCK_SIZE);
		if (err != 0) {
			goto error;
		}

		buf += SDMMC_DEFAULT_BLOCK_SIZE;
	}

	/* Ignore the error as STOP_TRANSMISSION always returns 0x7F */
	sdhc_spi_cmd_r1(data, SDHC_STOP_TRANSMISSION, 0);

	/* Wait until the card becomes ready */
	err = sdhc_spi_skip_until_ready(data);

error:
	sdhc_spi_set_cs(data, 1);

	return err;
}

static int sdhc_spi_write(struct sdhc_spi_data *data,
	const u8_t *buf, u32_t sector, u32_t count)
{
	int err;

	err = sdhc_map_disk_status(data->status);
	if (err != 0) {
		return err;
	}

	sdhc_spi_set_cs(data, 0);

	/* Write the blocks one-by-one */
	for (; count != 0U; count--) {
		err = sdhc_spi_cmd_r1(data, SDHC_WRITE_BLOCK, sector);
		if (err < 0) {
			goto error;
		}

		err = sdhc_spi_tx_block(data, (u8_t *)buf,
			SDMMC_DEFAULT_BLOCK_SIZE);
		if (err != 0) {
			goto error;
		}

		/* Wait for the card to finish programming */
		err = sdhc_spi_skip_until_ready(data);
		if (err != 0) {
			goto error;
		}

		err = sdhc_spi_cmd_r2(data, SDHC_SEND_STATUS, 0);
		if (err != 0) {
			goto error;
		}

		buf += SDMMC_DEFAULT_BLOCK_SIZE;
		sector++;
	}

	err = 0;
error:
	sdhc_spi_set_cs(data, 1);

	return err;
}

static int disk_spi_sdhc_init(struct device *dev);

static int sdhc_spi_init(struct device *dev)
{
	struct sdhc_spi_data *data = dev->driver_data;

	data->spi = device_get_binding(DT_INST_0_ZEPHYR_MMC_SPI_SLOT_BUS_NAME);

	data->cfg.frequency = SDHC_SPI_INITIAL_SPEED;
	data->cfg.operation = SPI_WORD_SET(8) | SPI_HOLD_ON_CS;
	data->cfg.slave = DT_INST_0_ZEPHYR_MMC_SPI_SLOT_BASE_ADDRESS;
	data->cs = device_get_binding(
		DT_INST_0_ZEPHYR_MMC_SPI_SLOT_CS_GPIOS_CONTROLLER);
	__ASSERT_NO_MSG(data->cs != NULL);

	data->pin = DT_INST_0_ZEPHYR_MMC_SPI_SLOT_CS_GPIOS_PIN;

	disk_spi_sdhc_init(dev);

	return gpio_pin_configure(data->cs, data->pin, GPIO_DIR_OUT);
}

static int disk_spi_sdhc_access_status(struct disk_info *disk)
{
	struct device *dev = disk->dev;
	struct sdhc_spi_data *data = dev->driver_data;

	return data->status;
}

static int disk_spi_sdhc_access_read(struct disk_info *disk,
	u8_t *buf, u32_t sector, u32_t count)
{
	struct device *dev = disk->dev;
	struct sdhc_spi_data *data = dev->driver_data;
	int err;

	LOG_DBG("sector=%u count=%u", sector, count);

	err = sdhc_spi_read(data, buf, sector, count);
	if (err != 0 && sdhc_is_retryable(err)) {
		sdhc_spi_recover(data);
		err = sdhc_spi_read(data, buf, sector, count);
	}

	return err;
}

static int disk_spi_sdhc_access_write(struct disk_info *disk,
	const u8_t *buf, u32_t sector, u32_t count)
{
	struct device *dev = disk->dev;
	struct sdhc_spi_data *data = dev->driver_data;
	int err;

	LOG_DBG("sector=%u count=%u", sector, count);

	err = sdhc_spi_write(data, buf, sector, count);
	if (err != 0 && sdhc_is_retryable(err)) {
		sdhc_spi_recover(data);
		err = sdhc_spi_write(data, buf, sector, count);
	}

	return err;
}

static int disk_spi_sdhc_access_ioctl(struct disk_info *disk,
	u8_t cmd, void *buf)
{
	struct device *dev = disk->dev;
	struct sdhc_spi_data *data = dev->driver_data;
	int err;

	err = sdhc_map_disk_status(data->status);
	if (err != 0) {
		return err;
	}

	switch (cmd) {
	case DISK_IOCTL_CTRL_SYNC:
		break;
	case DISK_IOCTL_GET_SECTOR_COUNT:
		*(u32_t *)buf = data->sector_count;
		break;
	case DISK_IOCTL_GET_SECTOR_SIZE:
		*(u32_t *)buf = SDMMC_DEFAULT_BLOCK_SIZE;
		break;
	case DISK_IOCTL_GET_ERASE_BLOCK_SZ:
		*(u32_t *)buf = SDMMC_DEFAULT_BLOCK_SIZE;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int disk_spi_sdhc_access_init(struct disk_info *disk)
{
	struct device *dev = disk->dev;
	struct sdhc_spi_data *data = dev->driver_data;
	int err;

	if (data->status == DISK_STATUS_OK) {
		/* Called twice, don't re-init. */
		return 0;
	}

	err = sdhc_spi_detect(data);
	sdhc_spi_set_cs(data, 1);

	return err;
}

static const struct disk_operations spi_sdhc_disk_ops = {
	.init = disk_spi_sdhc_access_init,
	.status = disk_spi_sdhc_access_status,
	.read = disk_spi_sdhc_access_read,
	.write = disk_spi_sdhc_access_write,
	.ioctl = disk_spi_sdhc_access_ioctl,
};

static struct disk_info spi_sdhc_disk = {
	.name = CONFIG_DISK_SDHC_VOLUME_NAME,
	.ops = &spi_sdhc_disk_ops,
};

static int disk_spi_sdhc_init(struct device *dev)
{
	struct sdhc_spi_data *data = dev->driver_data;

	data->status = DISK_STATUS_UNINIT;

	spi_sdhc_disk.dev = dev;

	return disk_access_register(&spi_sdhc_disk);
}

static struct sdhc_spi_data sdhc_spi_data_0;

DEVICE_AND_API_INIT(sdhc_spi_0,
	DT_INST_0_ZEPHYR_MMC_SPI_SLOT_LABEL,
	sdhc_spi_init, &sdhc_spi_data_0, NULL,
	APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, NULL);
#endif

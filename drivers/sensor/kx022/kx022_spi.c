#include "kx022.h"

#include <zephyr/drivers/spi.h>

int kx022_bus_access(const struct device *dev, uint8_t reg,
			      void *data, size_t length)
{
	const struct kx022_config *config = dev->config;

	const struct spi_buf buf[2] = {
		{
			.buf = &reg,
			.len = 1
		}, {
			.buf = data,
			.len = length
		}
	};

	struct spi_buf_set tx = {
		.buffers = buf,
	};

	if (reg & KX022_READ) {
		const struct spi_buf_set rx = {
			.buffers = buf,
			.count = 2
		};

		tx.count = 1;

		return spi_transceive_dt(&config->bus, &tx, &rx);
	}

	tx.count = 2;

	return spi_write_dt(&config->bus, &tx);
}

#define DT_DRV_COMPAT kionix_kx022

#define KX022_DEFINE(inst)									\
	static struct kx022_data kx022_data_##inst;						\
												\
	static const struct kx022_config kx022_config_##inst = {				\
		.bus = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0),	\
		.low_current = true,								\
		.tilt_detection = false,							\
		.tap_detection = false,								\
		.motion_detection = false,							\
		.range = KX022_RANGE_2G,							\
		.odr = KX022_ODR_50_HZ,								\
	};											\
												\
	DEVICE_DT_INST_DEFINE(inst, kx022_init, NULL, &kx022_data_##inst,			\
			&kx022_config_##inst, POST_KERNEL,					\
			CONFIG_SENSOR_INIT_PRIORITY, &kx022_api_funcs);				\

DT_INST_FOREACH_STATUS_OKAY(KX022_DEFINE)

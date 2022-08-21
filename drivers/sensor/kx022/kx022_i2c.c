#include "kx022.h"
#include <zephyr/drivers/i2c.h>

int kx022_bus_access(const struct device *dev, uint8_t reg,
			      void *data, size_t length)
{
	const struct kx022_config *config = dev->config;
	if (reg & KX022_READ) {
		return i2c_burst_read_dt(&config->bus,
					 KX022_TO_I2C_REG(reg),
					 (uint8_t *) data, length);
	} else {
		if (length != 1) {
			return -EINVAL;
		}

		return i2c_reg_write_byte_dt(&config->bus,
					     KX022_TO_I2C_REG(reg),
					     *(uint8_t *)data);
	}

}

#define DT_DRV_COMPAT kionix_kx022

#define KX022_DEFINE(inst)									\
	static struct kx022_data kx022_data_##inst;						\
												\
	static const struct kx022_config kx022_config_##inst = {				\
		.bus = I2C_DT_SPEC_INST_GET(inst),						\
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

/* kx022.c - KX022 Three-Axis Digital Accelerometers */

/*
 * Copyright (c) 2022 Maximilian Deubel
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include "kx022.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/__assert.h>
#include <stdlib.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(KX022, CONFIG_SENSOR_LOG_LEVEL);

static int kx022_reg_read(const struct device *dev,
			     uint8_t reg_addr,
			     uint8_t *reg_data,
                             uint16_t count)
{
	return kx022_bus_access(dev, KX022_REG_READ(reg_addr), reg_data, count);
}

static int kx022_reg_write(const struct device *dev,
			      uint8_t reg_addr,
			      uint8_t reg_data)
{
	LOG_DBG("[0x%X] = 0x%X", reg_addr, reg_data);

	return kx022_bus_access(dev, KX022_REG_WRITE(reg_addr), &reg_data, 1);
}

int kx022_init(const struct device *dev)
{
	const struct kx022_config *cfg = dev->config;
	struct kx022_data *data = dev->data;

#ifdef CONFIG_KX022_I2C
	if (!device_is_ready(cfg->bus.bus)) {
		LOG_ERR("I2C bus %s not ready!", cfg->bus.bus->name);
		return -EINVAL;
	}
#endif
#ifdef CONFIG_KX022_SPI
	if (!spi_is_ready(&cfg->bus)) {
		LOG_ERR("SPI bus %s not ready!", cfg->bus.bus->name);
		return -EINVAL;
	}
#endif

	/* standby mode */
	kx022_reg_write(dev, KX022_CNTL1, KX022_CNTL1_PC1_MODE(0));
	/* output data rate */
	kx022_reg_write(dev, KX022_ODCNTL, KX022_ODCNTL_ODR_MODE(cfg->odr));
	/* function odrs */
	kx022_reg_write(dev, KX022_CNTL3, 
		KX022_CNTL3_TILT_ODR_MODE(cfg->tilt_odr) |
		KX022_CNTL3_TAP_ODR_MODE(cfg->tap_odr) |
		KX022_CNTL3_WAKEUP_ODR_MODE(cfg->motion_detection_odr));
	/* enable/disable features, operating mode */
	kx022_reg_write(dev, KX022_CNTL1,
		KX022_CNTL1_PC1_MODE(1) |
		KX022_CNTL1_RES_MODE(cfg->low_current) |
		KX022_CNTL1_TPE_MODE(cfg->tilt_detection) |
		KX022_CNTL1_TDTE_MODE(cfg->tap_detection) |
		KX022_CNTL1_WUFE_MODE(cfg->motion_detection) |
		KX022_CNTL1_GSEL_MODE(cfg->range));

	data->range = cfg->range;

	return 0;
}

static int kx022_attr_set(const struct device *dev,
			    enum sensor_channel chan,
			    enum sensor_attribute attr,
			    const struct sensor_value *val)
{
	return -ENOTSUP;
}

static int kx022_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	struct kx022_data *data = dev->data;
	int16_t buf[3];
	int ret;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	ret = kx022_reg_read(dev, KX022_XOUT_L, (uint8_t *)buf,sizeof(buf));

	if (ret) {
		return ret;
	}

	for (size_t i = 0; i < 3; ++i) {
		data->acc_xyz[i] = sys_le16_to_cpu(buf[i]);
	}

	return 0;
}

static void adxl362_accel_convert(struct sensor_value *val, int accel,
				  enum kx022_range range)
{
	long scale = SENSOR_G / (32768 / 2);
	if (range == KX022_RANGE_4G) {
		scale = SENSOR_G / (32768 / 4);
	} else if (range == KX022_RANGE_8G) {
		scale = SENSOR_G / (32768 / 8);
	} else {
		__ASSERT_NO_MSG(range == KX022_RANGE_2G);
	}
	long micro_ms2 = accel * scale;

	__ASSERT_NO_MSG(scale != -EINVAL);

	val->val1 = micro_ms2 / 1000000;
	val->val2 = micro_ms2 % 1000000;
}

static int kx022_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct kx022_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		adxl362_accel_convert(val, data->acc_xyz[0], data->range);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		adxl362_accel_convert(val, data->acc_xyz[1], data->range);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		adxl362_accel_convert(val, data->acc_xyz[2],  data->range);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		for (size_t i = 0; i < 3; i++) {
			adxl362_accel_convert(&val[i], data->acc_xyz[i], data->range);
		}
		break;
	default:
		return -ENOTSUP;
	}
	return 0;
}

static int kx022_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler)
{
	return 0;
}

const struct sensor_driver_api kx022_api_funcs = {
	.attr_set     = kx022_attr_set,
	.sample_fetch = kx022_sample_fetch,
	.channel_get  = kx022_channel_get,
	.trigger_set = kx022_trigger_set,
};


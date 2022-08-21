/*
 * Copyright (c) 2022 Maximilian Deubel
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_KX022_KX022_H_
#define ZEPHYR_DRIVERS_SENSOR_KX022_KX022_H_

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/util.h>

#define KX022_XHP_L		0x00u
#define KX022_XHP_H		0x01u
#define KX022_YHP_L		0x02u
#define KX022_YHP_H		0x03u
#define KX022_ZHP_L		0x04u
#define KX022_ZHP_H		0x05u
#define KX022_XOUT_L		0x06u
#define KX022_XOUT_H		0x07u
#define KX022_YOUT_L		0x08u
#define KX022_YOUT_H		0x09u
#define KX022_ZOUT_L		0x0Au
#define KX022_ZOUT_H		0x0Bu
#define KX022_COTR		0x0Cu
#define KX022_WHO_AM_I		0x0Fu
#define KX022_TSCP		0x10u
#define KX022_TSPP		0x11u
#define KX022_INS1		0x12u
#define KX022_INS2		0x13u
#define KX022_INS3		0x14u
#define KX022_STAT		0x15u
#define KX022_INT_REL		0x17u
#define KX022_CNTL1		0x18u
#define KX022_CNTL2		0x19u
#define KX022_CNTL3		0x1Au
#define KX022_ODCNTL		0x1Bu
#define KX022_INC1		0x1Cu
#define KX022_INC2		0x1Du
#define KX022_INC3		0x1Eu
#define KX022_INC4		0x1Fu
#define KX022_INC5		0x20u
#define KX022_INC6		0x21u
#define KX022_TILT_TIMER	0x22u
#define KX022_WUFC		0x23u
#define KX022_TDTRC		0x24u
#define KX022_TDTC		0x25u
#define KX022_TTH		0x26u
#define KX022_TTL		0x27u
#define KX022_FTD		0x28u
#define KX022_STD		0x29u
#define KX022_TLT		0x2Au
#define KX022_TWS		0x2Bu
#define KX022_ATH		0x30u
#define KX022_TILT_ANGLE_LL	0x32u
#define KX022_TILT_ANGLE_HL	0x33u
#define KX022_HYST_SET		0x34u
#define KX022_LP_CNTL		0x35u
#define KX022_BUF_CNTL1		0x3Au
#define KX022_BUF_CNTL2		0x3Bu
#define KX022_BUF_STATUS_1	0x3Cu
#define KX022_BUF_STATUS_2	0x3Du
#define KX022_BUF_CLEAR		0x3Eu
#define KX022_BUF_READ		0x3Fu
#define KX022_SELF_TEST		0x60u

#define KX112_FFTH		0x2Cu
#define KX112_FFC		0x2Du
#define KX112_FFCNTL		0x2Eu

/* KX022_ODCNTL */
#define KX022_ODCNTL_IIR_BYPASS_MSK		BIT(7)
#define KX022_ODCNTL_IIR_BYPASS_MODE(x)		(((x) & 0x1) << 7)
#define KX022_ODCNTL_LPRO_MSK			BIT(6)
#define KX022_ODCNTL_LPRO_MODE(x)		(((x) & 0x1) << 6)
#define KX022_ODCNTL_ODR_MSK			GENMASK(3, 0)
#define KX022_ODCNTL_ODR_MODE(x)		(((x) & 0xf))

/* KX022_CNTL1 */
#define KX022_CNTL1_PC1_MSK			BIT(7)
#define KX022_CNTL1_PC1_MODE(x)			(((x) & 0x1) << 7)
#define KX022_CNTL1_RES_MSK			BIT(6)
#define KX022_CNTL1_RES_MODE(x)			(((x) & 0x1) << 6)
#define KX022_CNTL1_DRDYE_MSK			BIT(5)
#define KX022_CNTL1_DRDYE_MODE(x)		(((x) & 0x1) << 5)
#define KX022_CNTL1_GSEL_MSK			GENMASK(4, 3)
#define KX022_CNTL1_GSEL_MODE(x)		(((x) & 0x3) << 3)
#define KX022_CNTL1_TDTE_MSK			BIT(2)
#define KX022_CNTL1_TDTE_MODE(x)		(((x) & 0x1) << 2)
#define KX022_CNTL1_WUFE_MSK			BIT(1)
#define KX022_CNTL1_WUFE_MODE(x)		(((x) & 0x1) << 1)
#define KX022_CNTL1_TPE_MSK			BIT(0)
#define KX022_CNTL1_TPE_MODE(x)			(((x) & 0x1) << 0)

/* KX022_CNTL2 */
#define KX022_CNTL2_SRST_MSK			BIT(7)
#define KX022_CNTL2_SRST_MODE(x)		(((x) & 0x1) << 7)
#define KX022_CNTL2_COTC_MSK			BIT(6)
#define KX022_CNTL2_COTC_MODE(x)		(((x) & 0x1) << 6)
#define KX022_CNTL2_LEM_MSK			BIT(5)
#define KX022_CNTL2_LEM_MODE(x)			(((x) & 0x1) << 5)
#define KX022_CNTL2_RIM_MSK			BIT(4)
#define KX022_CNTL2_RIM_MODE(x)			(((x) & 0x1) << 4)
#define KX022_CNTL2_DOM_MSK			BIT(3)
#define KX022_CNTL2_DOM_MODE(x)			(((x) & 0x1) << 3)
#define KX022_CNTL2_UPM_MSK			BIT(2)
#define KX022_CNTL2_UPM_MODE(x)			(((x) & 0x1) << 2)
#define KX022_CNTL2_FDM_MSK			BIT(1)
#define KX022_CNTL2_FDM_MODE(x)			(((x) & 0x1) << 1)
#define KX022_CNTL2_FUM_MSK			BIT(0)
#define KX022_CNTL2_FUM_MODE(x)			(((x) & 0x1) << 0)

/* KX022_CNTL3 */
#define KX022_CNTL3_TILT_ODR_MSK		GENMASK(7, 6)
#define KX022_CNTL3_TILT_ODR_MODE(x)		(((x) & 0x3) << 6)
#define KX022_CNTL3_TAP_ODR_MSK			GENMASK(5, 3)
#define KX022_CNTL3_TAP_ODR_MODE(x)		(((x) & 0x7) << 3)
#define KX022_CNTL3_WAKEUP_ODR_MSK		GENMASK(2, 0)
#define KX022_CNTL3_WAKEUP_ODR_MODE(x)		(((x) & 0x7))

#define KX022_READ		0x80u
#define KX022_REG_READ(x)	(((x & 0xFFu)) | KX022_READ)
#define KX022_REG_WRITE(x)	((x & 0xFFu))
#define KX022_TO_I2C_REG(x)	((x) & ~KX022_READ)

enum kx022_odr
{
	KX022_ODR_12_5_HZ = 0,
	KX022_ODR_25_HZ,
	KX022_ODR_50_HZ,
	KX022_ODR_100_HZ,
	KX022_ODR_200_HZ,
	KX022_ODR_400_HZ,
	KX022_ODR_800_HZ,
	KX022_ODR_1600_HZ,
	KX022_ODR_0_781_HZ,
	KX022_ODR_1_563_HZ,
	KX022_ODR_3_125_HZ,
	KX022_ODR_6_25_HZ
};

enum kx022_range
{
	KX022_RANGE_2G = 0,
	KX022_RANGE_4G,
	KX022_RANGE_8G,
};

enum kx022_tilt_odr
{
	KX022_TILT_ODR_1_563_HZ = 0,
	KX022_TILT_ODR_6_25_HZ,
	KX022_TILT_ODR_12_5_HZ,
	KX022_TILT_ODR_50_HZ,
};

enum kx022_tap_odr
{
	KX022_TAP_ODR_50_HZ = 0,
	KX022_TAP_ODR_100_HZ,
	KX022_TAP_ODR_200_HZ,
	KX022_TAP_ODR_400_HZ,
	KX022_TAP_ODR_12_5_HZ,
	KX022_TAP_ODR_25_HZ,
	KX022_TAP_ODR_800_HZ,
	KX022_TAP_ODR_1600_HZ,
};

enum kx022_wakeup_odr
{
	KX022_WAKEUP_ODR_0_781_HZ = 0,
	KX022_WAKEUP_ODR_1_563_HZ,
	KX022_WAKEUP_ODR_3_125_HZ,
	KX022_WAKEUP_ODR_6_25_HZ,
	KX022_WAKEUP_ODR_12_5_HZ,
	KX022_WAKEUP_ODR_25_HZ,
	KX022_WAKEUP_ODR_50_HZ,
	KX022_WAKEUP_ODR_100_HZ,
};

struct kx022_config {
#ifdef CONFIG_KX022_I2C
	struct i2c_dt_spec bus;
#endif /* CONFIG_KX022_I2C */
#ifdef CONFIG_KX022_SPI
	struct spi_dt_spec bus;
#endif /* CONFIG_KX022_SPI */
#ifdef CONFIG_KX022_TRIGGER
	struct gpio_dt_spec interrupt;
#endif /* CONFIG_KX022_TRIGGER */
	enum kx022_odr odr;			// default: KX022_ODR_50_HZ
	enum kx022_tilt_odr tilt_odr;		// default: KX022_TILT_ODR_12_5_HZ
	enum kx022_tap_odr tap_odr;		// default: KX022_TAP_ODR_400_HZ
	enum kx022_wakeup_odr motion_detection_odr;	// default: KX022_WAKEUP_ODR_0_781_HZ
	enum kx022_range range;			// default: KX022_RANGE_2G
	bool low_current;			// default: true
	bool tap_detection;			// default: false
	bool motion_detection;			// default: false
	bool tilt_detection;			// default: false
};

struct kx022_data {
	int16_t acc_xyz[3];
	enum kx022_range range;
};

int kx022_bus_access(const struct device *dev, uint8_t reg,
		     void *data, size_t length);

int kx022_init(const struct device *dev);

extern const struct sensor_driver_api kx022_api_funcs;

#endif /* ZEPHYR_DRIVERS_SENSOR_KX022_KX022_H_ */
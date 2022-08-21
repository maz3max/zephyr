#
#  Copyright (c) 2022 Maximilian Deubel
#
#  SPDX-License-Identifier: Apache-2.0
#
zephyr_library()

zephyr_library_sources(kx022.c)
zephyr_library_sources_ifdef(CONFIG_KX022_SPI kx022_spi.c)
zephyr_library_sources_ifdef(CONFIG_KX022_I2C kx022_i2c.c)

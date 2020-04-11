/*
 * Copyright (c) 2020 Nick Ward
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef _X4HC595_H_
#define _X4HC595_H_

#include <drivers/gpio.h>

/* TODO */
#define DT_INST_X4HC595_DEV_HAS_SRCLK_GPIOS(x) false
#define DT_INST_X4HC595_DEV_HAS_OE_GPIOS(x) false
#define DT_X4HC595_SR_CNT                   2
#define CONFIG_X4HC595_INIT_PRIORITY 50

#define DEV_CFG(dev) \
	((const struct x4hc595_config *const)(dev)->config->config_info)
#define DEV_DATA(dev) ((struct x4hc595_data *const)(dev)->driver_data)

struct x4hc595_data {
	struct k_work work;

	/* spi device data */
	struct device *spi;
	struct spi_config spi_cfg;

	/* ? data */
	struct device *rclk_gpio;
#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
	/* TODO - need to deal with this if other drivers have enabled it for SPI? */
	struct spi_cs_control spi_cs_ctrl;
#endif /* DT_INST_SPI_DEV_HAS_CS_GPIOS(0) */

	/* general data */
	struct k_mutex mutex;

	/* output data */
	gpio_port_value_t value;
};

struct x4hc595_config {
	/* spi configuration */
	const char *spi_port;
	u32_t spi_freq;
	u8_t spi_slave;

	/* control pins */
#if DT_INST_X4HC595_DEV_HAS_SRCLK_GPIOS(0)
	u8_t srclk_pin;
	const char *srclk_port;
#endif /* DT_INST_X4HC595_DEV_HAS_SRCLK_GPIOS(0) */
	u8_t rclk_pin;
	const char *rclk_port;
#if DT_INST_X4HC595_DEV_HAS_OE_GPIOS(0)
	u8_t oe_pin;
	const char *oe_port;
#endif /* DT_INST_X4HC595_DEV_HAS_OE_GPIOS(0) */

	/* shift register count - TODO needed? */
	u8_t sr_cnt;
};

#endif /*_X4HC595_H_*/

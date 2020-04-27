/*
 * Copyright (c) 2020 Nick Ward
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef _X4HC595_H_
#define _X4HC595_H_

#include <drivers/gpio.h>


#define DEV_CFG(dev) \
	((const struct x4hc595_cfg *const)(dev)->config->config_info)
#define DEV_DATA(dev) ((struct x4hc595_data *const)(dev)->driver_data)

struct x4hc595_data {
	struct device *spi_dev;
	struct spi_cs_control cs_ctrl;

	/* general data */
	struct k_mutex mutex;

	/* output data */
	gpio_port_value_t value;
};

struct x4hc595_cfg {
	/* spi configuration */
	const char *spi_port;
	u32_t spi_freq;
	struct spi_config spi_cfg;
	const char *gpio_cs_port;
	u8_t spi_slave;
	u8_t spi_cs_pin;

	/* control pins */
	u8_t srclr_pin;
	const char *srclr_port;
	u8_t oe_pin;
	const char *oe_port;

	/* shift register count - TODO needed? */
	u8_t sr_cnt;
};

#endif /*_X4HC595_H_*/

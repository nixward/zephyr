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
#define DT_INST_X4HC595_DEV_HAS_SRCLR_GPIOS(x) false
#define DT_INST_X4HC595_DEV_HAS_OE_GPIOS(x) false
#define DT_X4HC595_SR_CNT                   2
#define CONFIG_X4HC595_INIT_PRIORITY 50

#define DEV_CFG(dev) \
	((const struct x4hc595_cfg *const)(dev)->config->config_info)
#define DEV_DATA(dev) ((struct x4hc595_data *const)(dev)->driver_data)

struct x4hc595_data {
	struct k_work work;

	/* spi device data */
	struct device *spi;
	struct spi_config spi_cfg;

	struct spi_cs_control spi_cs_ctrl;

	/* general data */
	struct k_mutex mutex;

	/* output data */
	gpio_port_value_t value;
};

struct x4hc595_cfg {
	/* spi configuration */
	const char *spi_port;
	u32_t spi_freq;
	u8_t spi_slave;

	/* control pins */
	u8_t srclr_pin;
	const char *srclr_port;
	u8_t spi_cs_pin;
	const char *spi_cs_port;
	u8_t oe_pin;
	const char *oe_port;

	/* shift register count - TODO needed? */
	u8_t sr_cnt;
};

#endif /*_X4HC595_H_*/

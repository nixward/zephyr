/*
 * SN54HC595, SN74HC595
 * 8-BIT SHIFT REGISTERS WITH 3-STATE OUTPUT REGISTERS
 * 
 * Copyright (c) 2020 Nick Ward
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_x4hc595_gpio

#include <kernel.h>
#include <device.h>
#include <drivers/spi.h>
#include <drivers/gpio.h>
#include <sys/byteorder.h>

#define LOG_LEVEL CONFIG_GPIO_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(x5hc595_gpio);

#include "gpio_x4hc595.h"

static int x4hc595_refresh(struct device *dev)
{
	const struct x4hc595_cfg *dev_cfg = DEV_CFG(dev);
	struct x4hc595_data *dev_data = DEV_DATA(dev);
	int ret;

	/* LSB is shifted last */
	u32_t val = sys_cpu_to_be32(dev_data->value) >> (4 - dev_cfg->sr_cnt);

	struct spi_buf tx_buf[] = {
		{ .buf = &val, .len = dev_cfg->sr_cnt }
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf, .count = ARRAY_SIZE(tx_buf)
	};

	ret = spi_write(dev_data->spi_dev, &dev_cfg->spi_cfg, &tx);
	if (ret != 0) {
		LOG_ERR("Shifting data [%d]", ret);
		return ret;
	}

	return 0;
}

#define GPIO_FLAGS_NOT_SUPPORTED (GPIO_INPUT | GPIO_INT_ENABLE | \
GPIO_INT_EDGE | GPIO_INT_LOW_0 | GPIO_INT_HIGH_1)

int x4hc595_pin_configure(struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct x4hc595_cfg *dev_cfg = DEV_CFG(dev);
	//struct x4hc595_data *dev_data = DEV_DATA(dev);

	if (pin >= dev_cfg->sr_cnt * 8) {
		LOG_ERR("GPIO pin %u configutaion not supported", pin);
		return -EINVAL;
	}

	if (flags & GPIO_FLAGS_NOT_SUPPORTED) {
		LOG_ERR("GPIO flags (0x%08X) configuation not supported", flags);
		return -ENOTSUP;
	}

	if (flags == GPIO_DISCONNECTED) {
		LOG_ERR("GPIO flags: disconnection not supported");
		return -ENOTSUP;
	}

	return 0;
}

int x4hc595_port_get_raw(struct device *dev, gpio_port_value_t *value)
{
	struct x4hc595_data *dev_data = DEV_DATA(dev);

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	*value = dev_data->value;

	k_mutex_unlock(&dev_data->mutex);

	return 0;
}

int x4hc595_port_set_masked_raw(struct device *dev, gpio_port_pins_t mask,
			   gpio_port_value_t value)
{
	const struct x4hc595_cfg *dev_cfg = DEV_CFG(dev);
	struct x4hc595_data *dev_data = DEV_DATA(dev);

	gpio_port_pins_t inval_mask = ~BIT_MASK(dev_cfg->sr_cnt * 8);
	if ((mask & inval_mask) != 0U) {
		LOG_ERR("Invalid mask 0x%08X", mask);
		return -EINVAL;
	}

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	dev_data->value |= mask & value;
	dev_data->value &= ~(mask & ~value);
	x4hc595_refresh(dev);

	k_mutex_unlock(&dev_data->mutex);

	return 0;
}

int x4hc595_port_set_bits_raw(struct device *dev, gpio_port_pins_t pins)
{
	const struct x4hc595_cfg *dev_cfg = DEV_CFG(dev);
	struct x4hc595_data *dev_data = DEV_DATA(dev);

	gpio_port_pins_t inval_mask = ~BIT_MASK(dev_cfg->sr_cnt * 8);
	if ((pins & inval_mask) != 0U) {
		LOG_ERR("Invalid mask 0x%08X", inval_mask);
		return -EINVAL;
	}

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	dev_data->value |= pins;
	x4hc595_refresh(dev);

	k_mutex_unlock(&dev_data->mutex);

	return 0;
}

int x4hc595_port_clear_bits_raw(struct device *dev, gpio_port_pins_t pins)
{
	const struct x4hc595_cfg *dev_cfg = DEV_CFG(dev);
	struct x4hc595_data *dev_data = DEV_DATA(dev);

	gpio_port_pins_t inval_mask = ~BIT_MASK(dev_cfg->sr_cnt * 8);
	if ((pins & inval_mask) != 0U) {
		LOG_ERR("Invalid mask 0x%08X", pins & inval_mask);
		return -EINVAL;
	}

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	dev_data->value &= ~pins;
	x4hc595_refresh(dev);

	k_mutex_unlock(&dev_data->mutex);

	return 0;
}

int x4hc595_port_toggle_bits(struct device *dev, gpio_port_pins_t pins)
{
	const struct x4hc595_cfg *dev_cfg = DEV_CFG(dev);
	struct x4hc595_data *dev_data = DEV_DATA(dev);
	int ret;

	gpio_port_pins_t inval_mask = ~BIT_MASK(dev_cfg->sr_cnt * 8);
	if ((pins & inval_mask) != 0U) {
		LOG_ERR("Invalid pins 0x%08X", pins & inval_mask);
		return -EINVAL;
	}

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	dev_data->value ^= pins;
	x4hc595_refresh(dev);

	k_mutex_unlock(&dev_data->mutex);

	return ret;
}

int x4hc595_pin_interrupt_configure(struct device *dev, gpio_pin_t pin,
			       enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	(void)dev;
	(void)pin;
	(void)mode;
	(void)trig;

	LOG_ERR("Not supported");

	return -ENOTSUP;
}

int x4hc595_manage_callback(struct device *dev, struct gpio_callback *cb,
		       bool set)
{
	(void)dev;
	(void)cb;
	(void)set;

	LOG_ERR("Not supported");

	return -ENOTSUP;
}

int x4hc595_enable_callback(struct device *dev, gpio_pin_t pin)
{
	(void)dev;
	(void)pin;

	LOG_ERR("Not supported");

	return -ENOTSUP;
}

int x4hc595_disable_callback(struct device *dev, gpio_pin_t pin)
{
	(void)dev;
	(void)pin;

	LOG_ERR("Not supported");

	return -ENOTSUP;
}

u32_t x4hc595_get_pending_int(struct device *dev)
{
	(void)dev;

	LOG_ERR("Not supported");

	return -ENOTSUP;
}

static const struct gpio_driver_api x4hc595_gpio_api_funcs = {
	.pin_configure = x4hc595_pin_configure,
	.port_get_raw = x4hc595_port_get_raw,
	.port_set_masked_raw = x4hc595_port_set_masked_raw,
	.port_set_bits_raw = x4hc595_port_set_bits_raw,
	.port_clear_bits_raw = x4hc595_port_clear_bits_raw,
	.port_toggle_bits = x4hc595_port_toggle_bits,
	.pin_interrupt_configure = x4hc595_pin_interrupt_configure,
	.manage_callback = x4hc595_manage_callback,
	.enable_callback = x4hc595_enable_callback,
	.disable_callback = x4hc595_disable_callback,
	.get_pending_int = x4hc595_get_pending_int
};

static int gpio_x4hc595_init(struct device *dev)
{
	const struct x4hc595_cfg *dev_cfg = DEV_CFG(dev);
	struct x4hc595_data *dev_data = DEV_DATA(dev);
	struct device *srclr_gpio = NULL;
	struct device *oe_gpio = NULL;
	int ret;

	k_mutex_init(&dev_data->mutex);

	dev_data->spi_dev = device_get_binding(dev_cfg->spi_port);
	if (dev_data->spi_dev == NULL) {
		LOG_ERR("SPI master device %s not found", dev_cfg->spi_port);
		return -ENODEV;
	}

	if (dev_cfg->srclr_port == NULL) {
		LOG_WRN("SRCLR pin not controlled by driver");
	} else {
		/* Set SRCLK input to physical low - shift registers cleared */
		srclr_gpio = device_get_binding(dev_cfg->srclr_port);
		if (srclr_gpio == NULL) {
			LOG_ERR("GPIO device %s not found", dev_cfg->srclr_port);
			return -ENODEV;
		}

		ret = gpio_pin_set_raw(srclr_gpio, dev_cfg->srclr_pin, 0);
		if (ret != 0) {
			LOG_ERR("SRCLK low");
			return ret;
		}

		ret = gpio_pin_configure(srclr_gpio, dev_cfg->srclr_pin,
					 GPIO_OUTPUT);
		if (ret != 0) {
			LOG_ERR("Unable to configure GPIO pin %u",
				dev_cfg->srclr_pin);
			return ret;
		}
	}

	if (dev_cfg->oe_port == NULL) {
		LOG_WRN("OE pin not controlled by driver");
	} else {
		/* Set OE input to physical high - outputs disabled */
		oe_gpio = device_get_binding(dev_cfg->oe_port);
		if (oe_gpio == NULL) {
			LOG_ERR("GPIO device %s not found", dev_cfg->oe_port);
			return -ENODEV;
		}

		ret = gpio_pin_set_raw(oe_gpio, dev_cfg->oe_pin, 1);
		if (ret != 0) {
			LOG_ERR("Failed to set OE high");
			return ret;
		}

		ret = gpio_pin_configure(oe_gpio, dev_cfg->oe_pin,
					 GPIO_OUTPUT);
		if (ret != 0) {
			LOG_ERR("Unable to configure GPIO pin %u",
				dev_cfg->oe_pin);
			return ret;
		}
	}

	dev_data->value = 0U;

	x4hc595_refresh(dev);

	if (oe_gpio != NULL) {
		/* Set OE input to physical low - outputs enabled */
		ret = gpio_pin_set_raw(oe_gpio, dev_cfg->oe_pin, 0);
		if (ret != 0) {
			LOG_ERR("Failed to set OE pin low");
			return ret;
		}
	}

	return 0;
}

#define GPIO_X4HC595_DEVICE(inst)                                    \
	static struct x4hc595_data x4hc595_##inst##_data;              \
	static const struct x4hc595_cfg x4hc595_##inst##_cfg = {       \
	.spi_port = NULL /*DT_INST_BUS_LABEL(inst)*/,                         \
	.spi_freq = DT_INST_PROP(inst, spi_max_frequency),           \
	.spi_cfg.operation = (SPI_OP_MODE_MASTER | SPI_MODE_CPOL |  \
		               SPI_MODE_CPHA | SPI_WORD_SET(8) |     \
		               SPI_LINES_SINGLE),                    \
	.spi_slave = DT_INST_REG_ADDR(inst),                         \
	.gpio_cs_port	    = DT_INST_SPI_DEV_CS_GPIOS_LABEL(inst),  \
	.cs_gpio	    = DT_INST_SPI_DEV_CS_GPIOS_PIN(inst),    \
	.spi_cfg.cs        = &x4hc595_##inst##_data.cs_ctrl,          \
	.srclr_pin = 0,                                              \
	.srclr_port = NULL,                                          \
	.oe_pin = 0,                                                 \
	.oe_port = NULL,                                             \
	.sr_cnt = DT_INST_PROP(inst, sr_cnt)                         \
     };                                                              \
     DEVICE_AND_API_INIT(x4hc595_##inst,                             \
                         DT_INST_LABEL(inst),                        \
			 gpio_x4hc595_init,                          \
                         &x4hc595_##inst##_data,                       \
                         &x4hc595_##inst##_cfg,                        \
                         POST_KERNEL,                                \
			 CONFIG_GPIO_X4HC595_INIT_PRIORITY,          \
                         &x4hc595_gpio_api_funcs)

#ifdef CONFIG_GPIO_X4HC595_P0
GPIO_X4HC595_DEVICE(0);
#endif

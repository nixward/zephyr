/*
 * SN54HC595, SN74HC595
 * 8-BIT SHIFT REGISTERS WITH 3-STATE OUTPUT REGISTERS
 * 
 * Copyright (c) 2020 Nick Ward
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_x4hc595

#include <kernel.h>
#include <device.h>
#include <drivers/spi.h>
#include <drivers/gpio.h>

#define LOG_LEVEL CONFIG_GPIO_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(x5hc595_gpio);

#include "gpio_x4hc595.h"

static int x4hc595_refresh(struct device *port)
{
	const struct x4hc595_config *dev_cfg = DEV_CFG(port);
	struct x4hc595_data *dev_data = DEV_DATA(port);
	int ret;

	struct spi_buf tx_buf[] = {
		{ .buf = &dev_data->value, .len = sizeof(dev_data->value) }
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf, .count = ARRAY_SIZE(tx_buf)
	};

	ret = gpio_pin_set_raw(dev_data->rclk_gpio, dev_cfg->rclk_pin, 0);
	if (ret != 0) {
		LOG_ERR("RVCK did not clear");
	}

	ret = spi_write(DEV_DATA(dev)->spi, &DEV_DATA(dev)->spi_cfg, &tx);
	if (ret != 0) {
		LOG_ERR("shifting data");
	}

	ret = gpio_pin_set_raw(dev_data->rclk_gpio, dev_cfg->rclk_pin, 1);
	if (ret != 0) {
		LOG_ERR("RVCK did not set");
	}

	return ret;
}

static int x4hc595_work_handler(struct work *item)
{
	struct device *port = CONTAINER_OF(item, struct device, work);

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	x4hc595_refresh(port);

	k_mutex_unlock(&dev_data->mutex);
}

#define GPIO_FLAGS_NOT_SUPPORTED (GPIO_INPUT | GPIO_INT_ENABLE | 
GPIO_INT_EDGE | GPIO_INT_LOW_0 | GPIO_INT_HIGH_1 |)

int x4hc595_pin_configure(struct device *port, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct x4hc595_config *dev_cfg = DEV_CFG(port);
	struct x4hc595_data *dev_data = DEV_DATA(port);

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

int x4hc595_port_get_raw(struct device *port, gpio_port_value_t *value)
{
	struct x4hc595_data *dev_data = DEV_DATA(port);

	*value = dev_data->value;

	return 0;
}

int x4hc595_port_set_masked_raw(struct device *port, gpio_port_pins_t mask,
			   gpio_port_value_t value)
{
	const struct x4hc595_config *dev_cfg = DEV_CFG(port);
	struct x4hc595_data *dev_data = DEV_DATA(port);

	gpio_port_pins_t inval_mask = ~BIT_MASK(dev_cfg->sr_cnt * 8);
	if (mask & inval_mask != 0U) {
		LOG_ERR("Invalid mask 0x%08X", mask);
		return -EINVAL;
	}

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	dev_data->value |= mask & value;
	dev_data->value &= ~(mask & ~value);

	k_mutex_unlock(&dev_data->mutex);

        k_work_submit(&port.work);

	return 0;
}

int x4hc595_port_set_bits_raw(struct device *port, gpio_port_pins_t pins)
{
	const struct x4hc595_config *dev_cfg = DEV_CFG(port);
	struct x4hc595_data *dev_data = DEV_DATA(port);

	gpio_port_pins_t inval_mask = ~BIT_MASK(dev_cfg->sr_cnt * 8);
	if (pins & inval_mask != 0U) {
		LOG_ERR("Invalid mask 0x%08X", mask);
		return -EINVAL;
	}

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	dev_data->value |= pins;

	k_mutex_unlock(&dev_data->mutex);

        k_work_submit(&port.work);

	return 0;
}

int x4hc595_port_clear_bits_raw(struct device *port, gpio_port_pins_t pins)
{
	const struct x4hc595_config *dev_cfg = DEV_CFG(port);
	struct x4hc595_data *dev_data = DEV_DATA(port);

	gpio_port_pins_t inval_mask = ~BIT_MASK(dev_cfg->sr_cnt * 8);
	if (pins & inval_mask != 0U) {
		LOG_ERR("Invalid mask 0x%08X", pins & inval_mask);
		return -EINVAL;
	}

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	dev_data->value &= ~pins;

	k_mutex_unlock(&dev_data->mutex);

        k_work_submit(&port.work);

	return 0;
}

int x4hc595_port_toggle_bits(struct device *port, gpio_port_pins_t pins)
{
	const struct x4hc595_config *dev_cfg = DEV_CFG(port);
	struct x4hc595_data *dev_data = DEV_DATA(port);
	int ret;

	gpio_port_pins_t inval_mask = ~BIT_MASK(dev_cfg->sr_cnt * 8);
	if (pins & inval_mask != 0U) {
		LOG_ERR("Invalid pins 0x%08X", pins & inval_mask);
		return -EINVAL;
	}

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	dev_data->value ^= pins;

	ret = x4hc595_refresh(port);

	k_mutex_unlock(&dev_data->mutex);

	return ret;
}

int x4hc595_pin_interrupt_configure(struct device *port, gpio_pin_t pin,
			       enum gpio_int_mode, enum gpio_int_trig)
{
	(void)port;
	(void)pin;
	(void)gpio_int_mode;
	(void)gpio_int_trig;

	return -ENOTSUP;
}

int x4hc595_manage_callback(struct device *port, struct gpio_callback *cb,
		       bool set)
{
	(void)port;
	(void)cb;
	(void)set;

	return -ENOTSUP;
}

int x4hc595_enable_callback(struct device *port, gpio_pin_t pin)
{
	(void)port;
	(void)pin;

	return -ENOTSUP;
}

int x4hc595_disable_callback(struct device *port, gpio_pin_t pin)
{
	(void)port;
	(void)pin;

	return -ENOTSUP;
}

u32_t x4hc595_get_pending_int(struct device *port)
{
	(void)port;

	return 0;
}

static int x4hc595_configure(struct device *dev)
{
	const struct x4hc595_config *dev_cfg = DEV_CFG(dev);
	struct x4hc595_data *dev_data = DEV_DATA(dev);
	int ret = 0;

	return ret;
}

static const struct gpio_driver_api gpio_api_funcs = {
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

static int x4hc595_init(struct device *dev)
{
	const struct x4hc595_config *dev_cfg = DEV_CFG(dev);
	struct x4hc595_data *dev_data = DEV_DATA(dev);
	int ret;
#if DT_INST_X4HC595_DEV_HAS_SRCLK_GPIOS(0)
	struct device *srclk_gpio;
#endif /* DT_INST_X4HC595_DEV_HAS_SRCLK_GPIOS(0) */
#if DT_INST_X4HC595_DEV_HAS_OE_GPIOS(0)
	struct device *oe_gpio;
#endif /* DT_INST_X4HC595_DEV_HAS_OE_GPIOS(0) */

	k_mutex_init(&dev_data->mutex);

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	k_work_init(&dev.work, x4hc595_work_handler);

	/* SPI config */
	dev_data->spi_cfg.operation = SPI_WORD_SET(8);
	dev_data->spi_cfg.frequency = dev_cfg->spi_freq;
	dev_data->spi_cfg.slave = dev_cfg->spi_slave;

	dev_data->spi = device_get_binding(dev_cfg->spi_port);
	if (!dev_data->spi) {
		LOG_ERR("SPI master port %s not found", dev_cfg->spi_port);
		return -EINVAL;
	}

	/* No SPI chip select pin */
	dev_data->spi_cfg.cs = NULL;

#if DT_INST_X4HC595_DEV_HAS_SRCLK_GPIOS(0)
	/* Set SRCLK input to physical low - shift registers cleared */
	srclk_gpio = device_get_binding(dev_cfg->srclk_port);
	if (srclk_gpio == NULL) {
		LOG_ERR("GPIO port %s not found", dev_cfg->srclk_port);
		return -EINVAL;
	}

	ret = gpio_pin_set_raw(srclk_gpio, dev_cfg->srclk_pin, 0);
	if (ret != 0) {
		LOG_ERR("SRCLK low");
		return -EINVAL;
	}

	if (gpio_pin_configure(srclk_gpio, dev_cfg->srclk_pin,
			       (GPIO_OUTPUT |
				DT_INST_GPIO_FLAGS(0, srclk_gpios)))) {
		LOG_ERR("Unable to configure GPIO pin %u", dev_cfg->srclk_pin);
		return -EINVAL;
	}
#endif /* DT_INST_X4HC595_DEV_HAS_SRCLK_GPIOS(0) */

#if DT_INST_X4HC595_DEV_HAS_OE_GPIOS(0)
	/* Set OE input to physical high - outputs disabled */
	oe_gpio = device_get_binding(dev_cfg->oe_port);
	if (oe_gpio == NULL) {
		LOG_ERR("GPIO port %s not found", dev_cfg->oe_port);
		return -EINVAL;
	}

	ret = gpio_pin_set_raw(oe_gpio, dev_cfg->oe_pin, 1);
	if (ret != 0) {
		LOG_ERR("OE high");
		return -EINVAL;
	}

	if (gpio_pin_configure(oe_gpio, dev_cfg->oe_pin,
			       (GPIO_OUTPUT |
				DT_INST_GPIO_FLAGS(0, oe_gpios)))) {
		LOG_ERR("Unable to configure GPIO pin %u", dev_cfg->oe_pin);
		return -EINVAL;
	}
#endif /* DT_INST_X4HC595_DEV_HAS_OE_GPIOS(0) */

	/* Set RCLK input to physical low - initialise low */
	dev_data->rclk_gpio = device_get_binding(dev_cfg->rclk_port);
	if (dev_data->rclk_gpio == NULL) {
		LOG_ERR("GPIO port %s not found", dev_cfg->rclk_port);
		return -EINVAL;
	}

	ret = gpio_pin_set_raw(dev_data->rclk_gpio, dev_cfg->rclk_pin, 0);
	if (ret != 0) {
		LOG_ERR("SRCLK low");
		return -EINVAL;
	}

	if (gpio_pin_configure(dev_data->rclk_gpio, dev_cfg->rclk_pin,
			       (GPIO_OUTPUT |
				DT_INST_GPIO_FLAGS(0, rclk_gpios)))) {
		LOG_ERR("Unable to configure GPIO pin %u", dev_cfg->rclk_pin);
		return -EINVAL;
	}

	//ret = x4hc595_configure(dev);

	(void)memset(dev_data->data, 0, sizeof(dev_data->data));

	x4hc595_refresh(dev);

#if DT_INST_X4HC595_DEV_HAS_OE_GPIOS(0)
	/* Set OE input to physical low - outputs enabled */
	ret = gpio_pin_set_raw(oe_gpio, dev_cfg->oe_pin, 0);
	if (ret != 0) {
		LOG_ERR("OE low");
		return -EINVAL;
	}
#endif /* DT_INST_X4HC595_DEV_HAS_OE_GPIOS(0) */

	k_mutex_unlock(&dev_data->mutex);

	return ret;
}

static const struct x4hc595_config x4hc595_config = {
	.spi_port = DT_INST_BUS_LABEL(0),
	.spi_freq = DT_INST_PROP(0, spi_max_frequency),
	.spi_slave = DT_INST_REG_ADDR(0),
#if DT_INST_X4HC595_DEV_HAS_SRCLK_GPIOS(0)
	.srclk_pin = DT_INST_GPIO_PIN(0, srclk_gpios),
	.srclk_port = DT_INST_GPIO_LABEL(0, srclk_gpios),
#endif /* DT_INST_X4HC595_DEV_HAS_SRCLK_GPIOS(0) */
	.rclk_pin = DT_INST_GPIO_PIN(0, rclk_gpios),
	.rclk_port = DT_INST_GPIO_LABEL(0, rclk_gpios),
#if DT_INST_X4HC595_DEV_HAS_OE_GPIOS(0)
	.oe_pin = DT_INST_GPIO_PIN(0, oe_gpios),
	.oe_port = DT_INST_GPIO_LABEL(0, oe_gpios),
#endif /* DT_INST_X4HC595_DEV_HAS_OE_GPIOS(0) */
	.sr_cnt = DT_INST_PROP(0, sr_cnt),
};

static struct x4hc595_data x4hc595_data;

DEVICE_AND_API_INIT(x4hc595, DT_INST_LABEL(0), x4hc595_init,
		    &x4hc595_data, &x4hc595_config, POST_KERNEL,
		    CONFIG_X4HC595_INIT_PRIORITY, &x4hc595_api_funcs);

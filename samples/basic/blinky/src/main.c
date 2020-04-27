/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_HAS_NODE(LED0_NODE)
#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN	24//DT_GPIO_PIN(LED0_NODE, gpios)
#if DT_PHA_HAS_CELL(LED0_NODE, gpios, flags)
#define FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)
#endif
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0	""
#define PIN	23
#endif

#ifndef FLAGS
#define FLAGS	0
#endif

void main(void)
{
	struct device *dev;
	bool led_is_on = true;
	int ret;

	dev = device_get_binding("GPIO_X4HC595_P0");
	if (dev != NULL) {
		printk("Found GPIO_X4HC595_P0 driver\n");
	} else {
		printk("No GPIO_X4HC595_P0 driver\n");
	}

	ret = gpio_port_clear_bits_raw(dev, 0xFFFFFFFF);
	printk("No gpio_port_clear_bits_raw [%d]\n", ret);

	dev = device_get_binding(LED0);
	if (dev == NULL) {
		return;
	}

	ret = gpio_pin_configure(dev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return;
	}

	while (1) {
		gpio_pin_set(dev, PIN, (int)led_is_on);
		led_is_on = !led_is_on;
		k_msleep(SLEEP_TIME_MS);
	}
}

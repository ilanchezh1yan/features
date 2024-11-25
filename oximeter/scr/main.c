/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <nrfx_spim.h>
#include "AFE_Function.h"
extern char DataToTransmit[20];
extern int16_t graphCount;

struct computed data;

int main(void)
{
	data.data_available = false;
	printk("Hello World! %s\n", CONFIG_BOARD);
	DeviceBinding();
    spi_init();
	printk("e");
    AFE4490_Init();
	k_sleep(K_MSEC(2000));
	while(1)
	{
		GetSamples();
		if(data.data_available == true) {
			printk("%u, %u\r", data.SpO2, data.Heart_Rate);
			data.data_available = false;
			//k_sleep(K_SECONDS(2));
		}

	}

	return 0;
}

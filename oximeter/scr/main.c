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
#include "AFE4490.h"
#include "bluetooth.h"
#include "spi.h"

extern char DataToTransmit[20];
extern int16_t graphCount;

struct computed data;

int main(void)
{
	int ret;
	DeviceBinding();
	spi_init();
	AFE4490_Init();

	ret = ble_init();
	if(ret) {
		return 1;
	}

	k_sleep(K_MSEC(2000));
	while(1)
	{
		GetSamples();
	}

	return 0;
	
}

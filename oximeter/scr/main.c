/**
 * @file main.c
 * @brief Main application entry point for the PPG Recorder firmware.
 *
 * Initializes the SPI interface, AFE4490 sensor, and Bluetooth services,
 * then continuously acquires and processes photoplethysmography (PPG)
 * samples for heart rate and SpO₂ estimation.
 */
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

/** Global structure containing processed physiological measurements. */
struct computed data;

/**
 * @brief Entry point of the PPG Recorder firmware.
 *
 * Performs hardware initialization, configures the AFE4490 sensor,
 * enables Bluetooth communication, and continuously processes
 * incoming PPG samples.
 *
 * @return int Application status.
 *         - 0 : Normal termination
 *         - 1 : Bluetooth initialization failed
 */
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

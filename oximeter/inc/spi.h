/**
 * @file spi.h
 * @brief SPI interface definitions.
 *
 * Declares SPI initialization and data transfer functions used by
 * higher-level modules to communicate with the AFE4490 sensor.
 */
#ifndef SPI_H
#define SPI_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <nrfx_spim.h>

#define LED0_NODE DT_ALIAS(led0)

#if 	DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0	LED0_NODE
#define AFE_RESET	DT_GPIO_PIN(LED0_NODE, gpios)///Here wre are defining the Pin for AFE Reset
#define FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0	""
#define PIN	0
#define FLAGS	0
#endif

#define LED4_NODE DT_ALIAS(led4)

#if DT_NODE_HAS_STATUS(LED4_NODE, okay)
#define LED4	LED4_NODE
#define CS_SPI	DT_GPIO_PIN(LED4_NODE, gpios)///////////here we are defining the Pin for Chip Select CS_Pin for SPI communication with AFE
#define FLAGS1	DT_GPIO_FLAGS(LED4_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led4 devicetree alias is not defined"
#define LED1	""
#define PIN	0
#define FLAGS1	0
#endif

#if !DT_NODE_EXISTS(DT_NODELABEL(spi1))
#error "0oops"
#endif
#define SPI1 DT_NODELABEL(spi_1)

/**
 * @brief Initialize the SPI interface.
 *
 * Configures the SPI peripheral and associated GPIO pins required
 * for communication with the AFE4490 sensor.
 *
 * @return None.
 */
void spi_init(void);

/**
 * @brief Bind GPIO devices used by the application.
 *
 * Obtains references to GPIO peripherals and configures the
 * required pins for sensor control and interrupt handling.
 *
 * @return None.
 */
void DeviceBinding();

/**
 * @brief Transmit data through the SPI interface.
 *
 * Performs an SPI transfer to the connected peripheral device.
 *
 * @return None.
 */
void SPI_SEND(void);

/**
 * @brief Initialize and configure the AFE4490 sensor.
 *
 * Programs the AFE4490 registers, timing parameters, LED drive
 * settings, and ADC conversion windows required for PPG acquisition.
 *
 * @return None.
 */
void AFE4490_Init(void);

/**
 * @brief Write data to an AFE4490 register.
 *
 * Sends a register address and associated data value to the
 * AFE4490 using the SPI interface.
 *
 * @param reg Register address.
 * @param data Data value to be written.
 *
 * @return None.
 */
void AFE4490_Write(uint8_t reg, uint32_t data);

/**
 * @brief Read data from an AFE4490 register.
 *
 * Retrieves register contents from the AFE4490 through the
 * SPI interface.
 *
 * @param reg Register address.
 * @param data Pointer to the receive buffer.
 *
 * @return None.
 */
void AFE4490_Read(uint8_t reg, uint8_t *data);

#endif


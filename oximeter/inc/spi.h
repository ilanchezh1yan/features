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

void spi_init(void);
void DeviceBinding();
void SPI_SEND();
void AFE4490_Init();
void AFE4490_Write(uint8_t, uint32_t );
void AFE4490_Read (uint8_t, uint8_t *);

#endif


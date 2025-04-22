#ifndef PIN_DISCRIPTION_H
#define PIN_DISCRIPTION_H

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

#define D_RDY 20
#define ON_LED 2

#if !DT_NODE_EXISTS(DT_NODELABEL(spi1))
#error "0oops"
#endif
#define SPI1 DT_NODELABEL(spi_1)

#endif  

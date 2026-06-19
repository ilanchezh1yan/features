/**
 * @file uart.c
 * @brief UART communication driver.
 *
 * Implements serial communication functions used for debugging,
 * diagnostics, and monitoring of system data through an FTDI interface.
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include "uart.h"

static const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0)); 

/**
 * @brief Initialize the UART peripheral.
 *
 * Configures the UART interface with a baud rate of 115200 bps,
 * 8 data bits, no parity, and one stop bit.
 *
 * @return int UART initialization status.
 *         - 0 : Success
 *         - Non-zero : Configuration failed
 */
int uart_init(void) 
{
	int err = 0;
	const struct uart_config uart_cfg = {
		.baudrate = 115200,
    		.parity = UART_CFG_PARITY_NONE,
    		.stop_bits = UART_CFG_STOP_BITS_1,
    		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
    		.data_bits = UART_CFG_DATA_BITS_8
        };

	err = uart_configure(uart_dev, &uart_cfg);
    	/*if(err){
	    return err;
    	}*/ 
	return err;
}

/**
 * @brief Transmit data through the UART interface.
 *
 * Sends a sequence of bytes over UART using polling mode.
 * Each byte is transmitted sequentially through the UART
 * peripheral.
 *
 * @param data Pointer to the transmit buffer.
 * @param size Number of bytes to transmit.
 *
 * @return None.
 */
void send_data(uint8_t * data, uint8_t size)
{
	uint8_t i = 0;
	while(i < size) {
		uart_poll_out(uart_dev, data[i]);
		i++;
	}
}




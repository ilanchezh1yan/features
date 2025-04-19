#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include "uart.h"

static const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0)); 

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
    	if(err){
	    return err;
    	}
	return err;
}

void send_data(uint8_t * data, uint8_t size)
{
	uint8_t i = 0;
	while(i < size) {
		uart_poll_out(uart_dev, data[i]);
		i++;
	}
}




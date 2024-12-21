#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0)); 

int uart_init(void) 
{
	int err;
	struct uart_config uart_cfg;
	
	uart_cfg.baudrate = 115200;
    	uart_cfg.parity = UART_CFG_PARITY_NONE;
    	uart_cfg.stop_bits = UART_CFG_STOP_BITS_1;
    	uart_cfg.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
    	uart_cfg.data_bits = UART_CFG_DATA_BITS_8;

	err = uart_configure(uart_dev, &uart_cfg);
    	if(err){
	    return 1;
    	}
	return 0;
}

void send_data(unsigned char * data, unsigned char size)
{
	char i = 0;
	while(i < size) {
		uart_poll_out(uart_dev, data[i]);
		i++;
	}
}




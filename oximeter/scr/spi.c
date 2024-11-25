#include <stdint.h>
#include "SPI_Zephyr.h"

#define D_RDY 20

struct gpio_callback Data_ready;
volatile bool Data_computed;

void Data_ready_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pin)
{
	Data_computed = true;
}

void spi_init(void)
{
	printk(" SPI init\n");
	spi_dev = DEVICE_DT_GET(SPI1);
	if (spi_dev == NULL) {
		printk("Could not find SPI driver\n");
		return;
	}else printk("spi init success\n");

	k_sleep(K_MSEC(50));

		printk("C");
}

int spi_master_write(struct device * spi_dev, 
                     struct spi_config * spi_cfg,
                     uint8_t * data)
{
    struct spi_buf bufs = {
            .buf = data,
            .len = 4    //2
    };
    struct spi_buf_set tx = {
        .buffers = &bufs
    };

    tx.count = 1;
	int err;
    err= spi_write(spi_dev, spi_cfg, &tx);
		if (err) {
		printk("spi write error: %d\n", err);
		return false;
	}

}

int spi_master_write_Single(struct device * spi_dev, 
                     struct spi_config * spi_cfg,
                     uint16_t * data)
{
    struct spi_buf bufs = {
            .buf = data,
            .len = 1    //2
    };
    struct spi_buf_set tx = {
        .buffers = &bufs
    };

    tx.count = 1;
	int err;
    err= spi_write(spi_dev, spi_cfg, &tx);
		if (err) {
		return false;
	}

}

int spi_master_read(struct device * spi_dev, 
                    struct spi_config * spi_cfg,
                    uint8_t * data)
{
    struct spi_buf bufs = {
            .buf = data,
            .len = 3
    };
    struct spi_buf_set rx = {
        .buffers = &bufs
    };

    rx.count = 1;

		int err;
    err=  spi_read(spi_dev, spi_cfg, &rx);
	if (err) {
		return false;
	}
}
void DeviceBinding()
{
    int ret;
	dev = DEVICE_DT_GET(DT_NODELABEL(chip_select));
	if (dev == NULL) {
		return;
	}

	ret = gpio_pin_configure(dev, AFE_RESET, GPIO_OUTPUT_ACTIVE | FLAGS1);
	if (ret < 0) {
		return;
	}

	ret = gpio_pin_configure(dev, CS_SPI, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return;
	}

	ret = gpio_pin_configure(dev, D_RDY, GPIO_INPUT);
	if(ret) {
		return;
	}

	gpio_init_callback(&Data_ready, &Data_ready_cb, BIT(D_RDY));

	ret = gpio_add_callback(dev, &Data_ready);
	if(ret < 0) {
		return;
	}

	gpio_pin_set(dev, AFE_RESET, 1);
	k_sleep(K_MSEC(500));
	gpio_pin_set(dev, AFE_RESET, 0);
	k_sleep(K_MSEC(500));

	ret = gpio_pin_interrupt_configure(dev, D_RDY, 	GPIO_INT_EDGE_TO_ACTIVE);
	if(ret < 0) {
		return;
	}

}

void SPI_SEND()
{
        gpio_pin_set(dev, CS_SPI, 1);
        spi_master_write(spi_dev, &spi_cfg, &tx_data1);
        gpio_pin_set(dev, CS_SPI, 0);
}

void AFE4490_Write(uint8_t address,uint32_t SPIdataSend)
{
	gpio_pin_set(dev, CS_SPI, 1);//CS enable
	uint8_t AFEsend[] = {address,((SPIdataSend>>16)& 0xFF),((SPIdataSend>>8)& 0xFF),(SPIdataSend & 0xFF)};
	spi_master_write(spi_dev, &spi_cfg, (uint8_t*)AFEsend); 
	gpio_pin_set(dev, CS_SPI, 0);//CS enable
}

void AFE4490_Read (uint8_t address, uint8_t *rx_buff)
{
	
	gpio_pin_set(dev, CS_SPI, 1);//CS enable
    uint8_t AFEsend[] = {address & 0xFF};
 	spi_master_write_Single(spi_dev, &spi_cfg, (uint8_t*)AFEsend); 
	spi_master_read(spi_dev, &spi_cfg, rx_buff);
	gpio_pin_set(dev, CS_SPI, 0);//CS enable

}

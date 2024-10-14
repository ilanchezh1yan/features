#include "main.h"

#define DAC_12_RESOLUTION  4095.0
#define DAC_16_RESOLUTION  65535
#define REF_VOLTAGE     5000.0
#define VOLT_CONV_COEFF(x) (float)(REF_VOLTAGE / x)

#define MXP5010GP_TYP_OFFSET    200
#define MXP5010GP_SENSITIVITY   44.13

#define MXP7002DP_OFFSET       500
#define MXP7002DP_SENSITIVITY  1

#define SLAVE_ADDRESS 0x48
#define SLAVE_WRITE   SLAVE_ADDRESS << 1
#define SLAVE_READ    (SLAVE_ADDRESS << 1) | 1

extern DAC_HandleTypeDef hdac;
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c2;

uint16_t calculated_pressure(void);
uint16_t calculated_volume(uint16_t );
int32_t Read_Flow(uint8_t *);
void calculated_capacity(void);

double  flow;

int32_t Read_Flow(uint8_t * data)
{
	int32_t ret;
	uint8_t Config_Reg[3] = {0x01, 0x40, 0xE3};

	ret = HAL_I2C_Master_Transmit(&hi2c2,  SLAVE_WRITE, Config_Reg, 3, 100);
	if(ret != HAL_OK)
		return ret;

	Config_Reg[0] = 0x00;
	ret = HAL_I2C_Master_Transmit(&hi2c2, SLAVE_WRITE, Config_Reg, 1, 100);
	if(ret != HAL_OK)
		return ret;

	ret = HAL_I2C_Master_Receive(&hi2c2, SLAVE_READ, data, 2, 100);
	if(ret != HAL_OK)
		return ret;

	return -1;
}

uint16_t calculated_pressure(void)
{
	uint16_t Vlt;
	uint16_t Raw_value, pressure;
	float volt_coeff = VOLT_CONV_COEFF(DAC_12_RESOLUTION );

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1);
	Raw_value = HAL_ADC_GetValue(&hadc1);

	Vlt = (volt_coeff * (float)Raw_value) - MXP5010GP_TYP_OFFSET;

	pressure = (uint16_t)((float)Vlt / MXP5010GP_SENSITIVITY);

	return pressure;
}

uint16_t calculated_volume(uint16_t offset)
{
	uint32_t ret;
	uint16_t Raw_value ;
	uint16_t prv_read = 0;
	uint16_t volume;
	static uint16_t reading;
	uint16_t vlt;
	float dp;
	//int8_t flow_direction = 1;



	while(1) {
    ret = Read_Flow((uint8_t *)&Raw_value);
	if(ret != -1)
		return ret;

	//Raw_value = Raw_value << 8 | Raw_value >> 8;
	vlt = Raw_value * 6144.0 / 32767;
	reading = (1 - 0.15) * reading + (0.15 * vlt);
	if(prv_read > reading) {
			break;
	}
	dp = (reading - offset)/ 1000.0;

	flow = ((0.72*dp*dp*dp) - (8.01*dp*dp) + (56.7*dp) - 6.18);
	//flow = ((0.1512*dp*dp*dp) - (3.3424*dp*dp) + (41.657*dp));  // as per data sheet

	volume += (flow / 60.0) * 2;
	prv_read = reading;
	HAL_Delay(2);
}
	return volume;
}

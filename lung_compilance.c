#include "main.h"

#include "lung_compilance.h"
#include "Kalman_filter.h"
#include "DWT_Delay.h"

//#include <math.h>
extern HAL_TickFreqTypeDef uwTickFreq;
extern DAC_HandleTypeDef hdac;
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c1;
extern uint16_t dac;
extern uint16_t O2_dac;

float  flow, Max_flow;
float dp;
uint16_t vlt;
uint32_t Ticks;
volatile uint32_t exe_time;
float O2_vlt;
uint16_t count;

float Ti;
uint8_t RR;

#if 0
void kalman_filter_init()
{
	struct system_model Model ={
						.A = 1,
						.H = 1,
						.Q = 1,
						.R = 1};
}
#endif


int32_t Read_ads115(uint8_t * data, uint8_t i2c)
{
	int32_t ret;
	uint8_t Config_Reg[3] = {0x01, 0x40, 0xE3};
	I2C_HandleTypeDef *hal_i2c = NULL;
	switch(i2c) {
	case I2C_1:
		hal_i2c = &hi2c1;
		break;

	case I2C_2:
		hal_i2c = &hi2c2;
		break;

	default:
		break;
	}
	ret = HAL_I2C_Master_Transmit(hal_i2c,  SLAVE_WRITE, Config_Reg, 3, 100);
	if(ret != HAL_OK)
		return ret;

	Config_Reg[0] = 0x00;
	ret = HAL_I2C_Master_Transmit(hal_i2c, SLAVE_WRITE, Config_Reg, 1, 100);
	if(ret != HAL_OK)
		return ret;

	ret = HAL_I2C_Master_Receive(hal_i2c, SLAVE_READ, data, 2, 100);
	if(ret != HAL_OK)
		return ret;

	return -1;
}

float dp_to_flow(float dp){

	float flow;
	if(dp < 0){
		return 0;
	}
	//flow = (0.1512f * dp * dp * dp) - (3.3424f * dp * dp) + (41.657f * dp);
		flow = dp > 0.46 ? ((0.1512f * dp * dp * dp) - (3.3424f * dp * dp) + (41.657f * dp)) : ((0.72f * dp * dp * dp) - (8.01f * dp * dp) + (56.7f * dp) - 6.18f);
	//flow =  ((0.72f * dp * dp * dp) - (8.01f * dp * dp) + (56.7f * dp) - 6.18f);
	return flow;
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

float calculated_volume(uint16_t offset, uint16_t error, uint32_t Ti)
{
	count = 0;
	uint32_t ret;
	uint16_t Raw_value, i = 0;
	volatile float prv_read = 0;
	uint16_t noisy_vlt;
	float V = 0;

	static uint16_t reading = 0;
	struct parameters KF_param = {
					.estimate = offset,
					.predict = error
	};

	Max_flow = 0;

	if ( dac < 400){
		return 0;
	}

	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac);
	Ticks =  HAL_GetTick();

	if (Ti < HAL_MAX_DELAY)
	{
	    Ti += (uint32_t)(uwTickFreq);
	}
	while((HAL_GetTick() - Ticks) < Ti){

		ret = Read_ads115((uint8_t *)&Raw_value, I2C_2);
		if(ret != -1)
				return ret;
		Raw_value = Raw_value << 8 | Raw_value >> 8;
		Raw_value = Raw_value & 0xFFFB;

		noisy_vlt = Raw_value * (6144.0f / 32767.0f);

		DWT->CYCCNT =(volatile uint32_t)0;
		vlt = (uint16_t)Kalman_filter(noisy_vlt , &KF_param);
		exe_time = (volatile uint32_t)DWT->CYCCNT;

		reading = (1 - 0.2f) * reading + (0.2f * vlt);
		dp = ((float)vlt - (float)offset) / (float)100.0;
		flow = dp_to_flow(dp);

		if(count >= 3) {
		if(flow > 0)  {
			V += (float)(flow / 60.0f) * 3;
			count = 0;
			}
		}
		Max_flow = flow > Max_flow ? flow : Max_flow;

		if(prv_read > (float)0 && flow < (float)0 ) {
			ret = Read_ads115((uint8_t *)&Raw_value, I2C_1);
			Raw_value = Raw_value << 8 | Raw_value >> 8;
			O2_vlt = Raw_value * (6144.0f / 32767.0f);
			ret = HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
		}
		prv_read = flow;
		count++;
	}
	flow = Max_flow;
	return V;
}

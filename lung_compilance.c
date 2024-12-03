#include "main.h"

#include "lung_compilance.h"
#include "Kalman_filter.h"

double  flow;
double Max_flow, Max_flow_2;
double flow_2;
float dp;
uint16_t vlt;

uint16_t count;

#if 0
void kalman_filter_init()
{
	struct system_model Model;
	Model.A = 1;
	Model.H = 1;
	Model.Q = 1;
	Model.R = 1;
}
#endif


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

double dp_to_flow(float dp){

#if 1
	if(dp < 0.1){
		return 0.0;
	}
#endif

	double flow = ((0.72*dp*dp*dp) - (8.01*dp*dp) + (56.7*dp) - 6.18);
	flow_2 = ((0.1512*dp*dp*dp) - (3.3424*dp*dp) + (41.657*dp));  // as per data sheet
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

float calculated_volume(uint16_t offset, uint16_t error)
{
	count = 0;
	uint32_t ret;
	uint16_t Raw_value, sampling_rate = 1000;
	float prv_read = 0;
	float V = 0;

	float arr[5000] = {0};
	static uint16_t reading = 0;
	struct parameters KF_param = {
					.estimate = offset,
					.predict = error
	};


	//int8_t flow_direction = 1;

	Max_flow_2 = 0;
	Max_flow = 0;


	while(1){
		ret = Read_Flow((uint8_t *)&Raw_value);
		if(ret != -1)
			return ret;

		Raw_value = Raw_value << 8 | Raw_value >> 8;
		Raw_value = Raw_value & 0xFFFB;
		vlt = Raw_value * 6144.0 / 32767.0;

		vlt = (uint16_t)Kalman_filter(vlt , &KF_param);
		reading = (1 - 0.15) * reading + (0.15 * vlt);
		dp = ((float)reading - (float)offset) / (float)100.0;
		flow = dp_to_flow(dp);
		arr[count] = flow;
		V += (float)((flow / 60)) * 1.078;
		Max_flow = flow > Max_flow ? flow : Max_flow;
		Max_flow_2 = flow_2 > Max_flow_2 ? flow_2 : Max_flow_2;

		if(prv_read > (float)0 && flow <= (float)0 ) {
			flow = Max_flow;
			break;
		}

		prv_read = flow;
		count++;
		sampling_rate--;
	}
	return V;
}

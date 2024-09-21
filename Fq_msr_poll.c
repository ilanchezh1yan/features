
#define FREQUENCY HAL_RCC_GetPCLK1Freq()

struct signal {
	uint16_t frequency;
};

 ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

uint32_t captureValue;
uint32_t previousCaptureValue;
uint32_t frequency;
struct signal signal;

void frequency_read(TIM_HandleTypeDef *);
  
void frequency_read(TIM_HandleTypeDef *htim) {
	uint16_t Period;
	if (TIM2->SR & TIM_SR_CC1IF)
	{
	   TIM2->SR &= ~(TIM_SR_CC1IF);
	   captureValue = TIM2->CCR1;
	   if(captureValue == previousCaptureValue)
        	return;

       Period = captureValue < previousCaptureValue ? (0xffffffff - previousCaptureValue) + captureValue : captureValue - previousCaptureValue;

       signal.frequency = FREQUENCY / Period;
       previousCaptureValue = captureValue;
    }
}
#include <stdint.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include "bluetooth.h"
#include "AFE4490.h"
#include "spi.h"
#include "pin_discription.h"
#include "uart.h"

static void estimate_spo2_hr(uint16_t *, int32_t, uint16_t *, uint8_t *, int8_t *, uint8_t *, int8_t *);
static uint8_t chksum8(const unsigned char *, size_t);
static uint16_t find_max(uint16_t *, uint16_t);
static uint16_t find_min(uint16_t *, uint16_t);
static void kalman_filter(uint16_t *, uint16_t);
static uint16_t find_peak(uint16_t *, uint16_t *, uint16_t);

extern volatile bool Data_computed;
extern const struct device *dev;
extern struct computed data;
extern volatile uint8_t BT_notify_enable;

static uint16_t IR_LED_Graph[BUFFER_SIZE];
static uint16_t RED_LED_Graph[BUFFER_SIZE];
static uint16_t filterd_ir_signal[BUFFER_SIZE];
static uint16_t inverted_ir_signal[BUFFER_SIZE]; 
static uint16_t inverted_red_signal[BUFFER_SIZE];

static float KF_process_noice_covariance = 0.01f;

void AFE4490_Init()
{
	  AFE4490_Write(CONTROL0, 0x000000);
	  AFE4490_Write(CONTROL0, 0x000008);
	  AFE4490_Write(TIAGAIN, 0x000000); 
	  AFE4490_Write(TIA_AMB_GAIN, 0x000001);
	  AFE4490_Write(LEDCNTRL, 0x011414);
	  AFE4490_Write(CONTROL2, 0x010000); 
	  AFE4490_Write(CONTROL1, 0x010707);
	  AFE4490_Write(PRPCOUNT, 0X001F3F);
	  AFE4490_Write(LED2STC, 0X001770);
	  AFE4490_Write(LED2ENDC, 0X001F3E);
	  AFE4490_Write(LED2LEDSTC, 0X001770);
	  AFE4490_Write(LED2LEDENDC, 0X001F3F);
	  AFE4490_Write(ALED2STC, 0X000000);
	  AFE4490_Write(ALED2ENDC, 0X0007CE);
	  AFE4490_Write(LED2CONVST, 0X000002);
	  AFE4490_Write(LED2CONVEND, 0X0007CF);
	  AFE4490_Write(ALED2CONVST, 0X0007D2);
	  AFE4490_Write(ALED2CONVEND, 0X000F9F);
	  AFE4490_Write(LED1STC, 0X0007D0);
	  AFE4490_Write(LED1ENDC, 0X000F9E);
	  AFE4490_Write(LED1LEDSTC, 0X0007D0);
	  AFE4490_Write(LED1LEDENDC, 0X000F9F);
	  AFE4490_Write(ALED1STC, 0X000FA0);
	  AFE4490_Write(ALED1ENDC, 0X00176E);
	  AFE4490_Write(LED1CONVST, 0X000FA2);
	  AFE4490_Write(LED1CONVEND, 0X00176F);
	  AFE4490_Write(ALED1CONVST, 0X001772);
	  AFE4490_Write(ALED1CONVEND, 0X001F3F);
	  AFE4490_Write(ADCRSTCNT0, 0X000000);
	  AFE4490_Write(ADCRSTENDCT0, 0X000000);
	  AFE4490_Write(ADCRSTCNT1, 0X0007D0);
	  AFE4490_Write(ADCRSTENDCT1, 0X0007D0);
	  AFE4490_Write(ADCRSTCNT2, 0X000FA0);
	  AFE4490_Write(ADCRSTENDCT2, 0X000FA0);
	  AFE4490_Write(ADCRSTCNT3, 0X001770);
	  AFE4490_Write(ADCRSTENDCT3, 0X001770);
}


void GetSamples()
{
	bool read_data = false;
	int ret;
	static uint8_t brc;
        static uint8_t graphCount;
	uint8_t spo2_valid = 0, hr_valid = 0;
	uint8_t ir_rx_buf[4] = {0};
	uint8_t red_rx_buf[4] = {0};
	struct ble_packet Live_graph = {
		.header1 = 0xBE,
		.header2 = 0xFF,
		.length = 0x09,
	};

	if(Data_computed == true) {	
		ret = gpio_pin_interrupt_configure(dev, D_RDY, 	GPIO_INT_DISABLE);
		if(ret < 0) {
			return;
		}

		AFE4490_Write(CONTROL0, 0x000001);
		AFE4490_Read(LED1ABSVAL, ir_rx_buf);
		AFE4490_Write(CONTROL0, 0x000001);
		AFE4490_Read(LED2ABSVAL, red_rx_buf);
		read_data = true;
	}

	if(read_data == true) {
		ir_rx_buf[0] = ir_rx_buf[0] & 0b00111111;
		data.raw_IR = ir_rx_buf[0] << 16 | ir_rx_buf[1] << 8 | ir_rx_buf[2];

		red_rx_buf[0] = red_rx_buf[0] & 0b00111111;
		data.raw_red = red_rx_buf[0] << 16 | red_rx_buf[1] << 8 | red_rx_buf[2];

		if (brc == SAMPLING_RATE) {
			IR_LED_Graph[graphCount] = (uint16_t)(data.raw_IR >> 4);
			RED_LED_Graph[graphCount] = (uint16_t)(data.raw_red >> 4);
			graphCount++;
			brc = 0;
		}
		
	 	Live_graph.IR_msb = ir_rx_buf[0];
		Live_graph.IR_mid = ir_rx_buf[1];
		Live_graph.IR_lsb = ir_rx_buf[2];

		Live_graph.RED_msb = red_rx_buf[0];
		Live_graph.RED_mid = red_rx_buf[1];
		Live_graph.RED_lsb = red_rx_buf[2];

		Live_graph.SpO2 = data.SpO2;

		Live_graph.HR = data.Heart_Rate;
	        Live_graph.CRC =  chksum8(&Live_graph.IR_lsb, 8);
#if 0
		if(BT_notify_enable && brc &&  !(brc % 5)) {	
			BT_send((uint8_t *)&Live_graph, sizeof(Live_graph));
		}
#endif
		if (brc % 2) {
			send_data((uint8_t *)&Live_graph, sizeof(Live_graph));
			
		}

		if(graphCount > BUFFER_SIZE - 1) {
			estimate_spo2_hr(IR_LED_Graph, BUFFER_SIZE, RED_LED_Graph, &(data.SpO2), &spo2_valid, &(data.Heart_Rate), &hr_valid);
			memmove(IR_LED_Graph, (void *)&IR_LED_Graph[(uint32_t)(MOVING_BUFFER_SIZE)], (BUFFER_SIZE - MOVING_BUFFER_SIZE) * sizeof(uint16_t));
			memmove(RED_LED_Graph, (void *)&RED_LED_Graph[(uint32_t)(MOVING_BUFFER_SIZE)], (BUFFER_SIZE - MOVING_BUFFER_SIZE) * sizeof(uint16_t)); 
			graphCount = BUFFER_SIZE - MOVING_BUFFER_SIZE;
			data.data_available = true;
		}

		Data_computed = false;
		read_data = false;
		brc++;

		ret = gpio_pin_interrupt_configure(dev, D_RDY, 	GPIO_INT_EDGE_TO_ACTIVE);
		if(ret < 0) {
			return;
		}
 		  
	}
}

static void estimate_spo2_hr(uint16_t *pure_ir_buffer, int32_t n_ir_buffer_length, uint16_t *pure_red_buffer, uint8_t *pn_spo2, int8_t *pch_spo2_valid, uint8_t *pn_heart_rate, int8_t *pch_hr_valid)
{
  uint8_t k, valid_ratio = 0;

  uint16_t peaks;    
  uint16_t ir_max, red_max;
  uint16_t ir_min, red_min;  
  uint16_t red_ac, ir_ac;
  uint16_t red_dc, ir_dc;
  
  uint32_t nume, denom;
  uint32_t peak_interval_sum;

  float ratio = 0;
  float Tot_peak_interval_sum;

  uint16_t peak_locs[NUMBER_LOCAL_PEAKS] = {0};

  ir_max = find_max(pure_ir_buffer, BUFFER_SIZE);
  red_max = find_max(pure_red_buffer, BUFFER_SIZE);

  for (k=0 ; k<n_ir_buffer_length ; k++ ) {  
    inverted_ir_signal[k] = ir_max - pure_ir_buffer[k];
    inverted_red_signal[k] = red_max - pure_red_buffer[k]; 
  }

  for(k = 0; k < BUFFER_SIZE - WINDOW_SIZE; k++) {
    for(int j = 0; j < WINDOW_SIZE; j++) {
        filterd_ir_signal[k] += inverted_ir_signal[k + j];
    }
    filterd_ir_signal[k] /= WINDOW_SIZE; 
  }

  peaks = find_peak(filterd_ir_signal, peak_locs, BUFFER_SIZE);
  KF_process_noice_covariance = peaks > 12 ? 1.0f : 0.01f ;
 
  peak_interval_sum =0;

  if (peaks > 1){
    for (k = 1; k < peaks; k++) peak_interval_sum += (peak_locs[k] - peak_locs[k -1] ) ;
    Tot_peak_interval_sum = (float)peak_interval_sum/(float)(peaks - 1.0f);
    *pn_heart_rate = (uint8_t)((SAMPLING_FREQ * 60.0f)/ Tot_peak_interval_sum);
    *pch_hr_valid  = 1;
  }

for(k = 0; k < peaks - 1; k++) {
  	ir_min = (ir_max + find_min(&inverted_ir_signal[peak_locs[k]], (peak_locs[k + 1] - peak_locs[k])));
  	red_min = (red_max + find_min(&inverted_red_signal[peak_locs[k]], (peak_locs[k + 1] - peak_locs[k]))) ;

  	ir_max = (ir_max + find_max(&inverted_ir_signal[peak_locs[k]], (peak_locs[k + 1] - peak_locs[k]))); 
  	red_max = (red_max + find_max(&inverted_red_signal[peak_locs[k]], (peak_locs[k + 1] - peak_locs[k])));

  	red_dc = (red_max + red_min) / 2;
  	ir_dc = (ir_max + ir_min) / 2;

  	red_ac = red_max - red_min;  
  	ir_ac = ir_max - ir_min; 
  
  	if(red_dc && red_ac) nume =  red_ac * ir_dc; 
  	if(ir_dc && ir_ac) denom =  ir_ac * red_dc;
  
  	if(nume && denom) {
		ratio += (float)nume / (float)denom;
		valid_ratio++;
	}
  }	
	
  if(ratio > 0.0f && valid_ratio) ratio /= (float)(valid_ratio);
  *pn_spo2 = ratio > 0.75f ? 127.05f - 41.238f * ratio : 128.5f - 41.238f * ratio;
  if(*pn_spo2 > 100) *pn_spo2 = 98; 
	
  return;
  
}

static void kalman_filter(uint16_t *signal, uint16_t Data_length) 
{
	uint16_t KF_state_estimation = signal[0];
	float KF_estimated_covariance = 1, covariance_predict, KF_gain;
	uint16_t measured, state_predict;

	for(int index = 0; index < Data_length; index++) {
		measured = signal[index];
		
		state_predict = KF_state_estimation;
		covariance_predict = KF_estimated_covariance + KF_process_noice_covariance;

		KF_gain = covariance_predict / (covariance_predict + KF_R_FACTOR);
		KF_state_estimation = state_predict + KF_gain * ( measured - state_predict);
		KF_estimated_covariance = (1 - KF_gain) *  covariance_predict; 
		  
		signal[index] = KF_state_estimation; 
	}
}

static uint16_t find_peak(uint16_t *Signal, uint16_t *Peak_interval, uint16_t Data_length)
{
	uint16_t PeakWindow[PEAK_DETECTION_WINDOW_SIZE] = {0};
	uint8_t MovingWindowCount  = 0;
	uint16_t MovingWindowSum = 0;
	uint8_t lastpeak = 0, peak_count = 0;

	kalman_filter(Signal, Data_length);
	
	for(int index = 0; index < Data_length; index++) {
		MovingWindowSum += Signal[index];
		MovingWindowCount++;
		if(MovingWindowCount > MOVING_WINDOW_SIZE) {
			MovingWindowCount = 0;
			memmove(&PeakWindow[1], PeakWindow, (PEAK_DETECTION_WINDOW_SIZE - 1) * sizeof(uint16_t));  
			PeakWindow[0] = MovingWindowSum / (MovingWindowCount + 1);
			MovingWindowSum = 0;
			if(lastpeak > SMALLEST_PEAK_INTERVAL)  {
				if((PeakWindow[PEAK_WINDOW_CENTER_INDEX - 2] < PeakWindow[PEAK_WINDOW_CENTER_INDEX]) && 													   					   (PeakWindow[PEAK_WINDOW_CENTER_INDEX + 2] < PeakWindow[PEAK_WINDOW_CENTER_INDEX])) {
					if( peak_count < NUMBER_LOCAL_PEAKS - 1) {
						Peak_interval[peak_count++] = index;
						lastpeak = 0; 
                                        }
				}
			}
			lastpeak++;
		}
		
	}
	return peak_count;

}

static uint8_t chksum8(const unsigned char *buff, size_t len)
{
    unsigned int sum;
    for ( sum = 0 ; len != 0 ; len-- )
        sum += *(buff++);
    return (uint8_t)sum;
}

static uint16_t find_max(uint16_t *data, uint16_t size)
{
	uint16_t max = 0;
	while(size--){
		if(data[size] > max) max = data[size];
	}
	return max;
}

static uint16_t find_min(uint16_t *data, uint16_t size)
{
	uint16_t min = -1;
	while(size--){
		if(data[size] < min) min = data[size];
		
	}
	return min;
}



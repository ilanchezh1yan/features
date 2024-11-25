#include <stdint.h>
#include <stdlib.h>
#include "AFE_Function.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include "string.h"
#include "stdio.h"
#include "inttypes.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "arm_common_tables.h"
#include "dsp/transform_functions.h"
#include "dsp/filtering_functions.h"
#include "math.h"

#include "arm_math_types.h"
#include "arm_math_memory.h"

#include "dsp/none.h"
#include "dsp/utils.h"

#include "dsp/support_functions.h"
#include "dsp/fast_math_functions.h"

#define CONTROL0    0x00
#define LED2STC     0x01
#define LED2ENDC    0x02
#define LED2LEDSTC    0x03
#define LED2LEDENDC   0x04
#define ALED2STC    0x05
#define ALED2ENDC   0x06
#define LED1STC     0x07
#define LED1ENDC    0x08
#define LED1LEDSTC    0x09
#define LED1LEDENDC   0x0a
#define ALED1STC    0x0b
#define ALED1ENDC   0x0c
#define LED2CONVST    0x0d
#define LED2CONVEND   0x0e
#define ALED2CONVST   0x0f
#define ALED2CONVEND  0x10
#define LED1CONVST    0x11
#define LED1CONVEND   0x12
#define ALED1CONVST   0x13
#define ALED1CONVEND  0x14
#define ADCRSTCNT0    0x15
#define ADCRSTENDCT0  0x16
#define ADCRSTCNT1    0x17
#define ADCRSTENDCT1  0x18
#define ADCRSTCNT2    0x19
#define ADCRSTENDCT2  0x1a
#define ADCRSTCNT3    0x1b
#define ADCRSTENDCT3  0x1c
#define PRPCOUNT    0x1d
#define CONTROL1    0x1e
#define SPARE1      0x1f
#define TIAGAIN     0x20
#define TIA_AMB_GAIN  0x21
#define LEDCNTRL    0x22
#define CONTROL2    0x23
#define SPARE2      0x24
#define SPARE3      0x25
#define SPARE4      0x26
#define SPARE4      0x26
#define RESERVED1   0x27
#define RESERVED2   0x28
#define ALARM     0x29
#define LED2VAL     0x2a
#define ALED2VAL    0x2b
#define LED1VAL     0x2c
#define ALED1VAL    0x2d
#define LED2ABSVAL    0x2e
#define LED1ABSVAL    0x2f
#define DIAG      0x30
#define count 60

#define CES_CMDIF_PKT_START_1   0x0A
#define CES_CMDIF_PKT_START_2   0xFA
#define CES_CMDIF_TYPE_DATA   0x02
#define CES_CMDIF_PKT_STOP    0x0B

#define BUFFER_SIZE 500
#define WINDOW_SIZE  10
#define min(x,y) ((x) < (y) ? (x) : (y))

#define D_RDY 20

extern volatile bool Data_computed;
extern const struct device *dev;
extern struct computed data;

static uint16_t IR_LED_Graph[BUFFER_SIZE]; 
static uint16_t RED_LED_Graph[BUFFER_SIZE];
static int32_t an_x[BUFFER_SIZE]; 
static int32_t an_y[BUFFER_SIZE];
static uint8_t brc;
static uint16_t graphCount;

const uint8_t uch_spo2_table[184]={ 95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99, 
              99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 
              100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97, 
              97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91, 
              90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81, 
              80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67, 
              66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50, 
              49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29, 
              28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5, 
              3, 2, 1 } ;

void AFE4490_Init()
{
	  AFE4490_Write(CONTROL0, 0x000000);
	  AFE4490_Write(CONTROL0, 0x000008);
	  AFE4490_Write(TIAGAIN, 0x000000); // CF = 5pF, RF = 500kR
	  AFE4490_Write(TIA_AMB_GAIN, 0x000001);
	  AFE4490_Write(LEDCNTRL, 0x001414);
	  AFE4490_Write(CONTROL2, 0x000000); // LED_RANGE=100mA, LED=50mA
	  AFE4490_Write(CONTROL1, 0x010707); // Timers ON, average 3 samples
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
	uint8_t spo2_valid = 0, hr_valid = 0;
	uint8_t ir_rx_buf[4] = {0};
	uint8_t red_rx_buf[4] = {0};
	//unsigned long ir_rx_buf = 0;
	//unsigned long red_rx_buf = 0;
	
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
		
		printf("%u, %u\r", (data.raw_IR),(data.raw_red));
		if(brc == 20) {
		 	IR_LED_Graph[graphCount] = (uint16_t)(data.raw_IR >> 4);
		 	RED_LED_Graph[graphCount] = (uint16_t)(data.raw_red >> 4);
		 	graphCount++;
		 	brc = 0; 
		}

		brc++;

		if(graphCount > BUFFER_SIZE - 1) {
			estimate_spo2(IR_LED_Graph, BUFFER_SIZE, RED_LED_Graph, &(data.SpO2), &spo2_valid, &(data.Heart_Rate), &hr_valid);
			graphCount = 0;
			data.data_available = true;
		}

		Data_computed = false;
		read_data = false;

		ret = gpio_pin_interrupt_configure(dev, D_RDY, 	GPIO_INT_EDGE_TO_ACTIVE);
		if(ret < 0) {
			return;
	 	}
	}
}

void estimate_spo2(uint16_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint16_t *pun_red_buffer, uint8_t *pn_spo2, int8_t *pch_spo2_valid, uint8_t *pn_heart_rate, int8_t *pch_hr_valid)
{
  uint32_t un_ir_mean;
  int32_t k, n_i_ratio_count;
  int32_t i, n_exact_ir_valley_locs_count, n_middle_idx;
  int32_t n_th1, n_npks;   
  int32_t an_ir_valley_locs[15] ;
  int32_t n_peak_interval_sum;
  
  int32_t n_y_ac, n_x_ac;
  int32_t n_spo2_calc; 
  int32_t n_y_dc_max, n_x_dc_max; 
  int32_t n_y_dc_max_idx, n_x_dc_max_idx; 
  float an_ratio[5]; 
  int32_t n_ratio_average; 
  int32_t n_nume, n_denom ;

  // calculates DC mean and subtract DC from ir
  un_ir_mean =0; 
  for (k=0 ; k<n_ir_buffer_length ; k++ ) un_ir_mean += pun_ir_buffer[k] ;
  un_ir_mean =un_ir_mean/n_ir_buffer_length ;
    
  // remove DC and invert signal so that we can use peak detector as valley detector
  for (k=0 ; k<n_ir_buffer_length ; k++ ) {  
    an_x[k] = -1*(pun_ir_buffer[k] - un_ir_mean) ;
  }
    
    
  // 4 pt Moving Average
  for(k=0; k< BUFFER_SIZE - WINDOW_SIZE ; k++){
    an_x[k]=( an_x[k]+an_x[k+1]+ an_x[k+2]+ an_x[k+3] + an_x[k+4] + an_x[k+5] + an_x[k+6] + an_x[k+7] +an_x[k+8] + an_x[k+9]) / (int)WINDOW_SIZE;     
    //an_x[k]=( an_x[k]+an_x[k+1]+ an_x[k+2]+ an_x[k+3])/(int)4;   
  }
  // calculate threshold  
  n_th1=0; 
  for ( k=0 ; k<BUFFER_SIZE ;k++){
    n_th1 +=  an_x[k];
  }
  n_th1=  n_th1/ ( BUFFER_SIZE);

  for ( k=0 ; k<15;k++) an_ir_valley_locs[k]=0;
  // since we flipped signal, we use peak detector as valley detector
  find_peak( an_ir_valley_locs, &n_npks, an_x, BUFFER_SIZE, n_th1, 4, 15 );//peak_height, peak_distance, max_num_peaks 
  n_peak_interval_sum =0;
  if (n_npks>=2){
    for (k=1; k<n_npks; k++) n_peak_interval_sum += (an_ir_valley_locs[k] -an_ir_valley_locs[k -1] ) ;
    n_peak_interval_sum =n_peak_interval_sum/(n_npks-1);
    *pn_heart_rate =(uint8_t)( (25 * 60)/ n_peak_interval_sum );
    *pch_hr_valid  = 1;
  }
  else  { 
    *pn_heart_rate = -1; // unable to calculate because # of peaks are too small
    *pch_hr_valid  = 0;
  }

  //  load raw value again for SPO2 calculation : RED(=y) and IR(=X)
  for (k=0 ; k<n_ir_buffer_length ; k++ )  {
      an_x[k] =  pun_ir_buffer[k] ; 
      an_y[k] =  pun_red_buffer[k] ; 
  }

  // find precise min near an_ir_valley_locs
  n_exact_ir_valley_locs_count = n_npks; 
  
  //using exact_ir_valley_locs , find ir-red DC andir-red AC for SPO2 calibration an_ratio
  //finding AC/DC maximum of raw

  n_ratio_average =0; 
  n_i_ratio_count = 0; 
  for(k=0; k< 5; k++) an_ratio[k]=0;
  for (k=0; k< n_exact_ir_valley_locs_count; k++){
    if (an_ir_valley_locs[k] > BUFFER_SIZE ){
      *pn_spo2 =  -1 ; // do not use SPO2 since valley loc is out of range
      *pch_spo2_valid  = 0; 
      return;
    }
  }

  // find max between two valley locations 
  // and use an_ratio betwen AC compoent of Ir & Red and DC compoent of Ir & Red for SPO2 
  for (k=0; k< n_exact_ir_valley_locs_count-1; k++){
    n_y_dc_max= -16777216 ; 
    n_x_dc_max= -16777216; 
    if (an_ir_valley_locs[k+1]-an_ir_valley_locs[k] >3){
        for (i=an_ir_valley_locs[k]; i< an_ir_valley_locs[k+1]; i++){
          if (an_x[i]> n_x_dc_max) {n_x_dc_max =an_x[i]; n_x_dc_max_idx=i;}
          if (an_y[i]> n_y_dc_max) {n_y_dc_max =an_y[i]; n_y_dc_max_idx=i;}
      }
      n_y_ac= (an_y[an_ir_valley_locs[k+1]] - an_y[an_ir_valley_locs[k] ] )*(n_y_dc_max_idx -an_ir_valley_locs[k]); //red
      n_y_ac=  an_y[an_ir_valley_locs[k]] + n_y_ac/ (an_ir_valley_locs[k+1] - an_ir_valley_locs[k])  ; 
      n_y_ac=  an_y[n_y_dc_max_idx] - n_y_ac;    // subracting linear DC compoenents from raw 
      n_x_ac= (an_x[an_ir_valley_locs[k+1]] - an_x[an_ir_valley_locs[k] ] )*(n_x_dc_max_idx -an_ir_valley_locs[k]); // ir
      n_x_ac=  an_x[an_ir_valley_locs[k]] + n_x_ac/ (an_ir_valley_locs[k+1] - an_ir_valley_locs[k]); 
      n_x_ac=  an_x[n_y_dc_max_idx] - n_x_ac;      // subracting linear DC compoenents from raw 
      n_nume=( n_y_ac *n_x_dc_max); //prepare X100 to preserve floating value
      n_denom= ( n_x_ac *n_y_dc_max);
      if (n_denom>0  && n_i_ratio_count <5 &&  n_nume != 0)
      {   
         an_ratio[n_i_ratio_count]= (float)n_nume / (float)n_denom ; //formular is ( n_y_ac *n_x_dc_max) / ( n_x_ac *n_y_dc_max) ;
        an_ratio[n_i_ratio_count] = 110.0 - 25.0 * an_ratio[n_i_ratio_count]; 
        n_i_ratio_count++;
      }
    }
  }
  // choose median value since PPG signal may varies from beat to beat
  sort_ascend((int32_t *)an_ratio, n_i_ratio_count);
  n_middle_idx = n_i_ratio_count % 2;

  if (!n_middle_idx)
    n_ratio_average =( an_ratio[n_middle_idx-1] +an_ratio[n_middle_idx])/2; // use median
  else
    n_ratio_average = an_ratio[n_middle_idx ];

  *pn_spo2 = (uint8_t)n_ratio_average;
}


void find_peak( int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num )
{
  find_peak_above( pn_locs, n_npks, pn_x, n_size, n_min_height );
  remove_close_peaks( pn_locs, n_npks, pn_x, n_min_distance );
  *n_npks = min( *n_npks, n_max_num );
}

void  find_peak_above( int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height )
{
  int32_t i = 1, n_width;
  *n_npks = 0;

  while (i < n_size - 1) {
    if (pn_x[i] > n_min_height && pn_x[i] > pn_x[i - 1]) {   // find left edge of potential peaks
      n_width = 1;
      while (i + n_width < n_size && pn_x[i] == pn_x[i + n_width]) // find flat peaks
        n_width++;
      if (pn_x[i] > pn_x[i + n_width] && (*n_npks) < 15 ) {   // find right edge of peaks
        pn_locs[(*n_npks)++] = i;
        // for flat peaks, peak location is left edge
        i += n_width + 1;
      }
      else
        i += n_width;
    }
    else
      i++;
  }
}

void  remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance)
{

  int32_t i, j, n_old_npks, n_dist;

  /* Order peaks from large to small */
  sort_indices_descend( pn_x, pn_locs, *pn_npks );

  for ( i = -1; i < *pn_npks; i++ ) {
    n_old_npks = *pn_npks;
    *pn_npks = i + 1;
    for ( j = i + 1; j < n_old_npks; j++ ) {
      n_dist =  pn_locs[j] - ( i == -1 ? -1 : pn_locs[i] ); // lag-zero peak of autocorr is at index -1
      if ( n_dist > n_min_distance || n_dist < -n_min_distance )
        pn_locs[(*pn_npks)++] = pn_locs[j];
    }
  }

  // Resort indices int32_to ascending order
  sort_ascend( pn_locs, *pn_npks );
}

void sort_ascend(int32_t  *pn_x, int32_t n_size)
{
  int32_t i, j, n_temp;
  for (i = 1; i < n_size; i++) {
    n_temp = pn_x[i];
    for (j = i; j > 0 && n_temp < pn_x[j - 1]; j--)
      pn_x[j] = pn_x[j - 1];
    pn_x[j] = n_temp;
  }
}

void sort_indices_descend(  int32_t  *pn_x, int32_t *pn_indx, int32_t n_size)
{
  int32_t i, j, n_temp;
  for (i = 1; i < n_size; i++) {
    n_temp = pn_indx[i];
    for (j = i; j > 0 && pn_x[n_temp] > pn_x[pn_indx[j - 1]]; j--)
      pn_indx[j] = pn_indx[j - 1];
    pn_indx[j] = n_temp;
  }
}
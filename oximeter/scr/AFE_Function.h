struct computed {
    uint8_t SpO2;
    uint8_t Heart_Rate;
    uint32_t raw_IR;
    uint32_t raw_red;
    volatile uint8_t data_available;
};

void AFE4490_Init();
void LED1_read();
void GetSamples();
float GetFFT_DC_AC();
void HeartRateCal();
void TimeIntervalHeartRate();
void ADC_sorting();
void ADC_Average();
void AFE4490_Write(uint8_t ,uint32_t);
void AFE4490_Read (uint8_t, uint8_t *);
void spi_init(void);
void DeviceBinding();
static float powers_of(float , uint8_t);

void estimate_spo2(uint16_t *, int32_t, uint16_t *, uint8_t *, int8_t *, uint8_t *, int8_t *);
void find_peak( int32_t *, int32_t *,  int32_t  *, int32_t, int32_t, int32_t, int32_t);
void find_peak_above( int32_t *, int32_t *,  int32_t  *, int32_t, int32_t);
void remove_close_peaks(int32_t *, int32_t *, int32_t *, int32_t);
void sort_ascend(int32_t  *, int32_t);
void sort_indices_descend(  int32_t  *, int32_t *, int32_t);

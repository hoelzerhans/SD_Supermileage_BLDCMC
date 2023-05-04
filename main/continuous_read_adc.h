#ifndef CONTINUOUS_READ_ADC_H_
#define CONTINUOUS_READ_ADC_H_

//#include "continuous_read_adc.h"
//ISR
//static bool IRAM_ATTR sensing_adc_conversion_done(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data);

//ADC continuous read and conversion
void continuous_adc_results_task(void *arg);
//ADC results printing
void continuous_adc_print_task(void *arg);

///Run ADC
void ADC_RUN(void);

void sensing_setEnableADCThrottle(int enabled);

#endif
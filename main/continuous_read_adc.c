
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "esp_timer.h"
#include "math.h"
#include <stdlib.h>
#include "driver/gpio.h"

//#include "continuous_read_adc.h"

//PROCESSED SENSOR VALUE
float proc_voltage = 0.0;
float actual_voltage = 0.0;
//RAW SENSOR VALUES
uint16_t raw_voltage = 0;
uint16_t throttle = 0;

float raw_a_current = 0;
float Raw_a_voltage = 0.0;

uint16_t raw_b_current = 0;
float Raw_b_voltage = 0.0;

uint16_t raw_c_current = 0;
float Raw_c_voltage = 0.0;

///Thermistor parameters
float raw_a_temp = 0;
float raw_b_temp = 0;
float raw_c_temp = 0;

float ThermistorR1;
float ThermistorR2;
float ThermistorR3;

float TK1;
float TF1;
float TC1;

float TK2;
float TF2;
float TC2;

float TK3;
float TF3;
float TC3;
/// Thermistor parameters end here

float CurrentA;
float CurrentB;
float CurrentC;

/// Current Cali variable
const float alpha = 0.0001;
double last_a_Avg =0.0;
double last_b_Avg =0.0;
double last_c_Avg =0.0;

//This handle is used to control the continuous ADC instance
adc_continuous_handle_t continuous_adc_handle = NULL;
//This task is where continuous ADC results are read once notified that the conversions are complete
static TaskHandle_t continuous_adc_results_task_handle = NULL;        
//These are the ADC channels that will be read (and the order they will be read in)
static adc_channel_t channel[8] = {ADC_CHANNEL_7, ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6};
//Used for some logging, could be removed if logging statements are also removed
static const char *TAG = "";


//This callback informs the continuous_adc_results_task that there are new ADC results to read
static bool IRAM_ATTR sensing_adc_conversion_done(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(continuous_adc_results_task_handle, &mustYield);
    return (mustYield == pdTRUE);
}


//This is the task that runs after the vTaskNotifyGiveFromISR() is run from the sensing_adc_conversion_done() interrupt
void continuous_adc_results_task(void *arg) { 
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[8*SOC_ADC_DIGI_RESULT_BYTES] = {0};
    memset(result, 0xcc, 8*SOC_ADC_DIGI_RESULT_BYTES);
       while(1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while (1) {
            ret = adc_continuous_read(continuous_adc_handle, result, 8*SOC_ADC_DIGI_RESULT_BYTES, &ret_num, 1);  ////continuous reading the ADC
            if (ret == ESP_OK) {
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                    adc_digi_output_data_t *p = (void*)&result[i];
                    if (p->type2.channel == 0) {
                        raw_voltage = p->type2.data; ////GPIO 11
                        proc_voltage = (3.3333/4095)*(float)raw_voltage; //Example of processing a raw value into a float
                        actual_voltage = (proc_voltage*270.0)/15.0;

                    }else if (p->type2.channel == 1) {
                        throttle = p->type2.data;  ////GPIO12  used for Speed control 

                    //---------------------------------------------------------------------------------------//
                    }else if (p->type2.channel == 2) {  ///GPIO 13                    
                     float curr_a_avg = 0.0;
                    raw_a_current = p->type2.data;    
                    curr_a_avg = (raw_a_current*alpha) + ((last_a_Avg)*(1-alpha));
                    last_a_Avg = curr_a_avg;
                    //printf("the value of raw a current is %f\n", raw_a_current);
                    //vTaskDelay(250);
                    Raw_a_voltage = ((curr_a_avg/4095.0))*5.0; //Calculate the raw voltage      
                    CurrentA = (((Raw_a_voltage*1.0615)-2.5)/0.05); 
                         if(CurrentA < 0){   ////futhur callibrate the negative current to make it accurate
                             CurrentA = CurrentA * 0.90;
                         } 
                    //---------------------------------------------------------------------------------------//
                    }else if (p->type2.channel == 3) {
                    
                    float curr_b_avg = 0.0;
                    raw_b_current = p->type2.data;  ////GPIO 14   
                    curr_b_avg = (raw_b_current*alpha) + ((last_b_Avg)*(1-alpha));
                    last_b_Avg = curr_b_avg;
                    //printf("the value of raw b current is %f\n", raw_b_current);
                    //vTaskDelay(250);
                    Raw_b_voltage = ((curr_b_avg/4095.0))*5.0; //Calculate the raw voltage      
                    CurrentB = (((Raw_b_voltage*1.0615)-2.5)/0.05); 
                         if(CurrentB < 0){   ////futhur callibrate the negative current to make it accurate
                             CurrentB = CurrentB * 0.90;
                         } 
                    //----------------------------------------------------------------------------------------//
                    }else if (p->type2.channel == 4) {
                    float curr_c_avg = 0.0;
                    raw_c_current = p->type2.data;  ////GPIO 15   
                    curr_c_avg = (raw_c_current*alpha) + ((last_c_Avg)*(1-alpha));
                    last_c_Avg = curr_c_avg;
                    //printf("the value of raw b current is %f\n", raw_b_current);
                    //vTaskDelay(250);
                    Raw_c_voltage = ((curr_c_avg/4095.0))*5.0; //Calculate the raw voltage      
                    CurrentC = (((Raw_c_voltage*1.0615)-2.5)/0.05); 
                         if(CurrentC < 0){   ////futhur callibrate the negative current to make it accurate
                             CurrentC = CurrentC * 0.90;
                         } 

                    }else if (p->type2.channel == 5) {    
                        raw_a_temp = p->type2.data;    /////GPIO 16
                        ThermistorR1 = 9892.0 * ((4095.0/raw_a_temp) -1); ////this formula convert to the real thermistor resistor
                       float To1 = (1.0/298.15) + (1.0/3950.0)*log(ThermistorR1/100000.00);
                         TK1 = 1/To1; ////This is the temperature in K
                         TC1 = TK1- 273.15; ///Convert to Celcius 
                         TF1 = TC1*1.8 + 32.0;   // convert to Fahrenheit


                    }else if (p->type2.channel == 6) {
                        raw_b_temp = p->type2.data;    /////GPIO 17
                        ThermistorR2 = 9892.0 * ((4095.0/raw_b_temp) -1); ////this formula convert to the real thermistor resistor
                       float To2 = (1.0/298.15) + (1.0/3950.0)*log(ThermistorR2/100000.00);
                         TK2 = 1/To2; ////This is the temperature in K
                         TC2 = TK2- 273.15; ///Convert to Celcius 
                         TF2 = TC2*1.8 + 32.0;   // convert to Fahrenheit


                    }else if (p->type2.channel == 7) {
                        raw_c_temp = p->type2.data;    /////GPIO 18
                        ThermistorR3 = 9892.0 * ((4095.0/raw_c_temp) -1); ////this formula convert to the real thermistor resistor
                        float To3 = (1.0/298.15) + (1.0/3950.0)*log(ThermistorR3/100000.00);
                        TK3 = 1/To3; ////This is the temperature in K
                        TC3 = TK3- 273.15; ///Convert to Celcius 
                        TF3 = TC3*1.8 + 32.0;   // convert to Fahrenheit
                    };

                }
            } else if (ret == ESP_ERR_TIMEOUT) {
                break;
            }
        }
    }
}


//Task that periodically prints values to console. Great place to experiment.
void continuous_adc_print_task(void *arg) { 
    while(1) {
         
        //ESP_LOGI(TAG, "voltage: %f\tActual Voltage: %f", proc_voltage,actual_voltage);
        ESP_LOGI(TAG, "voltage a %f\tCurrent a%f", Raw_a_voltage,CurrentA);
        //ESP_LOGI(TAG, "voltage b %f\tCurrent b%f", Raw_b_voltage,CurrentB);
        //ESP_LOGI(TAG, "voltage c %f\tCurrent c%f", Raw_c_voltage,CurrentC);
        //ESP_LOGI(TAG, " Temperature a in K : %f\tTemperature a in F: %f",TK1,TF1 );
        //ESP_LOGI(TAG, " Temperature b in K : %f\tTemperature b in F: %f",TK2,TF2 );
        //ESP_LOGI(TAG, " Temperature c in K : %f\tTemperature c in F: %f",TK3,TF3 );
        vTaskDelay(250);
    }    
}


void ADC_RUN(void)
{
    //Start the task that will handle processing sensor data incoming from DMA
    xTaskCreate(continuous_adc_results_task, "continuous_adc_results_task", 4096, NULL, 11, &continuous_adc_results_task_handle); 
    /////The first parameter is address of the function which execute, when this task is scheduled to run! Re-read the sentence once. 
    /////The second parameter hello_task is the name given to the task. The could be any name. 
    /////It used to obtain handle or while debugging the task. The next parameter 2048 is the memory allocated to the task in word (2 Bytes).

    //Start the task that will occasionally print DMA data to console
    xTaskCreate(continuous_adc_print_task, "continuous_adc_print_task", 4096, NULL, 5, NULL);
    //Create and setup the continuous_adc_handle object
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,  
        .conv_frame_size = 8*SOC_ADC_DIGI_RESULT_BYTES, ////// 8 bytes * 2 Bytes 
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &continuous_adc_handle));
    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 9 * 8 * 1000,
        .conv_mode = ADC_CONV_SINGLE_UNIT_2,  /////This is using the ADC2 for conversion
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };
    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = sizeof(channel) / sizeof(adc_channel_t);
    for (int i = 0; i < sizeof(channel) / sizeof(adc_channel_t); i++) {
        adc_pattern[i].atten = ADC_ATTEN_DB_11;
        adc_pattern[i].channel = channel[i];
        adc_pattern[i].unit = ADC_UNIT_2;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(continuous_adc_handle, &dig_cfg));
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = sensing_adc_conversion_done,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(continuous_adc_handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(continuous_adc_handle));
}
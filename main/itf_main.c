/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
/*
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

#define SD_PRINTS_DEF 1
#define SD_WRITE_PRINTS_DEF 1
#define COM_PRINT_DEF 1
//#define TEST_SD_WRITE_DEF 1

#include "itf_master_defines.h"
#include "itf_seven_seg.h"
#include "itf_com_funcs.h"
#include "itf_sd_card_writer.h"
#include "itf_sd_card_setup.h"

void writeHexTask(void * params){
    itf_initHex();
    while(1){
        int i;
        for(i=0;i<16;i++){
            itf_displayHex(i);
            vTaskDelay(500/portTICK_PERIOD_MS);
        }
    }
}

void PCComTask(void * params){
    itf_init_UART0();
    itf_initHex();
    
    int lastLength = -1;

    while(1){
        int length;
        uart_get_buffered_data_len(UART_NUM_0,(size_t*) &length);
           
        if(lastLength != length){
            ESP_LOGI("Info:","Data in buffer is %d long",length);
            lastLength = length;
        }
        
        uint8_t* data = (uint8_t*) malloc(7);
        const int rxBytes = uart_read_bytes(UART_NUM_0, data, 7, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI("RX_TASK_TAG", "Read %d bytes: '%s'", rxBytes, data);
            int messageVal = itf_decodePC(data);
            ESP_LOGI("PC","Decoded message %x",messageVal);
            if(itf_checkCRC(messageVal) != -1){
                itf_actOnMessage(messageVal,1);
            }else{
                u_int messageVal_itf_crc4 = itf_addCRC(messageVal);
                ESP_LOGI("PC","Correct CRC format would be %x",messageVal_itf_crc4);
            }
        }
        vTaskDelay(50/portTICK_PERIOD_MS);
    }
}

void MCUComTask(void * params){
    itf_init_UART1();
    itf_initHex();
    
    int lastLength = -1;

    while(1){
        int length;
        uart_get_buffered_data_len(UART_NUM_1,(size_t*) &length);
        
        uint8_t* data = (uint8_t*) malloc(2);
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, 2, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI("MCU", "Read %d bytes: '%x''%x'", rxBytes, data[0],data[1]);
            
            int messageVal = (data[0]<<8) | (data[1]);
            ESP_LOGI("MCU","Decoded message %x",messageVal);
            if(itf_checkCRC(messageVal) != -1){
                itf_actOnMessage(messageVal,0);
            }else{
                u_int messageVal_itf_crc4 = itf_addCRC(messageVal);
                ESP_LOGI("MCU","Correct CRC format would be %x",messageVal_itf_crc4);
            }
        }
        vTaskDelay(50/portTICK_PERIOD_MS);
    }
}

//Test task emulating a part of program adding data to SD queue.
void dataRecordTask(void * params){
    static char *str1 = "YahYeetgoGetemCowboy1YahYeetgoGetemCowboy2YahYeetgoGetemCowboy3YahYeetgoGetemCowboy4 \n";
    while(1){
        itf_addToSD(str1,86);
       //itf_addToSD("str1",86);
        //itf_addToSD("str1",86);
        vTaskDelay(50/portTICK_PERIOD_MS);
        ESP_LOGI("TestMessage","AddtoSD ran");
    }
}

void forceBufferTestTask(void * params){
    static char *str1 = "ForceBufferTest \n";
    while(1){
        itf_addToSD(str1,17);
        itf_forceWriteBuffers_FLAG =1;
        vTaskDelay(5000/portTICK_PERIOD_MS);
    }
}

void app_main_dummy(void)
{
    itf_initDirPins();
    itf_initHex();
    //xTaskCreate(dataRecordTask,"DataTask",1024*20,NULL,configMAX_PRIORITIES-1,NULL);
    xTaskCreate(MCUComTask,"MCUTask",1024*20,NULL,configMAX_PRIORITIES,NULL);
    xTaskCreate(itf_writeSD_task,"SDTask",1024*50,NULL,configMAX_PRIORITIES-2,NULL);
    xTaskCreate(PCComTask,"PCTask",1024*20,NULL,configMAX_PRIORITIES-1,NULL);
    //xTaskCreate(writeHexTask,"HexTask",1024*20,NULL,configMAX_PRIORITIES,NULL);\]
    //int xd = itf_businit;
}
*/
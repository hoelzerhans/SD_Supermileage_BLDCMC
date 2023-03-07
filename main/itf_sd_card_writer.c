/* SD card and FAT filesystem example.
   This example uses SPI peripheral to communicate with SD card.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_log.h"
#include "itf_sd_card_setup.h"

static const char *TAGW = "Writer:";

#ifndef ITF_SD_DEFINES
    #define ITF_MOUNT_POINT "/sdcard"
    #define ITF_SD_WRITE_SIZE 1024 //Power of 2 Pref (1024, 512, ect)
    #define ITF_SD_BUFFER_SIZE ITF_SD_WRITE_SIZE*2
#endif

//#define SD_WRITE_PRINTS_DEF in main to enable prints

int SD_buffer0_index = 0;
int SD_buffer1_index = 0;

char SD_buffer0[ITF_SD_BUFFER_SIZE];
char SD_buffer1[ITF_SD_BUFFER_SIZE];

static int bufferInUse = 0;
static int SDbufferMaxed = 0;

//Set to 1 to force write the buffers to data out
int itf_forceWriteBuffers_FLAG = 0;

int itf_forceWriteBuffers(void);
void itf_writeTestMessage(char *str);
int itf_writeFileFromBuffer(int bufferNum);
int itf_addToSD(char *toStore,int length);
int itf_addTestToSD(int testNum);

//A simple task for writing to the SD when buffers fill up
void itf_writeSD_task(void * params)
{
    int error = 1;
    error = itf_initSD();

    while(1){
        if(error==0){
            //itf_turnoffSD();
            if(itf_initSD()==1){
                error = 1;
            }
        }else{
            if(itf_forceWriteBuffers_FLAG){
                error = itf_forceWriteBuffers();
                itf_forceWriteBuffers_FLAG = 0;
            }
            if(SD_buffer0_index >= ITF_SD_BUFFER_SIZE){
                error = itf_writeFileFromBuffer(0);
                if(error)
                    SD_buffer0_index = 0;
            }
            if(SD_buffer1_index >= ITF_SD_BUFFER_SIZE){
                error = itf_writeFileFromBuffer(1);
                if(error)
                    SD_buffer1_index = 0;
            }
        }
        vTaskDelay(50/portTICK_PERIOD_MS);
    }
    itf_turnoffSD();
}

//Use itf_forceWriteBuffers_FLAG var to trigger this
int itf_forceWriteBuffers(void){
    const char *file_hello = ITF_MOUNT_POINT"/data.txt";
    #ifdef SD_WRITE_PRINTS_DEF
        ESP_LOGI("ForceWriteBuffers", "Opening file %s", file_hello);
    #endif
    FILE *f = fopen(file_hello, "a");

    if (f == NULL) {
        #ifdef SD_WRITE_PRINTS_DEF
            ESP_LOGE("ForceWriteBuffers", "Failed to open file for writing");
        #endif
        return 0;
    }
    char data_to_send[1];
    int i;
    for(i=0;i<SD_buffer0_index;i++){
        data_to_send[1] = SD_buffer0[i];
        fwrite(data_to_send,1,1,f);
    }
    SD_buffer0_index = 0;

    for(i=0;i<SD_buffer1_index;i++){
        data_to_send[1] = SD_buffer1[i];
        fwrite(data_to_send,1,1,f);
    }
    SD_buffer1_index = 0;

    fclose(f);

    return 1;
}

//Writes to test document, dont use unless you ARENT using other write funcs
void itf_writeTestMessage(char *str){
    #ifndef TEST_SD_WRITE_DEF
        return;
    #endif

    itf_initSD();

    const char *file_hello = ITF_MOUNT_POINT"/Tests.txt";
    #ifdef SD_WRITE_PRINTS_DEF
        ESP_LOGI("itf_writeTestMessage", "Opening file %s", file_hello);
    #endif
    FILE *f = fopen(file_hello, "a");

    if (f == NULL) {
        #ifdef SD_WRITE_PRINTS_DEF
            ESP_LOGE("itf_writeTestMessage", "Failed to open file for writing");
        #endif
        return;
    }

    fprintf(f,str);
    fclose(f);
    return;
}

//Write all the bytes in specified buffer number to data.txt, this takes 
//hundreds of ms, so be careful when calling
int itf_writeFileFromBuffer(int bufferNum){
    int startTime = esp_log_timestamp();
    const char *file_hello = ITF_MOUNT_POINT"/data.txt";
    int startTime2 = 0;
    int endTime2 = 0;
    #ifdef SD_WRITE_PRINTS_DEF
        ESP_LOGI("itf_writeFileFromBuffer", "Opening file %s", file_hello);
    #endif
    FILE *f = fopen(file_hello, "a");
    if (f == NULL) {
        #ifdef SD_WRITE_PRINTS_DEF
            ESP_LOGE("itf_writeFileFromBuffer", "Failed to open file for writing");
        #endif
        return 0;
    }

    int i = 0;
    char data_to_send[ITF_SD_WRITE_SIZE];

    if(bufferNum == 0){
        startTime2 = esp_log_timestamp();
        for(i=0;i<ITF_SD_BUFFER_SIZE;i+=ITF_SD_WRITE_SIZE){
            int k;
            for(k=0;k<ITF_SD_WRITE_SIZE;k++){
                data_to_send[k] = SD_buffer0[i+k];
            }
            fwrite(data_to_send,1,ITF_SD_WRITE_SIZE,f);
        }
    }else{
        startTime2= esp_log_timestamp();
        for(i=0;i<ITF_SD_BUFFER_SIZE;i+=ITF_SD_WRITE_SIZE){
            int k;
            for(k=0;k<ITF_SD_WRITE_SIZE;k++){
                data_to_send[k] = SD_buffer1[i+k];
            }
            fwrite(data_to_send,1,ITF_SD_WRITE_SIZE,f);
        }
    }
    endTime2 = esp_log_timestamp();
    fclose(f);
    int endTime = esp_log_timestamp();
    int timeTotal = endTime - startTime;
    int timeTotal2 = endTime2 - startTime2;
    #ifdef SD_WRITE_PRINTS_DEF
        ESP_LOGI(TAGW, "Time to open/write bufferNum %d was %d",bufferNum,timeTotal);
        ESP_LOGI(TAGW, "Time to write bufferNum %d was %d",bufferNum,timeTotal2);
    #endif
    return 1;
}

//Put a byte array in and specify length, and this func will 
//handle all the buffering and deciding when to send data to the SD
int itf_addToSD(char *toStore,int length){
    int i;

    if((ITF_SD_BUFFER_SIZE < SD_buffer0_index + length) && (ITF_SD_BUFFER_SIZE < SD_buffer1_index + length)){
        #ifdef SD_WRITE_PRINTS_DEF
        ESP_LOGE("itf_addToSD","BUFFERS ARE MAXED, AHHHHHHHH");
        #endif
        return 0;
    }

    for(i=0;i<length;i++){
        if(bufferInUse == 0 && SDbufferMaxed){
            bufferInUse = 1;
            SDbufferMaxed = 0;
        }
        if(bufferInUse == 1 && SDbufferMaxed){
            bufferInUse = 0;
            SDbufferMaxed = 0;
        }
        if(bufferInUse == 0){
            SD_buffer0[SD_buffer0_index] = toStore[i];
            SD_buffer0_index++;
            if(SD_buffer0_index >=ITF_SD_BUFFER_SIZE)
                SDbufferMaxed = 1;
        }else if(bufferInUse == 1){
            SD_buffer1[SD_buffer1_index] = toStore[i];
            SD_buffer1_index++;
            if(SD_buffer1_index >=ITF_SD_BUFFER_SIZE)
                SDbufferMaxed = 1;
        }
    }
    #ifdef SD_WRITE_PRINTS_DEF
        ESP_LOGI("BufferStatus","Buffer0: %d   Buffer1: %d",SD_buffer0_index,SD_buffer1_index);
    #endif
    return 1;
}

//WIP, dont use 
int itf_addTestToSD(int testNum){
    return -1;
}
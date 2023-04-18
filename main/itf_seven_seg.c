#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "itf_seven_seg.h"

#ifndef ITF_HEX_DEFINES
    #define ITF_A_PIN 40
    #define ITF_B_PIN 39
    #define ITF_C_PIN 36 
    #define ITF_D_PIN 37
    #define ITF_E_PIN 38
    #define ITF_F_PIN 42
    #define ITF_G_PIN 41
#endif

void itf_displayHex(int num);
void itf_initHex(void);

//Sets up all the pinouts based on defines in the file
void itf_initHex(void){
    static const int segPinArray[] = {ITF_A_PIN,ITF_B_PIN,ITF_C_PIN,ITF_D_PIN,ITF_E_PIN,ITF_F_PIN,ITF_G_PIN};
    int p=0;
    for(p=0;p<7;p++){
        int gpioNum = segPinArray[p];
        gpio_config_t io_conf = {};
        //disable interrupt
        io_conf.intr_type = GPIO_INTR_DISABLE;
        //set as output mode
        io_conf.mode = GPIO_MODE_OUTPUT;
        //bit mask of the pins that you want to set,e.g.GPIO18/19
        uint64_t bitMask = (((uint64_t)1)<<gpioNum);
        io_conf.pin_bit_mask = bitMask;//GPIO X
        //io_conf.pin_bit_mask = (1<<gpioNum);
        //disable pull-down mode
        io_conf.pull_down_en = 0;
        //disable pull-up mode
        io_conf.pull_up_en = 0;
        //configure GPIO with the given settings
        gpio_config(&io_conf);
        gpio_set_level(gpioNum,1);

        //GPIO_MODE_
        //gpio_set_pull_mode(gpioNum,GPIO_PULLDOWN_DISABLE);
        //gpio_set_pull_mode(gpioNum,GPIO_PULLUP_DISABLE);
        //gpio_set_direction(gpioNum,GPIO_MODE_OUTPUT);
        //gpio_set_level(gpioNum,1);
        //gpio_set_level(gpioNum,0);
        //gpio_drive_cap_t s;
        //GPIO_DRIVE_CAP_1
        //gpio_get_drive_capability(gpioNum,&s);
        //ESP_LOGI("Setpin:","Pin %d",gpioNum);
    }
    ESP_LOGI("Hex","Init!");
}

//Write a number between 0 and 15 to the seven segment
void itf_displayHex(int num){
    static const int segPinArray[] = {ITF_A_PIN,ITF_B_PIN,ITF_C_PIN,ITF_D_PIN,ITF_E_PIN,ITF_F_PIN,ITF_G_PIN};
    static const uint8_t segLUT[] = {
        0b00000011,//0
        0b10011111,//1
        0b00100101,//2
        0b00001101,//3
        0b10011001,//4
        0b01001001,//5
        0b01000001,//6
        0b00011111,//7
        0b00000001,//8
        0b00001001,//9
        0b00010001,//A
        0b11000001,//B
        0b01100011,//C
        0b10000101,//D
        0b01100001,//E
        0b01110001//F
    };

    if(num > 15 || num <0){
        return;
    }

    int p=0;
    for(p=0;p<7;p++){
        int gpioNum = segPinArray[6-p];
        int state = segLUT[num] & (1<<(p+1));
        if(state > 0){
            state = 1;
        }
        gpio_set_level(gpioNum,state);
        //ESP_LOGI("SetDisp:","Pin %d, Set %d",gpioNum,state);
        //ESP_LOGI("SetDisp:","Pin %d, Got %d",gpioNum,gpio_get_level(gpioNum));
    }

}

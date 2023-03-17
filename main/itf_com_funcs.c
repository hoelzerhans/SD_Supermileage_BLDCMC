#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "itf_seven_seg.h"
#include "itf_sd_card_writer.h"
//#ifndef CTRL_SUBSYSTEM_H_
#include "ctrl_subsystem.h"
//#endif
#include "itf_master_defines.h"

#ifndef ITF_COM_DEFINES
    #define ITF_TX_PIN_UART1 4
    #define ITF_RX_PIN_UART1 5
    #define ITF_DIR0_PIN 19
    #define ITF_DIR1_PIN 20
    #define ITF_RX_BUF_SIZE_UART0 1024
    #define ITF_TX_BUF_SIZE_UART0 1024
    #define ITF_RX_BUF_SIZE_UART1 1024
    #define ITF_TX_BUF_SIZE_UART1 1024
#endif

//#define COM_PRINT_DEF
/*
static const int ITF_RX_BUF_SIZE_UART0 = 1024;
static const int ITF_TX_BUF_SIZE_UART0 = 1024;
static const int ITF_RX_BUF_SIZE_UART1 = 1024;
static const int ITF_TX_BUF_SIZE_UART1 = 1024;
*/


int itf_dirInput0 = -1;
int itf_dirInput1 = -1;
int itf_speedLocked = 0;


int itf_decodePC(uint8_t* str);
int itf_checkCRC(int message);
int itf_addCRC(int message);
int itf_actOnMessage(int message,int source);
void itf_init_UART0(void);
void itf_init_UART1(void);
uint8_t itf_crc4(uint8_t c, uint64_t x, int bits);
int itf_sendDataMCU(const char* data);
void itf_dirHandler(void *arg);
void itf_initDirPins(void);
int itf_crc4AndSend(int message);

//Init the UART0 (PC->MCU) with baud rate of 19200
void itf_init_UART0(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,//19200
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM_0, ITF_RX_BUF_SIZE_UART0 * 2, ITF_TX_BUF_SIZE_UART0 *2, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

//Init the UART1 (MCU->MCU) with baud rate of 19200
void itf_init_UART1(void) {
    const uart_config_t uart_config = {
        .baud_rate = 19200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM_1, ITF_RX_BUF_SIZE_UART1 * 2, ITF_TX_BUF_SIZE_UART1*2, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, ITF_TX_PIN_UART1, ITF_RX_PIN_UART1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

//Utils for itf_crc4
static const uint8_t itf_crc4_tab[] = {
	0x0, 0x7, 0xe, 0x9, 0xb, 0xc, 0x5, 0x2,
	0x1, 0x6, 0xf, 0x8, 0xa, 0xd, 0x4, 0x3,
};

//itf_crc4 function 
uint8_t itf_crc4(uint8_t c, uint64_t x, int bits)
{
	int i;
	/* mask off anything above the top bit */
	x &= (1ull << bits) - 1;
	/* Align to 4-bits */
	bits = (bits + 3) & ~0x3;
	/* Calculate itf_crc4 over four-bit nibbles, starting at the MSbit */
	for (i = bits - 4; i >= 0; i -= 4)
		c = itf_crc4_tab[c ^ ((x >> i) & 0xf)];
	return c;
}

//Decode raw char data in "0xFFFF" format into numbers,return message
int itf_decodePC(uint8_t* str){
    #ifdef COM_PRINT_DEF
        ESP_LOG_BUFFER_HEXDUMP("EnteredData", str, 7, ESP_LOG_INFO);
    #endif
    if(!(str[0] == 0x30 && str[1] == 0x78 && str[6]==0x0d)){
        #ifdef COM_PRINT_DEF
            ESP_LOGI("PC_ERROR","Problem decoding PC message");
        #endif
        return -1;
    }
    int i;
    int message=0;
    for(i=0;i<4;i++){
        int currentChar = str[2+i];
        int charValue = 0;
        if(currentChar == 0x30){
            charValue = 0;
        }else if(currentChar == 0x31){
            charValue = 1;
        }else if(currentChar == 0x32){
            charValue = 2;
        }else if(currentChar == 0x33){
            charValue = 3;
        }else if(currentChar == 0x34){
            charValue = 4;
        }else if(currentChar == 0x35){
            charValue = 5;
        }else if(currentChar == 0x36){
            charValue = 6;
        }else if(currentChar == 0x37){
            charValue = 7;
        }else if(currentChar == 0x38){
            charValue = 8;
        }else if(currentChar == 0x39){
            charValue = 9;
        }else if(currentChar == 0x41){
            charValue = 10;
        }else if(currentChar == 0x42){
            charValue = 11;
        }else if(currentChar == 0x43){
            charValue = 12;
        }else if(currentChar == 0x44){
            charValue = 13;
        }else if(currentChar == 0x45){
            charValue = 14;
        }else if(currentChar == 0x46){
            charValue = 15;
        }
        message |= charValue<<(4*(3-i));
    }
    //ESP_LOGI("PC","Decoded message '%s' as '%x'",str,message);
    return message;
}

//Check itf_crc4 of message and compare with calculated, return CRC if passed, -1 if wrong
int itf_checkCRC(int message){
    int mCRC = message & 0x000F;
    int calcCRC = (int) itf_crc4(1,(uint16_t) (message & 0xFFF0),16);
    if(mCRC == calcCRC){
        #ifdef COM_PRINT_DEF
            ESP_LOGI("itf_crc4","Message and CRC match, %d",mCRC);
        #endif
        return mCRC;
    }else{
        #ifdef COM_PRINT_DEF
            ESP_LOGI("itf_crc4","CRC Mismatch! Message: %d, Calc: %d",mCRC,calcCRC);
        #endif
        return -1;
    }
}

//Enter as 16bit message with important parts in MSBs, 0xFFF0 where F is info
int itf_addCRC(int message){
    int calcCRC = (int) itf_crc4(1,(uint16_t) (message & 0xFFF0),16);
    int messAndCRC = ((message & 0xFFF0) | (calcCRC & 0x000F)); //0xFA60 | 0x0004 -> 0xFA64
    return messAndCRC;
}

//Contains all the info about what to do for each message. Source=0 MCU->MCU   Source=1 PC->MCU
int itf_actOnMessage(int message,int source){
    int packetID = (message & 0xF000) >>12;
    int packetDATA = (message & 0x0FF0) >>4;

    char *c = "Response";
    #ifdef COM_PRINT_DEF
        ESP_LOGI("AoM","Packet ID is %d",packetID);
    #endif
    
    switch (packetID){
        case 0: //Speed set
            //set speed
            if(itf_speedLocked == 0){
                ctrl_setThrottle((uint16_t) (packetDATA*16));
            }
            break;
        case 1: //Speed set mph
            //set speed mph
            float mphCalc = 10+0.17647*packetDATA;
            if(itf_speedLocked == 0){
                ctrl_setSpeedControl(mphCalc);
            }
            break;
        case 2: //Enable/disable speed set
            //Lock/Unlock
            itf_speedLocked = (packetDATA > 0);//1 = locked, 0 = unlocked
            break;
        case 3: //Dir set & dir lock
            //unlock/lock and set dir
            //00 = NOT RUNNING, 01 = FORWARD, 10 = BACKWARD, 11 = NOT RUNNING
            if(packetDATA < 4){
                ctrl_setDirection(packetDATA);
            }
            break;
        case 4: //Req temp
            int tempWanted = packetDATA & (0x03);//ABC
            int temp_DUMMY = 0;

            if(tempWanted == 0x00){
                temp_DUMMY = ctrl_getPhaseTempA_f();
            }else if(tempWanted == 0x01){
                temp_DUMMY = ctrl_getPhaseTempB_f();
            }else if(tempWanted == 0x02){
                temp_DUMMY = ctrl_getPhaseTempC_f();
            }
            temp_DUMMY = temp_DUMMY*1;//CHANGE TO PROPER SCALING

            if(source == 0){
                int message = (packetID << 12) | (temp_DUMMY << 4);
                message = itf_addCRC(message);
                uint8_t data[2] = {(uint8_t) ((message & 0xFF00)>>8),(uint8_t) (message & 0x00FF)};
                itf_sendDataMCU((char*)data);
            }else{
                ESP_LOGI(c,"Temp = %d",temp_DUMMY);
            }
            break;
        case 5://Req MPH
            int MPH_encoded = (int) ctrl_getSpeed_mph();
            if(source == 0){
                int message = (packetID<<12) | (MPH_encoded <<4);
                itf_crc4AndSend(message);
            }else{
                 ESP_LOGI(c,"MPH = %d",MPH_encoded);
            }
            //Stuff
            break;
        case 6://Req CurrA
            int CurrA_encoded = (int) ctrl_getCurrent_A();
            if(source == 0){
                int message = (packetID<<12) | (CurrA_encoded <<4);
                itf_crc4AndSend(message);
            }else{
                 ESP_LOGI(c,"CurrA = %d",CurrA_encoded);
            }
            //Stuff
            break;
        case 7://Req BatVol
            int BattV_encoded = (int) ctrl_getBatVolts_V();
            if(source == 0){
                int message = (packetID<<12) | (BattV_encoded <<4);
                itf_crc4AndSend(message);
            }else{
                 ESP_LOGI(c,"MPH = %d",BattV_encoded);
            }
            //Stuff
            break;
        case 14://Read Direction request (TEST MSG, DONT USE IN ACTUAL)
            if(source == 0){
                itf_writeTestMessage("Dir Test Performed via MCU \n");
                //SEND ADC DATA VIA MCU
                int message = 0xE000 | itf_dirInput0<<5 | itf_dirInput1<<4;
                message = itf_addCRC(message);
                uint8_t data[2] = {(uint8_t) ((message & 0xFF00)>>8),(uint8_t) (message & 0x00FF)};
                itf_sendDataMCU((char*)data);
            }else{
                ESP_LOGI(c,"D0=%d   D1=%d",itf_dirInput0,itf_dirInput1);
                itf_writeTestMessage("Dir Test Performed via PC \n");
            }
            break;   
        case 15://Set hex display (TEST MSG, DONT USE IN ACTUAL)
            itf_displayHex((message & 0x00F0)>>4);
            if(source == 1){
                ESP_LOGI(c,"Hex test message received");
                itf_writeTestMessage("Hex Test Performed via PC \n");
            }else{
                uint8_t data[2] = {0xFA,0x08};
                itf_sendDataMCU((char*)data);
                itf_writeTestMessage("Hex Test Performed via MCU \n");
            }
            break;
        default:
            break;
    }//End case statement
    return 1;
}

int itf_crc4AndSend(int message){
    message = itf_addCRC(message);
    uint8_t data[2] = {(uint8_t) ((message & 0xFF00)>>8),(uint8_t) (message & 0x00FF)};
    return itf_sendDataMCU((char*)data);
}

int itf_sendDataMCU(const char* data)
{
    const int len = strlen(data);
    //const int len = 2;
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI("logName", "Wrote %d bytes", len);
    return txBytes;
}

void itf_initDirPins(void){
    //Dir0, Pulldown
    //gpio_pad_select_gpio(ITF_DIR0_PIN);
	gpio_set_direction(ITF_DIR0_PIN, GPIO_MODE_INPUT);
	gpio_set_pull_mode(ITF_DIR0_PIN, GPIO_PULLDOWN_ONLY);
	gpio_set_intr_type(ITF_DIR0_PIN, GPIO_INTR_ANYEDGE);
	gpio_intr_enable(ITF_DIR0_PIN);
    //gpio_isr_register(itf_dirHandler, "Args", 0, NULL);
    itf_dirInput0 = gpio_get_level(ITF_DIR0_PIN);

    //Dir1, Pulldown
    //gpio_pad_select_gpio(ITF_DIR1_PIN);
	gpio_set_direction(ITF_DIR1_PIN, GPIO_MODE_INPUT);
	gpio_set_pull_mode(ITF_DIR1_PIN, GPIO_PULLDOWN_ONLY);
	gpio_set_intr_type(ITF_DIR1_PIN, GPIO_INTR_ANYEDGE);
	gpio_intr_enable(ITF_DIR1_PIN);
    //gpio_isr_register(dir1Handler, "Args", 0, NULL);
    itf_dirInput1 = gpio_get_level(ITF_DIR1_PIN);
    ctrl_setDirection((itf_dirInput0<<1) | itf_dirInput1);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(ITF_DIR0_PIN, itf_dirHandler, (void*) ITF_DIR0_PIN);
    gpio_isr_handler_add(ITF_DIR1_PIN, itf_dirHandler, (void*) ITF_DIR1_PIN);
}

void itf_dirHandler(void *arg){
    itf_dirInput0 = gpio_get_level(ITF_DIR0_PIN);
    itf_dirInput1 = gpio_get_level(ITF_DIR1_PIN);
    //00 = NOT RUNNING, 01 = FORWARD, 10 = BACKWARD, 11 = NOT RUNNING
    ctrl_setDirection((itf_dirInput0<<1) | itf_dirInput1);

    #ifdef COM_PRINT_DEF
        ESP_LOGI("itf_dirHandler","Dir0 = %d, Dir1 = %d",itf_dirInput0,itf_dirInput1);
    #endif
    return;
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
    //init_control_subsystem();
    
    //int lastLength = -1;

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



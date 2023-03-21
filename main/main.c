//The main

//WIP the master defines
#include "itf_master_defines.h"

//Add all needed includes
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"


#include "ctrl_subsystem.h"
#include "itf_seven_seg.h"
#include "itf_com_funcs.h"
#include "itf_sd_card_writer.h"
#include "itf_sd_card_setup.h"

//#ifndef CTRL_SUBSYSTEM_H_
//#include "ctrl_subsystem.h"
//#endif

#define SD_PRINTS_DEF 1
#define SD_WRITE_PRINTS_DEF 1
#define COM_PRINT_DEF 1
//#define TEST_SD_WRITE_DEF 1





void app_main(void)
{
    itf_initDirPins();
    itf_initHex();
    init_control_subsystem();
    
    xTaskCreate(PCComTask,"PCTask",1024*20,NULL,configMAX_PRIORITIES-1,NULL);
    xTaskCreate(MCUComTask,"MCUTask",1024*20,NULL,configMAX_PRIORITIES,NULL);
    xTaskCreatePinnedToCore(itf_writeSD_task,"SDTask",1024*50,NULL,configMAX_PRIORITIES-2,NULL,0);

    //init_control_subsystem();   //Single line to initialize and run the control subsystem. Comment out when not needed.
}

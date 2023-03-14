//The main

//WIP the master defines


//Add all needed includes
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"

//Subsystem includes
#include <ctrl_subsystem.h>


//WILL NOT RUN BY DEFAULT, TEST MAIN for TESTS!!!!!!!!!!!
void app_main(void)
{
    //This is the actaul main for the project.
    //Rename this function will testing another main, and dont delete other stuff here
    //This should have setup for board and task creation

    init_control_subsystem();   //Single line to initialize and run the control subsystem. Comment out when not needed.
}

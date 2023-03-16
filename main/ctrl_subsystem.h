#ifndef CTRL_SUBSYSTEM_H_
#define CTRL_SUBSYSTEM_H_

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "itf_seven_seg.h"

#include <time.h>




//Comment out this line when NOT testing
#define _CTRL_SYSTEM_TEST_


//******************************* GET functions
float ctrl_getSpeed_mph(void);
float ctrl_getInstPower_W(void);
float ctrl_getAvePower_W(void);
float ctrl_getTotEnergy_j(void);
float ctrl_getBatVolts_V(void);
float ctrl_getCurrent_A(void);
float ctrl_getPhaseCurA_A(void);
float ctrl_getPhaseCurB_A(void);
float ctrl_getPhaseCurC_A(void);
float ctrl_getPhaseTempA_f(void);
float ctrl_getPhaseTempB_f(void);
float ctrl_getPhaseTempC_f(void);
float ctrl_getSpeedSetting_mph(void);         //Speed may be controlled between 10.0 and 55.0 mph
float ctrl_getThrottle(void);                //Throttle may be from 0 to 4096 (0% to 100%)
bool  ctrl_isArmed(void);
bool  ctrl_isInSafetyShutdown(void);   //If safety shutdown is anything except 0, system is in safety shutdown
uint8_t ctrl_getErrorCode(void);           //The value of safety_shutdown IS the error code
uint64_t ctrl_getTime(void);

//******************************* SET functions (Return 0 on **SUCCESS**)
uint8_t ctrl_setSpeedControl(float target_mph);

uint8_t ctrl_setThrottle(uint16_t desired_throttle);

uint8_t ctrl_setDirection(uint8_t new_direction);



//******************************************************     PROTOTYPES    ******************************************************
//SETS UP AND RUNS ALL CONTROL SUSTEM CODE
void init_control_subsystem(void);

//ISRs:
static void ctrl_update_timer_cb(void *arg);    //ctrl_timer_cb() runs whenever the control timer goes off. It unblocks ctrl_operational_task() using an event 
static void IRAM_ATTR ctrl_hall_isr(void *args);    //ctrl_hall_isr() runs whenever any hall sensor pin changes state, and handles commutation without applying speed control

//TASKS:
void ctrl_operational_task(void *arg);         //ctrl_operational_task() is a task that runs 100 times per second and handles all control subsystem operations.

//FUNCTIONS:
void ctrl_alignOutputToHall(void);      //ctrl_alignOutputToHall() aligns the cur_input_index, cur_output_index, and expected_hall_state to match to the most recently read hall_state (also takes direction into account for the expected_hall_state)
void ctrl_getHallState(void);           //ctrl_getHallState() reads the hall sensor pins and updates the hall_state variable.
void ctrl_set_MSFTOutput(uint8_t output_table_index_to_use);    //ctrl_set_MSFTOutput() sets all of the MOSFET output signals to match the given index in the output_table. Also responsible for enforcing safety_shutdown as well as considering whether or not run_motor is good to go

//SETUP (ONE-TIME) FUNCTIONS:
void ctrl_setup_Output(void);
void ctrl_setup_Hall(void);     //ctrl_setup_Hall() sets up the pins and associated interrupts to start monitoring the hall effect sensor states. ALSO: gets the starting hall_state and aligns the output to it

//SPECIAL OBJECTS:
//esp_timer_handle_t ctrl_speed_control_timer;    //The timer handle used to apply speed control at a regular interval
//TaskHandle_t ctrl_operational_task_handle;      //Handle for the operational task of the control subsystem

#endif
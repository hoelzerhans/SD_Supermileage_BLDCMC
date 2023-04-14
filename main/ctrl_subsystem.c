#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "itf_seven_seg.h"
#include "itf_sd_card_writer.h"
#include <time.h>
#include "continuous_read_adc.h"





//Comment out this line when NOT testing
#define _CTRL_SYSTEM_TEST_
//Same here, initizes (somewhat unrealistic) values for everything recorded by SD, for proper testing
//Needs above initailized aswell
//#define _CTRL_ITF_SYSTEM_TEST_



//******************************************************     GENERAL     ******************************************************
static const char *TAG_CTRL = "CTRL";       //Classification tag applied to any outgoing serial communication messages from this subsystem.

//Calculation of vital parameters:
#define ctrl_HUBDIAMETER_IN (19.0)         //USERSET: The RADIUS of the wheel in inches
#define ctrl_MOTORPOLES_FL  (46.0)         //USERSET: The number of poles on the BLDC motor to be controlled
#define ctrl_MOTORPHASES_FL (3.0)          //USERSET: The number of phases on the BLDC motor to be controlled
#define ctrl_CONV_IN_MI     (1.0/63360.0)  //Conversion factor: inches to miles
#define ctrl_HUBCIRCUM_MI   (3.141592*(ctrl_HUBDIAMETER_IN*ctrl_CONV_IN_MI))   //Motor hub circumference (including wheel) in miles
#define ctrl_DIST_PER_COM   (ctrl_HUBCIRCUM_MI/(ctrl_MOTORPOLES_FL*ctrl_MOTORPHASES_FL)) //distance travelled during one commutation
#define ctrl_CONV_COM_PER_SEC_TO_MPH (ctrl_DIST_PER_COM*3600.0)


//******************************************************     PINS     ******************************************************
//These are the output pins for the motor driving logic:
#define ctrl_MSFT_AL 6
#define ctrl_MSFT_AH 7
#define ctrl_MSFT_BL 15
#define ctrl_MSFT_BH 16
#define ctrl_MSFT_CL 4
#define ctrl_MSFT_CH 5

//These are the input pins for the hall effect sensors:
#define ctrl_HA_IN 10
#define ctrl_HB_IN 9
#define ctrl_HC_IN 3




//******************************************************     PWM & SPEED CONTROL    ******************************************************
#define ctrl_SPEED_CONTROL_UPDATE_PERIOD (10000)     //How long to wait before each subsequent speed control adjustment (in microseconds)
#define ctrl_SPEED_CONTROL_UPDATE_PERIOD_MS (ctrl_SPEED_CONTROL_UPDATE_PERIOD/1000) 
#define ctrl_PWM_CHN_AH LEDC_CHANNEL_0
#define ctrl_PWM_CHN_BH LEDC_CHANNEL_1
#define ctrl_PWM_CHN_CH LEDC_CHANNEL_2
#define ctrl_PWM_FREQ   (10000)             // Frequency in Hertz.
#define ctrl_MIN_SPEED_CONTROL_MPH      10.0
#define ctrl_MAX_SPEED_CONTROL_MPH      55.0
#define ctrl_SPDCTRL_PGAIN              (100.0)      //91.02 gives full range of throttle from 10 to 55 mph




//******************************************************     ERROR CODES & THRESHOLDS     ******************************************************
//Error codes for the different reasons for safety shutdown to occur:
#define ctrl_ERROR_HALL_WIRE              0x01
#define ctrl_ERROR_HALL_CHANGE            0x02
#define ctrl_ERROR_SENSING_TIMEOUT        0x03
#define ctrl_ERROR_BAT_UNDERVOLT          0x04
#define ctrl_ERROR_BAT_OVERVOLT           0x05
#define ctrl_ERROR_PHASE_CURRENT          0x06
#define ctrl_ERROR_BAT_CURRENT            0x07
#define ctrl_ERROR_OVERHEAT               0x08
#define ctrl_ERROR_NONZERO_START_THROTTLE 0x09

//Safety thresholds (exceeding these will cause one of the error codes above)
#define ctrl_UNDERVOLTAGE_THRESHOLD_V      38.0
#define ctrl_OVERVOLTAGE_THRESHOLD_V       60.0
#define ctrl_OVERCURRENT_THRESHOLD_A       35.0
#define ctrl_TOTAL_OVERCURRENT_THRESHOLD_A ctrl_OVERCURRENT_THRESHOLD_A*2.0
#define ctrl_OVERTEMP_THRESHOLD_F          212.0
#define ctrl_SKIPPED_COMMUTATIONS_ERROR_THRESHOLD   200




//******************************************************     TASK-RELATED DEFINITIONS     ******************************************************
#define ctrl_OPERATIONAL_TASK_STACK_SIZE 4096*5




//******************************************************     ENUM< STRUCTS, TABLES    ******************************************************
//Motor driver output table, where bits 5..0 represent CH, CL, BH, BL, AH, AL (in order from highest to lowest)
const uint8_t ctrl_output_table[7] = {
    0b00000110, //6     1-(hall)
    0b00100100, //36    3-(hall)
    0b00100001, //33    2-(hall)
    0b00001001, //9     6-(hall)
    0b00011000, //24    4-(hall)
    0b00010010, //18    5-(hall)
    0b00000000  //Here as a safety measure only
};

//Valid hall effect input table, where each row in this table correlates to the same numbered row in the output_table
//Note that these items are the ZERO torque hall effect status (final outcome) of applying the motor driving logic in the output_table
//Therefore, to move forward one would need to move to output_table[row of current hall input state +/- 1] in order to actually move the motor 
const uint8_t ctrl_hall_input_table[6] = {
    0b00000001, //1
    0b00000011, //3
    0b00000010, //2
    0b00000110, //6
    0b00000100, //4
    0b00000101, //5
};

//This table contains ONLY the specific pin changes that must occur to affect the table. These changes are moving DOWN the output_table
//This means that if you currently at output_table[5], then you would make the changes from change_table[0] in order to change your
//output to match output_table[0]. Or, you could make the changes from change_table[4] instead, which would move your output to now
//match output_table[4]
const uint8_t ctrl_change_table[6][4] = {
    {ctrl_MSFT_CL, 0, ctrl_MSFT_BL, 1},
    {ctrl_MSFT_AH, 0, ctrl_MSFT_CH, 1},
    {ctrl_MSFT_BL, 0, ctrl_MSFT_AL, 1},
    {ctrl_MSFT_CH, 0, ctrl_MSFT_BH, 1},
    {ctrl_MSFT_AL, 0, ctrl_MSFT_CL, 1},
    {ctrl_MSFT_BH, 0, ctrl_MSFT_AH, 1},
};




//******************************************************     VARIABLES/GETTERS/SETTERS    ******************************************************
//Processed sensor values
double ctrl_batVolt = 0;
double ctrl_totCur = 0;
double ctrl_curA = 0;
double ctrl_curB = 0;
double ctrl_curC = 0;
double ctrl_tempA = 0;
double ctrl_tempB = 0;
double ctrl_tempC = 0;

//Motor control and speed variables
uint8_t  ctrl_direction_command = 0x00;             //00 = NOT RUNNING, 01 = FORWARD, 10 = BACKWARD, 11 = NOT RUNNING
double   ctrl_speedSetting_mph = 0.0;               //Set to true to use the speed control algorithm. False will use the current hall state only.
bool     ctrl_mc_armed = false;                     //Set to true after the motor passes startup safety checks (including throttle == 0) AND direction is 0b10 or 0b01
                                                    //Set to false after the direction changes to 0b00 or 0b11
uint16_t ctrl_speed_control_duty_raw = 2047;        //Raw speed control value. Value from 0 to 4095.
uint16_t ctrl_speed_control_duty_final = 0;         //Speed control raw value combined with any P or I feedback. 
uint16_t ctrl_throttle = 0;                         //Throttle value can be from 0 to 255. Must be 0 on startup, or motor will not be allowed to start.
uint8_t  ctrl_safety_shutdown = false;              //This can be set to true from anywhere in the program. Should only ever be set to false within the safety_shutdown task

//Hall-state and output table tracking:
uint8_t  ctrl_cur_output_index = 0;                 //Used to keep track of what output (based on the output_table) is being sent to the MOSFET drivers
uint8_t  ctrl_cur_input_index = 0;                  //Used to keep track of what index in the hall_input_table correlates to the most recent hall_state
uint8_t  ctrl_hall_state = 0x00;                    //Contains the most recently observed hall state
uint8_t  ctrl_expected_hall_state = 0x00;           //Contains the next hall state that we expect to see based on our latest MOSFET driver output
uint8_t  ctrl_skipped_commutations = 0x00;          //Tracks the number of incoherent hall state commutations that have been observed since program start.

//Calculated quantities
double ctrl_speed_mph = 0.0;         //Current ground speed in mph
double ctrl_instPower = 0.0;         //Power based on the most recent current and voltage readings
double ctrl_avePower = 0.0;          //Average power consumption in watts over the entire runtime
double ctrl_totEnergy = 0.0;         //Total energy consumption in Joules
uint32_t ctrl_runTime = 0.0;        //Total motor running time in milliseconds

//double ctrl_instPower_safe = 0.0; Hans test thing

//Important internal variables
bool ctrl_usingSpeedControl = false;
uint32_t ctrl_commutation_counter = 0;
uint64_t ctrl_commutation_timestamps[3] = {0,0,0};

//HANS TEST VAR
uint64_t intrTime_test = 0;
uint64_t intrTime_test_last = 0;


//******************************* GET functions
double ctrl_getSpeed_mph(void)           { return ctrl_speed_mph; }
double ctrl_getInstPower_W(void)         { return ctrl_instPower; }
double ctrl_getAvePower_W(void)          { return (ctrl_totEnergy / ((double)(ctrl_runTime)));  }
double ctrl_getTotEnergy_j(void)         { return ctrl_totEnergy;  }
double ctrl_getBatVolts_V(void)          { return ctrl_batVolt; }
double ctrl_getCurrent_A(void)           { return ctrl_totCur; }
double ctrl_getPhaseCurA_A(void)         { return ctrl_curA; }
double ctrl_getPhaseCurB_A(void)         { return ctrl_curB; }
double ctrl_getPhaseCurC_A(void)         { return ctrl_curC; }
double ctrl_getPhaseTempA_f(void)        { return ctrl_tempA; }
double ctrl_getPhaseTempB_f(void)        { return ctrl_tempB; }
double ctrl_getPhaseTempC_f(void)        { return ctrl_tempC; }
double ctrl_getSpeedSetting_mph(void)    { return ctrl_speedSetting_mph; }           //Speed may be controlled between 10.0 and 55.0 mph
double ctrl_getThrottle(void)            { return ctrl_throttle; }                   //Throttle may be from 0 to 4096 (0% to 100%)



bool  ctrl_isArmed(void)                { return ctrl_mc_armed; }
bool  ctrl_isInSafetyShutdown(void)     { return ((bool)ctrl_safety_shutdown); }    //If safety shutdown is anything except 0, system is in safety shutdown
uint8_t ctrl_getErrorCode(void)         { return ctrl_safety_shutdown; }            //The value of safety_shutdown IS the error code
uint64_t ctrl_getTime(void)             { return esp_timer_get_time(); }

//******************************* SET functions (Return 0 on **SUCCESS**)


uint8_t ctrl_setBatVolts(double new_voltage) {
    ctrl_batVolt = new_voltage;
    return 0;    //Success
}

uint8_t ctrl_setTempA(double new_temp) {
    ctrl_tempA = new_temp;
    return 0;    //Success
}

uint8_t ctrl_setTempB(double new_temp) {
    ctrl_tempB = new_temp;
    return 0;    //Success
}

uint8_t ctrl_setTempC(double new_temp) {
    ctrl_tempC = new_temp;
    return 0;    //Success
}

uint8_t ctrl_setCurA(double new_cur) {
    ctrl_curA = new_cur;
    return 0;    //Success
}

uint8_t ctrl_setCurB(double new_cur) {
    ctrl_curB = new_cur;
    return 0;    //Success
}

uint8_t ctrl_setCurC(double new_cur) {
    ctrl_curC = new_cur;
    return 0;    //Success
}

uint8_t ctrl_setSpeedControl(float target_mph) {
    //Reasons this CANNOT be set:
    //      (1) motor is in safety shutdown
    //      (2) motor is NOT armed (has not started)
    //      (3) speed control value is out of operational range (10-55 mph at time this was written)
    if          (ctrl_safety_shutdown > 0) { ctrl_usingSpeedControl = false; return 1; }
    else if     (!(ctrl_isArmed())) { ctrl_usingSpeedControl = false; return 2; }
    else if     ((target_mph < ctrl_MIN_SPEED_CONTROL_MPH) || (target_mph > ctrl_MAX_SPEED_CONTROL_MPH)) { ctrl_usingSpeedControl = false; return 3; }

    //If execution reaches this point, then the speed control setting may be made:
    ctrl_speedSetting_mph = target_mph;
    ctrl_usingSpeedControl = true;      //Automatically turns on speed control when execution reaches this point
    return 0;    //Success
}

uint8_t ctrl_turnOffSpeedControl(void) {
    ctrl_usingSpeedControl = false;
    return 0;    //Success
}

uint8_t ctrl_setThrottle(uint16_t desired_throttle) {
    //Reasons this CANNOT be set:
    //      (1) motor is in safety shutdown
    //      (2) throttle value is out of operational range (0-4096) (operational range matches to 12-bit ADC resolution)
    if          ((ctrl_safety_shutdown > 0) && (ctrl_safety_shutdown != ctrl_ERROR_NONZERO_START_THROTTLE)) { return 1; }   //must make an exception for the NONZERO STARTING THROTTLE SETTING
    else if     (desired_throttle > 4096) { return 2; }

    //If execution reaches this point, then the new throttle can take effect:
    ctrl_throttle = desired_throttle;
    return 0;    //Success
}

uint8_t ctrl_setDirection(uint8_t new_direction) {
    //For the moment, this is allowed to be set under all circumstances (the motor arming sequence should handle
    //      any undesirable situations). However, placing this into a "settter" function in case we find future
    //      cases where logic needs to be applied to avoid issues.
    ctrl_direction_command = new_direction;
    return 0;    //Success
}




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
uint8_t ctrl_getHallState(void);           //ctrl_getHallState() reads the hall sensor pins and updates the hall_state variable.
void ctrl_set_MSFTOutput(uint8_t output_table_index_to_use);    //ctrl_set_MSFTOutput() sets all of the MOSFET output signals to match the given index in the output_table. Also responsible for enforcing safety_shutdown as well as considering whether or not run_motor is good to go

//SETUP (ONE-TIME) FUNCTIONS:
void ctrl_setup_Output(void);
void ctrl_setup_Hall(void);     //ctrl_setup_Hall() sets up the pins and associated interrupts to start monitoring the hall effect sensor states. ALSO: gets the starting hall_state and aligns the output to it

//SPECIAL OBJECTS:
esp_timer_handle_t ctrl_speed_control_timer;    //The timer handle used to apply speed control at a regular interval
TaskHandle_t ctrl_operational_task_handle;      //Handle for the operational task of the control subsystem



//******************************************************     PROTOTYPE DEFINITIONS    ******************************************************
void init_control_subsystem(void) {
    //Setup variables for testing if _CTRL_SYSTEM_TEST_ is defined
    #ifdef _CTRL_SYSTEM_TEST_
        ctrl_batVolt    = ctrl_OVERVOLTAGE_THRESHOLD_V-1.0;
        ctrl_curA       = ctrl_OVERCURRENT_THRESHOLD_A-1.0;
        ctrl_curB       = 0;
        ctrl_curC       = 0;
        ctrl_tempA      = ctrl_OVERTEMP_THRESHOLD_F-1.0;
        ctrl_tempB      = ctrl_OVERTEMP_THRESHOLD_F-1.0;
        ctrl_tempC      = ctrl_OVERTEMP_THRESHOLD_F-1.0;
    
        ctrl_direction_command = 0x01;              //00 = NOT RUNNING, 01 = FORWARD, 10 = BACKWARD, 11 = NOT RUNNING
        ctrl_mc_armed = true;                      //Set to true after the motor passes startup safety checks (including throttle == 0) AND direction is 0b10 or 0b01
                                                    //Set to false after the direction changes to 0b00 or 0b11
        ctrl_speed_control_duty_raw = 2047;            //2047 = 50%
        ctrl_speedSetting_mph = 22.0;                  //Set to non-zero (and between the min and max thresholds (10-55 mph at time of writing)) to activate speed control.
        ctrl_usingSpeedControl = true;                   
        ctrl_throttle = 1027;                       //Throttle value can be from 0 to 4096. Must be 0 on startup, or motor will not be allowed to start.     
        
        #ifdef _CTRL_ITF_SYSTEM_TEST_
            ctrl_curB       = -10.2;
            ctrl_curC       = 23.6;
            ctrl_tempB = 40.5;
            ctrl_tempC = 63.2;
        #endif
    
    #endif

    //Run initial setup functions for control subsystem:
    ctrl_setup_Output();    //Prepare the gate driver control output pins
    ctrl_setup_Hall();      //Prepare the hall sensor input pins and interrupts

    //Create a periodic timer that will handle the speed control algorithm
    const esp_timer_create_args_t ctrl_speed_control_timer_args = {
            .callback = &ctrl_update_timer_cb,
            .name = "speed_control_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&ctrl_speed_control_timer_args, &ctrl_speed_control_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(ctrl_speed_control_timer, ctrl_SPEED_CONTROL_UPDATE_PERIOD)); //period in us

    //Start the control operational task, which will run 200 times per second
    xTaskCreate(ctrl_operational_task, "ctrl_operational_task", ctrl_OPERATIONAL_TASK_STACK_SIZE, NULL, 8, &ctrl_operational_task_handle);
}




//******************************************************     FUNCTIONS     ******************************************************
//ctrl_setup_Output() sets up the output pins for MOSFET driver output, as well as for the error LED
void ctrl_setup_Output(void) {
    //Prepare LOWSIDE output pins for the motor driving signals
    gpio_set_direction(ctrl_MSFT_AL, GPIO_MODE_OUTPUT);
    gpio_set_direction(ctrl_MSFT_BL, GPIO_MODE_OUTPUT);
    gpio_set_direction(ctrl_MSFT_CL, GPIO_MODE_OUTPUT);
    gpio_set_level(ctrl_MSFT_AL, 0);
    gpio_set_level(ctrl_MSFT_BL, 0);
    gpio_set_level(ctrl_MSFT_CL, 0);

    //Setup HIGHSIDE pins for PWM:
        // Prepare and then apply the LEDC PWM timer configuration
        ledc_timer_config_t ledc_timer = {
            .speed_mode       = LEDC_LOW_SPEED_MODE,
            .timer_num        = LEDC_TIMER_0,
            .duty_resolution  = LEDC_TIMER_12_BIT,
            .freq_hz          = ctrl_PWM_FREQ,  // Set output frequency to 10 kHz
            .clk_cfg          = LEDC_AUTO_CLK
        };
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

        ledc_channel_config_t MSFT_PWM_CONFIG = {
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = ctrl_PWM_CHN_AH,
            .timer_sel      = LEDC_TIMER_0,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = (ctrl_MSFT_AH),
            .duty           = 0, // Set initial duty to 0%
            .hpoint         = 0,
            .flags.output_invert = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&MSFT_PWM_CONFIG));
        MSFT_PWM_CONFIG.channel = (ctrl_PWM_CHN_BH);
        MSFT_PWM_CONFIG.gpio_num = (ctrl_MSFT_BH);
        ESP_ERROR_CHECK(ledc_channel_config(&MSFT_PWM_CONFIG));
        MSFT_PWM_CONFIG.channel = (ctrl_PWM_CHN_CH);
        MSFT_PWM_CONFIG.gpio_num = (ctrl_MSFT_CH);
        ESP_ERROR_CHECK(ledc_channel_config(&MSFT_PWM_CONFIG));
}


//ctrl_setup_Hall() sets up the pins and associated interrupts to start monitoring the hall effect sensor states. ALSO: gets the starting hall_state and aligns the output to it
void ctrl_setup_Hall(void) {
    //Prepare hall pins for interrupt inputs
    gpio_set_direction(ctrl_HA_IN, GPIO_MODE_INPUT);
    gpio_pulldown_dis(ctrl_HA_IN);
    gpio_pullup_en(ctrl_HA_IN);
    gpio_set_intr_type(ctrl_HA_IN, GPIO_INTR_ANYEDGE);
    gpio_set_direction(ctrl_HB_IN, GPIO_MODE_INPUT);
    gpio_pulldown_dis(ctrl_HB_IN);
    gpio_pullup_en(ctrl_HB_IN);
    gpio_set_intr_type(ctrl_HB_IN, GPIO_INTR_ANYEDGE);
    gpio_set_direction(ctrl_HC_IN, GPIO_MODE_INPUT);
    gpio_pulldown_dis(ctrl_HC_IN);
    gpio_pullup_en(ctrl_HC_IN);
    gpio_set_intr_type(ctrl_HC_IN, GPIO_INTR_ANYEDGE);

    //Get current hall state and align the output tables
    ctrl_getHallState();
    ctrl_alignOutputToHall();

    //Install the ISR services for each of the hall input pins
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ctrl_HA_IN, ctrl_hall_isr, (void *)ctrl_HA_IN);
    gpio_isr_handler_add(ctrl_HB_IN, ctrl_hall_isr, (void *)ctrl_HB_IN);
    gpio_isr_handler_add(ctrl_HC_IN, ctrl_hall_isr, (void *)ctrl_HC_IN);
}


//ctrl_getHallState() reads the hall sensor pins and updates the hall_state variables
uint8_t ctrl_getHallState(void) { ctrl_hall_state = (gpio_get_level(ctrl_HC_IN) << 2) | (gpio_get_level(ctrl_HB_IN)<<1) | gpio_get_level(ctrl_HA_IN);
    return ctrl_hall_state;
 }


void ctrl_set_MSFTOutput(uint8_t output_table_index_to_use) { 
    if ((ctrl_mc_armed) && (ctrl_safety_shutdown == 0)) {
        //Determine which duty cycle to apply (throttle or speed control).
        uint16_t duty = 0;
        if (ctrl_usingSpeedControl) { duty = ctrl_speed_control_duty_final; } else { duty = ctrl_throttle; }
        
        //Set LOWSIDE outputs
        gpio_set_level(ctrl_MSFT_AL, ((ctrl_output_table[output_table_index_to_use] & 0b00000001) >> 0));
        gpio_set_level(ctrl_MSFT_BL, ((ctrl_output_table[output_table_index_to_use] & 0b00000100) >> 2));
        gpio_set_level(ctrl_MSFT_CL, ((ctrl_output_table[output_table_index_to_use] & 0b00010000) >> 4));

        //Set AH output
        if((ctrl_output_table[output_table_index_to_use] & 0b00000010) > 0) { 
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, ctrl_PWM_CHN_AH, duty));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, ctrl_PWM_CHN_AH));
        } else { 
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, ctrl_PWM_CHN_AH, 0));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, ctrl_PWM_CHN_AH));
        }

        //Set BH output
        if((ctrl_output_table[output_table_index_to_use] & 0b00001000) > 0) { 
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, ctrl_PWM_CHN_BH, duty));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, ctrl_PWM_CHN_BH));
        } else { 
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, ctrl_PWM_CHN_BH, 0));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, ctrl_PWM_CHN_BH));
        }

        //Set CH output
        if((ctrl_output_table[output_table_index_to_use] & 0b00100000) > 0) { 
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, ctrl_PWM_CHN_CH, duty));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, ctrl_PWM_CHN_CH));
        } else { 
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, ctrl_PWM_CHN_CH, 0));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, ctrl_PWM_CHN_CH));
        }

    } else {
        //Set all low.
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, ctrl_PWM_CHN_AH, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, ctrl_PWM_CHN_AH));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, ctrl_PWM_CHN_BH, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, ctrl_PWM_CHN_BH));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, ctrl_PWM_CHN_CH, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, ctrl_PWM_CHN_CH));
        gpio_set_level(ctrl_MSFT_AL, 0);
        gpio_set_level(ctrl_MSFT_BL, 0);
        gpio_set_level(ctrl_MSFT_CL, 0);
    }
}


/* ctrl_alignOutputToHall() aligns the cur_input_index, cur_output_index, and expected_hall_state to 
 * match to the most recently read hall_state (also takes direction into account for the expected_hall_state)
*/
void ctrl_alignOutputToHall(void) {
    //Re-align the output to the output table based on the current hall state and desired movement direction
    //Align the cur_table_index to match the starting hall state.
    uint8_t i = 0;
    for(i = 0; i < 6; i++) { if(ctrl_hall_input_table[i] == ctrl_hall_state) { break; } }
    if (i < 6) {    //if i is 6 or more, there was no matching state
        ctrl_cur_input_index = i;
        //Adjust the commutation table index based on which way we want to turn
        //ALSO: Handle wrap around of the cur_table_index
        if (ctrl_direction_command == 0x01) {
            ctrl_cur_output_index = ctrl_cur_input_index + 1;
            if (ctrl_cur_output_index > 5) { ctrl_cur_output_index = 0; }
        } else if (ctrl_direction_command == 0x02) {
            ctrl_cur_output_index = ctrl_cur_input_index - 1;
            if (ctrl_cur_output_index > 5) { ctrl_cur_output_index = 5; } //Note that since cur_table_index is a uint, if it goes "below 0" it becomes 255
        }

        //Determine what the next hall state should look like
        ctrl_expected_hall_state = ctrl_hall_input_table[ctrl_cur_output_index];

        //************************************THIS MAY BE THE ONE PLACE WHERE A SHORT COULD HAPPEN!*****************************************
        //AND that short can only happen if a commutation state was missed
        //Fix ALL outputs
        ctrl_set_MSFTOutput(ctrl_cur_output_index);
    }
}







//******************************************************     TASKS     ******************************************************
//ctrl_operational_task() is a task that runs 100 times per second and handles all control subsystem operations.
/*
    SPECIFICALLY:
        Before startup (Startup Section):
            Wait for throttle to be zero
            Ensure hall-states are valid / wiring is OK
            Ensure battery is not OVERVOLTAGE or UNDERVOLTAGE
            Wait for direction command to be NON-NEUTRAL
            ONCE ALL CRITERIA ARE MET: Arm the motor and begin commutation
        After startup (Operational Section):


*/
void ctrl_operational_task(void *arg) {
    static uint32_t update_timer_alarmed;
    while(1)
    {
        if(intrTime_test != intrTime_test_last){
            //ESP_LOGI("Testmsg","time: %d",(int)intrTime_test);
            intrTime_test_last = intrTime_test;
        }
        //Wake when notified that the update timer has alarmed
        update_timer_alarmed = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if(update_timer_alarmed) {
            //STARTUP SECTION
            if (!ctrl_mc_armed) {
                //Check for nonzero starting throttle
                if (!(ctrl_throttle == 0)) {
                    if (ctrl_safety_shutdown != ctrl_ERROR_NONZERO_START_THROTTLE) { ESP_LOGE(TAG_CTRL, "ERROR: NONZERO STARTING THROTTLE (%d)", ctrl_throttle); }
                    ctrl_safety_shutdown = ctrl_ERROR_NONZERO_START_THROTTLE;

                //Check for hall sensor wiring issues
                } else if (((ctrl_hall_state == 7) || (ctrl_hall_state == 0))) {
                    if (ctrl_safety_shutdown != ctrl_ERROR_HALL_WIRE) { ESP_LOGE(TAG_CTRL, "ERROR: HALL WIRING ISSUE (digital '000' or '111')(%d)", ctrl_hall_state); }
                    ctrl_safety_shutdown = ctrl_ERROR_HALL_WIRE;

                //Ensure the battery is neither overvoltage nor undervoltage
                } else if (ctrl_batVolt < ctrl_UNDERVOLTAGE_THRESHOLD_V) {
                    ctrl_safety_shutdown = ctrl_ERROR_BAT_UNDERVOLT;
                    ESP_LOGE(TAG_CTRL, "ERROR: BATTERY UNDERVOLTAGE (%f)", ctrl_batVolt);
                } else if (ctrl_batVolt > ctrl_OVERVOLTAGE_THRESHOLD_V) {
                    ctrl_safety_shutdown = ctrl_ERROR_BAT_OVERVOLT;
                    ESP_LOGE(TAG_CTRL, "ERROR: BATTERY OVERVOLTAGE (%f)", ctrl_batVolt);

                //If the above tests have passed, then there is no safety issue indicated at this time
                } else {
                    ctrl_safety_shutdown = 0;                    
                    //If the direction command is in forward or backward state, allow motor to run.
                    if((ctrl_direction_command == 0b01) || (ctrl_direction_command == 0b10)) {
                        ctrl_mc_armed = true;
                        ctrl_alignOutputToHall();   //must align output to hall
                        ESP_LOGI(TAG_CTRL, "*****MOTOR ARMED*****");
                    } else {
                        //If this point of execution is reached, the direction bits indicate the motor should not be armed.
                        ctrl_mc_armed = false;
                        //Also: push 0 output to all MOSFET outputs using the set_MSFTOutput() function IMMEDIATELY
                        ctrl_set_MSFTOutput(6);
                    }
                }


            //OPERATIONAL SECTION:
            } else { 
                //Ensure hall sensor wiring is valid
                if ((ctrl_hall_state == 7) || (ctrl_hall_state == 0)) {
                    ctrl_safety_shutdown = ctrl_ERROR_HALL_WIRE;
                    ESP_LOGE(TAG_CTRL, "ERROR: HALL WIRING (%d)", ctrl_hall_state);
                    

                //Ensure that there have not been too many commutation errors (likely indicates wiring issue)
                } else if (ctrl_skipped_commutations > ctrl_SKIPPED_COMMUTATIONS_ERROR_THRESHOLD) {
                    ctrl_safety_shutdown = ctrl_ERROR_HALL_CHANGE;
                    ESP_LOGE(TAG_CTRL, "ERROR: TOO MANY HALL SEQUENCE FAILURES (%d). CHECK WIRING AND RESTART", ctrl_skipped_commutations);
                    

                //Ensure the battery is neither overvoltage nor undervoltage
                } else if (ctrl_batVolt < ctrl_UNDERVOLTAGE_THRESHOLD_V) {
                    ctrl_safety_shutdown = ctrl_ERROR_BAT_UNDERVOLT;
                    ESP_LOGE(TAG_CTRL, "ERROR: BATTERY UNDERVOLTAGE (%f)", ctrl_batVolt);
                    
                } else if (ctrl_batVolt > ctrl_OVERVOLTAGE_THRESHOLD_V) {
                    ctrl_safety_shutdown = ctrl_ERROR_BAT_OVERVOLT;
                    ESP_LOGE(TAG_CTRL, "ERROR: BATTERY OVERVOLTAGE (%f)", ctrl_batVolt);
                    

                //Ensure total current and phase currents have not exceeded safety thresholds
                } else if ((ctrl_curA + ctrl_curB + ctrl_curC) > ctrl_TOTAL_OVERCURRENT_THRESHOLD_A) {
                    ctrl_safety_shutdown = ctrl_ERROR_BAT_CURRENT;
                    ESP_LOGE(TAG_CTRL, "ERROR: BATTERY OVERCURRENT (%f). PHASE CURRENTS WERE: %f\t%f\t%f)", (ctrl_curA + ctrl_curB + ctrl_curC), ctrl_curA, ctrl_curB, ctrl_curC);
                    

                } else if ((ctrl_curA > ctrl_OVERCURRENT_THRESHOLD_A) || (ctrl_curB > ctrl_OVERCURRENT_THRESHOLD_A) || (ctrl_curC > ctrl_OVERCURRENT_THRESHOLD_A)) {
                    ctrl_safety_shutdown = ctrl_ERROR_PHASE_CURRENT;
                    ESP_LOGE(TAG_CTRL, "ERROR: PHASE OVERCURRENT (%f\t%f\t%f)", ctrl_curA, ctrl_curB, ctrl_curC);
                    
                
                //Ensure heat sink temperature has not exceeded safety threshold
                } else if ((ctrl_tempA > ctrl_OVERTEMP_THRESHOLD_F) || (ctrl_tempB > ctrl_OVERTEMP_THRESHOLD_F) || (ctrl_tempC > ctrl_OVERTEMP_THRESHOLD_F)) {
                    ctrl_safety_shutdown = ctrl_ERROR_OVERHEAT;
                    ESP_LOGE(TAG_CTRL, "ERROR: MOSFET OVERHEAT (%f\t%f\t%f)", ctrl_tempA, ctrl_tempB, ctrl_tempC);                    
                }

                //Now handle any error that occured
                if (ctrl_safety_shutdown) {
                    //Push 0 output to all MOSFET outputs using the set_MSFTOutput() function IMMEDIATELY
                    ctrl_set_MSFTOutput(6);
                    ESP_LOGI(TAG_CTRL, "*****MOTOR DISARMED (SAFETY)*****");

                //If the direction bits indicate the motor should disengage, do so now
                } else if ((ctrl_direction_command == 0b00) || (ctrl_direction_command == 0b11)) {
                    //Also: push 0 output to all MOSFET outputs using the set_MSFTOutput() function IMMEDIATELY
                    ctrl_set_MSFTOutput(6);
                    ctrl_mc_armed = false;
                    ctrl_alignOutputToHall();   //must align output to hall
                    ESP_LOGI(TAG_CTRL, "*****MOTOR DISARMED (NORMAL)*****");

                } else {
                    
                    /* 
                        If the program reaches this point in execution, we know:
                            (1) the motor is armed/running and
                            (2) there are no safety issues present
                        Therefore it is time to perform updates:
                            Update the motor runTime
                            Calculate the instantaneous power usage of the motor
                            Calculate the total energy consumption of the motor
                            Checks how many commutations have occured since the last time this block executed (should be approx 5ms prior)
                                if no commutations were made in that time, look at the time between three most recent commutations
                                Calculates the current ground speed based on the number of commutations or three commutation times (as applicable)
                            When speed control setting is valid:
                                Increases the speed control duty cycle if moving too slow
                                Reduces the speed control duty cycle if moving too fast
                    */
                    ctrl_runTime += ctrl_SPEED_CONTROL_UPDATE_PERIOD_MS;  //Add one timer period to the run time.
                    float update_period_float = ((float)ctrl_SPEED_CONTROL_UPDATE_PERIOD_MS)/1000.0;
                    ctrl_instPower = ((ctrl_curA+ctrl_curB+ctrl_curC) * 0.5) * ctrl_batVolt;    //The current at any time should be 1/2 of the sum of all current sensor readings (because both the high phase and low phase share the same current)
                    
                    ctrl_totEnergy += update_period_float*ctrl_instPower; //Update the total energy consumption
                    //Calculate ground speed (from commutation quantity if there is enough)
                    if (ctrl_commutation_counter >= 4) {
                        ctrl_speed_mph = (((float)ctrl_commutation_counter)/update_period_float)*ctrl_CONV_COM_PER_SEC_TO_MPH;
                    //If there have not been enough commutations in the last update period, check if there have been at least three recorded commutation times
                    //  recently enough that speed can be determined (with 10ms update period, this generally happens when under 8mph)
                    } else if ((ctrl_commutation_timestamps[0] > 0) && (ctrl_commutation_timestamps[1] > 0) && (ctrl_commutation_timestamps[2] > 0)) {
                        float ave_time = (((float)(ctrl_commutation_timestamps[0]-ctrl_commutation_timestamps[1])) + ((float)(ctrl_commutation_timestamps[1]-ctrl_commutation_timestamps[2])))/2.0;
                        ctrl_speed_mph = ((ctrl_CONV_COM_PER_SEC_TO_MPH/ave_time)*1000000.0); //1000000.0 to convert us to s
                        ctrl_commutation_timestamps[2] = 0; //Setting this equal to 0 ensures that this branch of the if...else will not execute again until another commutation occurs and is stored in the array
                    } else {
                        //There haven't been enough commutations to determine speed recently.
                        ctrl_speed_mph = 0;
                    }
                    ctrl_commutation_counter = 0;   //reset commutation counter

                    //Update speed control
                    if (ctrl_usingSpeedControl) {
                        double error = ctrl_speedSetting_mph - ctrl_speed_mph;
                        ctrl_speed_control_duty_final = ctrl_speed_control_duty_raw + ((uint16_t)(error*ctrl_SPDCTRL_PGAIN));
                        //Perform min/max control
                        if (ctrl_speed_control_duty_final > 16000)      { ctrl_speed_control_duty_final = 0; }    //Better way to handle uint overflow from substraction
                        if (ctrl_speed_control_duty_final > 4095)       { ctrl_speed_control_duty_final = 4095; }
                    }
                }
            }
            itf_displayHex(ctrl_safety_shutdown);   //lastly, update the hex display
        } //END if(update_timer_alarmed)
    }
}




//******************************************************     ISRs     ******************************************************
//ctrl_hall_isr() runs whenever any hall sensor pin changes state, and handles commutation without applying speed control
static void ctrl_hall_isr(void *args)
{
    //Find which pin triggered the ISR (may be useful in future)
    //int pinNumber = (int)args;

    //Increment the commutation counter (used for speed control)
    //uint64_t startTime = esp_timer_get_time();
    ctrl_commutation_counter++;

    //Record and retain the time information for the three most recent commutations.
    ctrl_commutation_timestamps[2] = ctrl_commutation_timestamps[1];
    ctrl_commutation_timestamps[1] = ctrl_commutation_timestamps[0];
    ctrl_commutation_timestamps[0] = esp_timer_get_time();

    //Update hall state
    ctrl_getHallState();

    //Check for skipped commutations
    if(ctrl_hall_state != ctrl_expected_hall_state) { ctrl_skipped_commutations++; }

    //Update output to match hall input
    ctrl_alignOutputToHall();


    //HANS TEST STUFF!!
    
    itf_addShortData();
    if(ITF_LONGDATA_FLAG){
        itf_addLongData();
    }
    //uint8_t dataToStore[23];


   // int iPower1 = ctrl_instPower_safe;
    //int iPower = iPower1;
    //int iPower = ctrl_getInstPower_W();
    //dataToStore[5] = (char) ((iPower & 0xFF00)>>8);
    //dataToStore[6] = iPower;
    
    //uint64_t endTime = esp_timer_get_time();
    //if(ITF_LONGDATA_FLAG){
    //    intrTime_test = endTime - startTime;
    //}
    ITF_LONGDATA_FLAG = 0;
}

//ctrl_operational_timer_cb() is used to unblock the control subsystem's operational tasks with precise timing
static void ctrl_update_timer_cb(void *arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(ctrl_operational_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
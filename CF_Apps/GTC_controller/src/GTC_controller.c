
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"
#include "app_channel.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "GTC_CONTROLLER"
#include "debug.h"
#include "led.h"

#include "log.h"
#include "param.h"

#include "controller.h"
#include "GTC_controller.h"

float Theta_x_cam_est = 0.0f;
float Theta_y_cam_est = 0.0f;
float Theta_z_cam_est = 0.0f;

struct GTC_CmdPacket{
    uint8_t cmd_type; 
    float cmd_val1;
    float cmd_val2;
    float cmd_val3;
    float cmd_flag;
} __attribute__((packed));

struct GTC_CmdPacket GTC_Cmd;

void appMain() {
    DEBUG_PRINT("Waiting for activation ...\n");


    

    while(1) {

        if (appchannelReceiveDataPacket(&GTC_Cmd,sizeof(GTC_Cmd),APPCHANNEL_WAIT_FOREVER))
        {

            // consolePrintf("App channel received x: %.3f\n",(double)GTC_Cmd.cmd_val3);

        }
    }
  
}


void controllerOutOfTreeInit() {
    // TURN OFF BLUE LEDS (THEY'RE VERY BRIGHT)
    ledSet(LED_BLUE_L, 0);
    ledSet(LED_BLUE_NRF, 0);

}

bool controllerOutOfTreeTest() {

  return true;
}

void controllerOutOfTree(control_t *control,const setpoint_t *setpoint, 
                                            const sensorData_t *sensors, 
                                            const state_t *state, 
                                            const uint32_t tick) 
{

    if (RATE_DO_EXECUTE(2, tick))
    {
        // DEBUG_PRINT("Hello Custom Controller!\n");
        // DEBUG_PRINT("Param Value: %.3f\n",(double)value_1);
        // DEBUG_PRINT("Log Value: %.3f\n",(double)value_2);



    }
  

}

PARAM_GROUP_START(my_PARAM)
    PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, value_1, &value_1)
PARAM_GROUP_STOP(my_PARAM)

LOG_GROUP_START(my_LOG)
    LOG_ADD(LOG_FLOAT, value_2, &value_2)
LOG_GROUP_STOP(my_LOG)

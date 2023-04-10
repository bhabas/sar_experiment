#include "GTC_controller.h"


struct GTC_CmdPacket{
    uint8_t cmd_type; 
    float cmd_val1;
    float cmd_val2;
    float cmd_val3;
    float cmd_flag;
    bool  cmd_rx;
} __attribute__((packed));

extern struct GTC_CmdPacket GTC_Cmd;

void appMain() {

    // while(1) {

    //     // #ifdef GAZEBO_SIM
    //     // // EXECUTE GTC COMMAND WHEN RECEIVED
    //     // if (GTC_Cmd.cmd_rx == true)
    //     // {
    //     //     // GTC_Command(&GTC_Cmd);
    //     // }
    //     // #else
    //     // // WAIT UNTIL GTC COMMAND IS RECEIVED
    //     // if (appchannelReceiveDataPacket(&GTC_Cmd,sizeof(GTC_Cmd),APPCHANNEL_WAIT_FOREVER))
    //     // {
    //     //     // if (GTC_Cmd.cmd_rx == true) GTC_Command(&GTC_Cmd);
    //     // }
    //     // #endif
    // }
  
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
        DEBUG_PRINT("Hello Custom Controller!\n");


    }
  

}

#ifndef GAZEBO_SIM
// PARAM_GROUP_START(my_PARAM)
//     PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, value_1, &value_1)
// PARAM_GROUP_STOP(my_PARAM)

// LOG_GROUP_START(my_LOG)
//     LOG_ADD(LOG_FLOAT, value_2, &value_2)
// LOG_GROUP_STOP(my_LOG)
#endif

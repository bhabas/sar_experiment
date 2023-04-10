#include "controller_GTC.h"



void appMain() {

    while(1) {

        #ifdef GAZEBO_SIM
        // EXECUTE GTC COMMAND WHEN RECEIVED
        if (GTC_Cmd.cmd_rx == true)
        {
            // GTC_Command(&GTC_Cmd);
        }
        #else
        // WAIT UNTIL GTC COMMAND IS RECEIVED
        if (appchannelReceiveDataPacket(&GTC_Cmd,sizeof(GTC_Cmd),APPCHANNEL_WAIT_FOREVER))
        {
            // if (GTC_Cmd.cmd_rx == true) GTC_Command(&GTC_Cmd);
            consolePrintf("GTC RX value1: %.3f\n",(double)GTC_Cmd.cmd_val1);
        }
        #endif
    }
    
}

    


void controllerOutOfTreeInit() {
    consolePrintf("GTC init\n");

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
        consolePrintf("GTC loop value1: %.3f\n",(double)GTC_Cmd.cmd_val1);
    }

    // UPDATE OPTICAL FLOW VALUES AT 100 HZ
    if (RATE_DO_EXECUTE(RATE_100_HZ, tick)) {

    }
    

    if (RATE_DO_EXECUTE(2, tick)) {

        controlOutput(state,sensors);

    }
 

}

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "GTC_CONTROLLER"
#include "debug.h"
#include "led.h"

#include "log.h"
#include "param.h"

#include "controller.h"
#include "GTC_controller.h"
// #include "aideck_comm_task.h"

#include "uart1.h"

#define ARRAY_LENGTH 3
static uint8_t data_Array[sizeof(float) * ARRAY_LENGTH]; // Byte-String data array
float value_Array[ARRAY_LENGTH]; // Value array to store the received float values

float Theta_x_cam_est = 0.0f;
float Theta_y_cam_est = 0.0f;
float Theta_z_cam_est = 0.0f;


void appMain() {
    DEBUG_PRINT("Waiting for activation ...\n");

    // TURN OFF BLUE LEDS (THEY'RE VERY BRIGHT)
    ledSet(LED_BLUE_L, 0);
    ledSet(LED_BLUE_NRF, 0);

    // INITIALIZE UART1 CONNECTION
    uart1Init(UART1_BAUDRATE); 


    while(1) {

        // RECIEVE BYTE-STRING OVER UART1
        uart1GetBytesWithDefaultTimeout(sizeof(data_Array),data_Array);

        // CONVERT BYTE-STRING TO FLOAT ARRAY
        for (int i = 0; i < ARRAY_LENGTH; i++) {
        memcpy(&value_Array[i], &data_Array[i * sizeof(float)], sizeof(float));
        }

        Theta_x_cam_est = value_Array[0];
        Theta_y_cam_est = value_Array[1];
        Theta_z_cam_est = value_Array[2];
    }
  
}


void controllerOutOfTreeInit() {

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
        // DEBUG_PRINT("Param Value: %.3f\n",(double)value_1);
        // DEBUG_PRINT("Log Value: %.3f\n",(double)value_2);

        DEBUG_PRINT("Value1: %f \t Value2: %f \t Value3: %f\n\n",(double)setpoint->cmd_val1,(double)setpoint->cmd_val2,(double)setpoint->cmd_val3);

    }
  

}

PARAM_GROUP_START(my_PARAM)
    PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, value_1, &value_1)
PARAM_GROUP_STOP(my_PARAM)

LOG_GROUP_START(my_LOG)
    LOG_ADD(LOG_FLOAT, value_2, &value_2)
LOG_GROUP_STOP(my_LOG)

/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * hello_world.c - App layer application of a simple hello world debug print every
 *   2 seconds.
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "MYCONTROLLER"
#include "debug.h"
#include "led.h"

#include "log.h"
#include "param.h"


void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  // TURN OFF BLUE LEDS (THEY'RE VERY BRIGHT)
  ledSet(LED_BLUE_L, 0);
  ledSet(LED_BLUE_NRF, 0);

  while(1) {
    vTaskDelay(M2T(2000));
    // DEBUG_PRINT("Hello World!\n");
  }
}

// The new controller goes here --------------------------------------------
// Move the includes to the the top of the file if you want to
#include "controller.h"

// Call the PID controller in this example to make it possible to fly. When you implement you own controller, there is
// no need to include the pid controller.
#include "controller_pid.h"

float value_1 = 5.4f;
float value_2 = 3.14f;


void controllerOutOfTreeInit() {
  // Initialize your controller data here...

  // Call the PID controller instead in this example to make it possible to fly
  controllerPidInit();
}

bool controllerOutOfTreeTest() {
  // Always return true
  return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  // Implement your controller here...

  // Call the PID controller instead in this example to make it possible to fly
  // controllerPid(control, setpoint, sensors, state, tick);
  if (RATE_DO_EXECUTE(2, tick))
  {
    DEBUG_PRINT("Hello Custom Controller!\n");
    DEBUG_PRINT("Param Value: %.3f\n",(double)value_1);
    DEBUG_PRINT("Log Value: %.3f\n",(double)value_2);


    }
  

}

PARAM_GROUP_START(my_PARAM)
  PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, value_1, &value_1)
PARAM_GROUP_STOP(my_PARAM)

LOG_GROUP_START(my_LOG)
LOG_ADD(LOG_FLOAT, value_2, &value_2)
LOG_GROUP_STOP(my_LOG)

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
#include "uart1.h"
#include "log.h"


#define DEBUG_MODULE "HELLOWORLD"
#include "debug.h"
#include "log.h"
#define ARRAY_LENGTH 3


uint8_t myArray[3] = {0,0,0}; // array to store the 3 float values
uint8_t tempArray[sizeof(uint8_t) * 3]; // temporary array of char to store the received data

uint8_t val = 0;

void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");
  uart1Init(UART1_BAUDRATE);


  while(1) {
    vTaskDelay(M2T(500));

    uart1GetDataWithDefaultTimeout(&val);
    // uart1GetBytesWithDefaultTimeout(sizeof(tempArray),tempArray);

    // for (int i = 0; i < 3; i++) {
    //   memcpy(&myArray[i], &tempArray[i * sizeof(uint8_t)], sizeof(uint8_t));
    // }

    consolePrintf("Value: %d\n",val);

    // consolePrintf("Value1: %f \t Value2: %f \t Value3: %f\n",(double)myArray[0],(double)myArray[1],(double)myArray[2]);
  }
}


// LOG_GROUP_START(my_LOG)
// LOG_ADD(LOG_UINT8, my_char, &c)
// LOG_GROUP_STOP(my_LOG)
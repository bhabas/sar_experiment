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


char my_char = 'C';

void appMain() {
    DEBUG_PRINT("Waiting for activation ...\n");
    uart1Init(115200);
    consolePrintf("Value: %c\n",my_char);

    char char_arr[2] = {'A','B'};

    while(1) {
        vTaskDelay(M2T(500));

        // uart1GetDataWithDefaultTimeout(&my_char);
        // uart1GetDataWithTimeout(&my_char,portMAX_DELAY);
        uart1GetBytesWithDefaultTimeout(2,char_arr);
        // uart1Getchar(&my_char);

        consolePrintf("Value: %c \t Value: %c\n",char_arr[0],char_arr[1]);
  }
}


LOG_GROUP_START(my_LOG)
LOG_ADD(LOG_UINT8, my_char, &my_char)
LOG_GROUP_STOP(my_LOG)
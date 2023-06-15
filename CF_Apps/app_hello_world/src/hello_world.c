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

#define DEBUG_MODULE "HELLOWORLD"
#include "debug.h"
#include <stdlib.h>


void appMain() {
    DEBUG_PRINT("Waiting for activation ...\n");

    // Allocate memory for an array of 10 integers.
    int *arr = (int *)malloc(10 * sizeof(int));
    if (arr == NULL) {
        DEBUG_PRINT("Memory allocation failed.\n");
    }


    // Initialize the array with values.
    for (int i = 0; i < 10; i++) {
        arr[i] = i;
    }

    // Print the array elements.
    DEBUG_PRINT("Array elements are:\n");
    for (int i = 0; i < 10; i++) {
        DEBUG_PRINT("%d ", arr[i]);
    }

    DEBUG_PRINT("\n");

    // // Free the memory.
    free(arr);

    while(1) {
        vTaskDelay(M2T(2000));
        DEBUG_PRINT("Hello World!\n");
    }
}

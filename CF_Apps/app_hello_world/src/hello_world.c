#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "app.h"
#include "task.h"

#define DEBUG_MODULE "HELLOWORLD"
#include "debug.h"

#define ARR_LEN 10

int* allocate_and_initialize(int size)
{
    // Allocate memory for an array of 10 integers.
    int *arr = (int *)malloc(size * sizeof(int));
    if (arr == NULL) {
        DEBUG_PRINT("Memory allocation failed.\n");
    }

    // Initialize the array with values.
    for (int i = 0; i < size; i++) {
        arr[i] = i;
    }

    return arr;
}

void appMain() 
{
    DEBUG_PRINT("Waiting for activation ...\n");
    int size = 10;

    // // Allocate memory for an array of 10 integers.
    // int *arr = (int *)malloc(size * sizeof(int));
    // if (arr == NULL) {
    //     DEBUG_PRINT("Memory allocation failed.\n");
    // }


    // // Initialize the array with values.
    // for (int i = 0; i < size; i++) {
    //     arr[i] = i;
    // }

    int *arr = allocate_and_initialize(size);

    // Print the array elements.
    DEBUG_PRINT("Array elements are:\n");
    for (int i = 0; i < size; i++) {
        DEBUG_PRINT("%d \n", arr[i]);
    }

    DEBUG_PRINT("\n");

    // Free the memory.
    free(arr);

    while(1) {
        vTaskDelay(M2T(2000));
        DEBUG_PRINT("Hello World!\n");
    }
}

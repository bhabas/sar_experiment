#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "app.h"
#include "task.h"

#define DEBUG_MODULE "HELLOWORLD"
#include "debug.h"

double str_to_double(const char *str) {
    double result = 0, factor = 1;

    if (*str == '-') {
        str++;
        factor = -1;
    }

    for (int decimal_seen = 0; *str; str++) {
        if (*str == '.') {
            decimal_seen = 1; 
            continue;
        }

        int digit = *str - '0';
        if (digit >= 0 && digit <= 9) {
            if (decimal_seen) factor /= 10.0;
            result = result * 10.0 + (double)digit;
        }
    }

    return result * factor;
}

void appMain() 
{
    DEBUG_PRINT("Waiting for activation ...\n");
    
    // CONVERSION TO INT WORKS
    char int_string[] = "4";
    consolePrintf("Int Str: %s\n",int_string);
    int int_val = atoi(int_string);
    consolePrintf("Int Val: %d\n",int_val);

    // CONVERSION TO FLOAT DOESN'T WORK
    char float_string[] = "-7.124";
    consolePrintf("Float Str: %s\n",float_string);
    double float_val = str_to_double(float_string);
    consolePrintf("Float Val: %f\n",float_val);



    while(1) {
        vTaskDelay(M2T(2000));
        DEBUG_PRINT("Hello World!\n");
    }
}

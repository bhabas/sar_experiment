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

uint8_t delimiter = '\1';
uint8_t escape = '\2';

uint8_t receivedBytes[6];  
int byteIndex = 0;
bool lastByteWasEscape = false;
int32_t value = 0;

#define NUM_VALUES 10  // The number of values expected
int32_t receivedValues[NUM_VALUES];  // Array to store received values
int receivedValuesIndex = 0;  // Counter for number of received values

void appMain() {
    DEBUG_PRINT("Waiting for activation ...\n");
    uart1Init(115200);

    while(1) {

        uint8_t byte;
        uart1GetBytesWithDefaultTimeout(1, &byte);

        if (lastByteWasEscape) {
            receivedBytes[byteIndex++] = byte - '\2';
            lastByteWasEscape = false;
        } 
        else if (byte == escape) {
            lastByteWasEscape = true;
        } 
        else if (byte == delimiter) {
            memcpy(&value, receivedBytes, sizeof(int32_t));
            receivedValues[receivedValuesIndex++] = value;  // Store received value
            consolePrintf("Received Value: %ld\n", value);
            byteIndex = 0;  

            // Reset counter if we received the expected number of values
            if (receivedValuesIndex >= NUM_VALUES) {
                receivedValuesIndex = 0;
            }
        } 
        else {
            receivedBytes[byteIndex++] = byte;
        }
    }
}

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

void appMain() {
    DEBUG_PRINT("Waiting for activation ...\n");
    uart1Init(115200);

    while(1) {
        // vTaskDelay(M2T(500));

        uint8_t byte;
        uart1GetBytesWithDefaultTimeout(1, &byte);

        // consolePrintf("Received Byte: %02X\n", byte); 

        if (lastByteWasEscape) {
            receivedBytes[byteIndex++] = byte - '\2';
            lastByteWasEscape = false;
        } else if (byte == escape) {
            lastByteWasEscape = true;
        } else if (byte == delimiter) {
            memcpy(&value, receivedBytes, sizeof(int32_t));
            consolePrintf("Received Value: %ld\n", value);
            byteIndex = 0;  // Reset byteIndex for the next value
        } else {
            receivedBytes[byteIndex++] = byte;
        }
    }
}

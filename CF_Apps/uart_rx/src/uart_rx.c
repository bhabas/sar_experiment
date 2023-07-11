#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uart1.h"
#include "log.h"
#include "debug.h"

#define DEBUG_MODULE "UART"
#define NUM_VALUES 10  // The number of values expected


uint8_t delimiter = '\1';
uint8_t escape = '\2';
uint8_t startMarker = '\3';
uint8_t endMarker = '\4';

uint8_t receivedBytes[6];  
int byteIndex = 0;
bool lastByteWasEscape = false;
bool startMarkerReceived = false;


int32_t value = 0;
int32_t val_arr[NUM_VALUES];  // Array to store received values
int valIndex = 0;  // Counter for number of received values
bool arr_full = false;

void appMain() {
    DEBUG_PRINT("Waiting for activation ...\n");
    uart1Init(115200);

    while(1) {

        uint8_t byte;
        uart1GetBytesWithDefaultTimeout(1, &byte);

        // HANDLE START MARKER
        if (byte == startMarker) {
            startMarkerReceived = true;
            byteIndex = 0;  // Reset byte index
            arr_full = false;
            continue;
        }

        if (!startMarkerReceived) {
            continue;  // If start marker not received yet, don't process bytes
        }

        // HANDLE END MARKER
        if (byte == endMarker) {
            startMarkerReceived = false;    // Ready to receive a new array
            valIndex = 0;                   // Reset the array index
            continue;
        }

        // HANDLE ESCAPE SEQUENCE
        if (lastByteWasEscape) {
            receivedBytes[byteIndex++] = byte - '\2';
            lastByteWasEscape = false;
        } 
        else if (byte == escape) {
            lastByteWasEscape = true;
        } 
        else if (byte == delimiter) {
            memcpy(&value, receivedBytes, sizeof(int32_t));
            val_arr[valIndex++] = value;  // Store received value
            byteIndex = 0;  

            // Reset counter if we received the expected number of values
            if (valIndex >= NUM_VALUES) {
                valIndex = 0;
                arr_full = true;

            }
        } 
        else {
            receivedBytes[byteIndex++] = byte;
        }

        if (arr_full == true)
        {
            for (int i = 0; i < NUM_VALUES; i++) {
                consolePrintf("%ld ", val_arr[i]);
            }
            consolePrintf("\n");
        }
        
        
    }
}

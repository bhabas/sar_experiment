// Import necessary standard libraries
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// Include necessary project specific files
#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uart1.h"
#include "log.h"

// Define DEBUG_MODULE for debugging
#define DEBUG_MODULE "HELLOWORLD"
#include "debug.h"
#include "log.h"

// Set control characters for UART transmission
uint8_t delimiter = '\1';
uint8_t escape = '\2';

// Initialize array to hold received bytes, set maximum size to 6
uint8_t receivedBytes[6];
// Index to track position in the receivedBytes array
int byteIndex = 0;
// Boolean flag to track if the last received byte was the escape character
bool lastByteWasEscape = false;
// Integer to hold the final received value
int32_t value = 0;

void appMain() {
    // Print debug message
    DEBUG_PRINT("Waiting for activation ...\n");
    // Initialize UART communication
    uart1Init(115200);

    // Main loop
    while(1) {
        // Delay task execution for 500 milliseconds
        vTaskDelay(M2T(500));

        uint8_t byte;
        // Receive one byte from UART with default timeout
        uart1GetBytesWithDefaultTimeout(1, &byte);

        // Print each received byte in hexadecimal format
        consolePrintf("Received Byte: %02X\n", byte); 

        // Check conditions based on received byte
        if (lastByteWasEscape) {
            // If the last byte was an escape character, subtract 2 from current byte
            // and store it in the buffer, reset escape flag
            receivedBytes[byteIndex++] = byte - '\2';
            lastByteWasEscape = false;
        } 
        else if (byte == escape) {
            // If the current byte is an escape character, set escape flag to true
            lastByteWasEscape = true;
        } 
        else if (byte == delimiter) {
            // If the current byte is a delimiter, copy the bytes collected till now 
            // as a complete int32_t value into `value` and print it, reset the byte index
            memcpy(&value, receivedBytes, sizeof(int32_t));
            consolePrintf("Received Value: %ld\n", value);
            byteIndex = 0;
        } 
        else {
            // If current byte is none of the above, just add it to the buffer
            receivedBytes[byteIndex++] = byte;
        }
    }
}

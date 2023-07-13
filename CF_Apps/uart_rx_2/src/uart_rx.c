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
#define NUM_VALUES 10

uint8_t startMarker = '\3';
uint8_t endMarker = '\4';

typedef enum
{
    WAIT_FOR_START,
    WAIT_FOR_SIZE,
    WAIT_FOR_PAYLOAD,
    WAIT_FOR_END,
} ReadState;

int32_t val_arr[NUM_VALUES];
int valIndex = 0;

void appMain() {
    DEBUG_PRINT("Waiting for activation ...\n");
    uart1Init(115200);

    ReadState state = WAIT_FOR_START;
    uint8_t expectedSize = 0;
    uint8_t byteCount = 0;
    uint8_t buffer[sizeof(int32_t)];

    while(1) {

        uint8_t byte;
        uart1GetBytesWithDefaultTimeout(1, &byte);

        switch (state)
        {
            case WAIT_FOR_START:
                if (byte == startMarker) {
                    valIndex = 0;
                    state = WAIT_FOR_SIZE;
                }
                break;

            case WAIT_FOR_SIZE:
                if (byte == sizeof(int32_t)) {
                    expectedSize = byte;
                    byteCount = 0;
                    state = WAIT_FOR_PAYLOAD;
                } else {
                    // Invalid size byte, go back to waiting for start marker
                    state = WAIT_FOR_START;
                }
                break;

            case WAIT_FOR_PAYLOAD:
                buffer[byteCount++] = byte;
                if (byteCount == expectedSize) {
                    int32_t value;
                    memcpy(&value, buffer, sizeof(int32_t));
                    val_arr[valIndex++] = value;
                    state = WAIT_FOR_SIZE;  // Ready for next value size
                }
                break;

            case WAIT_FOR_END:
                if (byte == endMarker) {
                    if (valIndex == NUM_VALUES) {
                        // Print received values
                        for (int i = 0; i < NUM_VALUES; i++) {
                            consolePrintf("%ld ", val_arr[i]);
                        }
                        consolePrintf("\n");
                    }
                    // Regardless of whether we got a complete message or not,
                    // we're ready to process the next one.
                    state = WAIT_FOR_START;
                } else {
                    // Invalid end marker, keep waiting for end marker
                }
                break;
        }
    }
}

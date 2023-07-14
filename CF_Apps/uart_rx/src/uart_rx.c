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
#define NUM_VALUES 2


// Define your markers
uint8_t START_MARKER[] = {0xAA, 0xBB, 0xCC, 0xCD};
uint8_t END_MARKER[] = {0xFF, 0xEE, 0xDD, 0xDC};

#define STATE_WAIT_START 0
#define STATE_RECEIVE_DATA 1
#define STATE_WAIT_END 2


// Initialize state and data array
int state = STATE_WAIT_START;

int32_t data[NUM_VALUES];
int data_counter = 0;


int buffer_counter = 0;

int32_t bytes_to_int32(uint8_t* bytes) {
    int32_t result = 0;
    for (int i = 0; i < 4; i++) {
        result |= ((int32_t) bytes[i]) << (i*8);
    }
    return result;
}

void appMain() {
    DEBUG_PRINT("Waiting for activation ...\n");
    uart1Init(115200);

    uint8_t buffer[4];

    while(1) {

        // uart1Getchar(&received_byte);
        // COLLECT NEW 4-BYTE BUFFER
        uart1GetBytesWithDefaultTimeout(4,buffer);

        // for (int i = 0; i < 4; i++)
        // {
        //     consolePrintf("%02X ",buffer[i]);
        // }
        // consolePrintf("\n");

        switch (state)
        {
            case STATE_WAIT_START:

                // CHECK IF BUFFER MATCHES START SEQUENCE
                if (memcmp(buffer, START_MARKER, sizeof(START_MARKER)) == 0) {
                    state = STATE_RECEIVE_DATA;
                    buffer_counter = 0;
                    consolePrintf("1\n");
                    break;
                }

                break;

            case STATE_RECEIVE_DATA:

                // data[data_counter] = bytes_to_int32(buffer);
                consolePrintf("Val: %d \t %ld\n",data_counter,bytes_to_int32(buffer));
                data_counter++;
         
                if(data_counter == NUM_VALUES)
                {
                    state = STATE_WAIT_END;
                    consolePrintf("2\n");
                }
                break;

            case STATE_WAIT_END:

                break;

            
            default:
                break;
        }
        
        

        // switch (state)
        // {
        //     case STATE_WAIT_START:
                
        //         // Add the received byte to the buffer
        //         buffer[buffer_counter] = received_byte;
        //         buffer_counter++;

        //         // Check for both markers
        //         if (buffer_counter == sizeof(START_MARKER)) {
        //             if (memcmp(buffer, START_MARKER, sizeof(START_MARKER)) == 0) {
        //                 state = STATE_RECEIVE_DATA;
        //                 buffer_counter = 0;
        //                 consolePrintf("1\n");
        //                 break;
        //             }
        //         } 
        //         else if (buffer_counter == sizeof(START_MARKER_WITH_ZERO)) {
        //             if (memcmp(buffer, START_MARKER_WITH_ZERO, sizeof(START_MARKER_WITH_ZERO)) == 0) {
        //                 state = STATE_RECEIVE_DATA;
        //                 buffer_counter = 0;
        //                 consolePrintf("2\n");
        //                 break;
        //             }
        //         }

        // }
    }
}

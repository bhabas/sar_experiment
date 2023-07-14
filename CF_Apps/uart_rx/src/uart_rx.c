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

// DEFINE AND INIT STATE MACHINE
#define STATE_WAIT_START 0
#define STATE_RECEIVE_DATA 1
#define STATE_WAIT_END 2
int state = STATE_WAIT_START;

// DATA CAP MARKERS
uint8_t START_MARKER[] = {0xAA, 0xBB, 0xCC, 0xCD};
uint8_t END_MARKER[] = {0xFF, 0xEE, 0xDD, 0xDC};

// INIT DATA ARRAY
#define NUM_VALUES 7
int32_t data[NUM_VALUES];
int data_counter = 0;

// ASSUME 4-BYTE CHUNK
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
                    data_counter = 0;
                    // consolePrintf("1\n");
                }

                break;

            case STATE_RECEIVE_DATA:

                // CONVERT BUFFER TO INT32_T DATA
                data[data_counter] = bytes_to_int32(buffer);
                data_counter++;
                // consolePrintf("Val: %d \t %ld\n",data_counter,bytes_to_int32(buffer));
         
                // TRANSITION TO NEXT STATE
                if(data_counter == NUM_VALUES)
                {
                    state = STATE_WAIT_END;
                    // consolePrintf("2\n");
                }
                break;

            case STATE_WAIT_END:

                // CHECK IF BUFFER MATCHES END SEQUENCE
                if (memcmp(buffer, END_MARKER, sizeof(END_MARKER)) == 0) {
                    // Print the received data
                    for (int i = 0; i < NUM_VALUES; i++) {
                        consolePrintf("%ld ", data[i]);
                    }
                    consolePrintf("\n");
                    // consolePrintf("\n3\n");

                    // RESET STATE TO WAIT FOR NEXT TRANSMISSION
                    state = STATE_WAIT_START;
                }

                break;

        }
        
    
    }
}

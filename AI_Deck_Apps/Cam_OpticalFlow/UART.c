#include "UART.h"

uint8_t message[MESSAGE_SIZE];
int msg_index = 0;

int send_uart_arr(struct pi_device *UART_device, int32_t uart_arr[])
{
    // FILL START MESSAGE
    msg_index = 0;
    for (size_t i = 0; i < sizeof(START_MARKER); i++) {
        message[msg_index++] = START_MARKER[i];
    }

    // FILL DATA ARRAY
    for (int i = 0; i < UART_ARR_SIZE; i++) {

        uint8_t *byte_array = int32_to_bytes(uart_arr[i]);

        // ADD BYTE TO BUFFER
        for (int j = 0; j < 4; j++) {
            message[msg_index++] = byte_array[j];
        }
    }

    // FILL END MESSAGE
    for (size_t i = 0; i < sizeof(END_MARKER); i++) {
        message[msg_index++] = END_MARKER[i];
    }
    
    int result = 0;
    result = pi_uart_write(UART_device,message,MESSAGE_SIZE);
    return result;

}








uint8_t* int32_to_bytes(int32_t value) {
    static uint8_t bytes[4];
    for(int i=0; i<4; i++) {
        bytes[i] = (value >> (i*8)) & 0xFF;
    }
    return bytes;
}
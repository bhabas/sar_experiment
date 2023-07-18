#include "UART.h"

uint8_t message[MESSAGE_SIZE];
int msg_index = 0;

/**
 * @brief Opens UART connection that communicates between the AI-Deck and Crazyflie. 
 * Connection is over the UART1 pins/functions and is used to pass an int32_t array containing 
 * gradient dot products and image capture data
 * 
 * @param device 
 * @return int32_t - Returns 1 if successful
 */
int32_t open_uart(struct pi_device *device)
{
    struct pi_uart_conf UART_config;
    pi_uart_conf_init(&UART_config);
    UART_config.baudrate_bps = 115200;
    UART_config.enable_tx = 1;
    UART_config.enable_rx = 0;

    pi_open_from_conf(device, &UART_config);
    if (pi_uart_open(device))
    {
        return 1;
    }
    
    // WRITE JUNK MSG BECAUSE OF EXTRA 00 SENT
    pi_uart_write(device,&OFFSET_MSG,3);
    printf("[UART] \t\tOpen\n");

    return 0;
}


/**
 * @brief Sends the designated UART array via UART1 to the Crazyflie. 
 * The size is locked to the length of UART_ARR_SIZE and all terms are int32_t
 * 
 * @param UART_device 
 * @param uart_arr 
 * @return int 
 */
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
    
    int result = 1;
    result = pi_uart_write(UART_device,message,MESSAGE_SIZE);
    return result;

}

/**
 * @brief Converts a single int32_t value into its uint8_t byte components
 * so they can be send via UART. It outputs a pointer to the first uint8_t byte
 * 
 * @param value 
 * @return uint8_t* 
 */
uint8_t* int32_to_bytes(int32_t value) {
    static uint8_t bytes[4];
    for(int i=0; i<4; i++) {
        bytes[i] = (value >> (i*8)) & 0xFF;
    }
    return bytes;
}
// Include necessary libraries
#include "stdlib.h"
#include "stdio.h"
#include "pmsis.h"

// Macro to reverse bytes in 32-bit number
#define reverse_bytes_32(num) ( ((num & 0xFF000000) >> 24) | ((num & 0x00FF0000) >> 8) | ((num & 0x0000FF00) << 8) | ((num & 0x000000FF) << 24) )

// Initialize control characters and value to be sent
int32_t valueToSend = 1234;
uint8_t delimiter = '\1';
uint8_t escape = '\2';

// Initialize result variable for function return codes
int result;

// Function to test UART on GAP8
static void test_gap8(void)
{
    printf("Entering main controller...\n");

    // Initialize and configure UART
    struct pi_uart_conf conf;
    struct pi_device device;
    
    pi_uart_conf_init(&conf);
    conf.baudrate_bps = 115200;
    conf.enable_tx = 1;
    conf.enable_rx = 0;

    // Open UART with the specified configuration
    pi_open_from_conf(&device, &conf);
    printf("[UART] Open\n");

    if (pi_uart_open(&device))
    {
        printf("[UART] open failed !\n");
        pmsis_exit(-1);
    }

    // Allocate buffer for storing the value to send
    uint8_t *buffer = pmsis_l2_malloc(6 * sizeof(uint8_t));  // Maximum possible size (each byte could be escaped and then the delimiter)
    int bufferSize = 0;

    // Reference the bytes of the value to send
    // Convert valueToSend to byte string format
    uint8_t *valueBytes = (uint8_t *)&valueToSend;
    
    // Go through each byte and handle control characters
    for (size_t i = 0; i < sizeof(int32_t); i++) {
        if (valueBytes[i] == delimiter || valueBytes[i] == escape) {
            buffer[bufferSize++] = escape;
            buffer[bufferSize++] = valueBytes[i] + '\2';
        } 
        else {
            buffer[bufferSize++] = valueBytes[i];
        }
    }

    // Add the delimiter at the end
    buffer[bufferSize++] = delimiter;

    // Continuous loop for sending the value
    while(1)
    {
        result = pi_uart_write(&device, buffer, bufferSize);

        if (result == 0) {
            printf("Value successfully sent!\n");
        } 
        else {
            printf("Failed to send value, error code: %d\n", result);
        }

        // Wait for 500 milliseconds before sending the value again
        pi_time_wait_us(500000);
    }

    pmsis_exit(0);
}

// Main function to kickoff the program
int main(void)
{
    return pmsis_kickoff((void *)test_gap8);
}

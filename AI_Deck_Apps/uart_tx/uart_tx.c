#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "pmsis.h"


#define NUM_VAL 11

// Define your markers
uint8_t START_MARKER[] = {0xAA, 0xBB, 0xCC, 0xCD};
uint8_t END_MARKER[] = {0xFF, 0xEE, 0xDD, 0xDC};
uint8_t JUNK_MSG[] = {0x00, 0x00, 0x00};
uint8_t JUNK_MSG2[4] = {0x55, 0xE3, 0x3D, 0x8C};;

// Initialize your data array
int32_t data[NUM_VAL] = {1234,2345,3456,4567,5678,6789,7890,9876,8765,180,100};

#define MESSAGE_SIZE (sizeof(START_MARKER) + NUM_VAL*sizeof(int32_t) + sizeof(END_MARKER))
uint8_t message[MESSAGE_SIZE];
int msg_index = 0;


uint8_t buffer[4];
int buffer_index = 0;

static int32_t open_uart(struct pi_device *device)
{
    struct pi_uart_conf UART_config;
    pi_uart_conf_init(&UART_config);
    UART_config.baudrate_bps = 115200;
    UART_config.enable_tx = 1;
    UART_config.enable_rx = 0;

    pi_open_from_conf(device, &UART_config);
    printf("[UART] Open\n");
    if (pi_uart_open(device))
    {
        printf("[UART] open failed !\n");
        pmsis_exit(-1);
    }

    return 0;
}

uint8_t* int32_to_bytes(int32_t value) {
    static uint8_t bytes[4];
    for(int i=0; i<4; i++) {
        bytes[i] = (value >> (i*8)) & 0xFF;
    }
    return bytes;
}

int send_uart_arr(struct pi_device *UART_device, int32_t uart_arr[])
{
    // FILL START MESSAGE
    msg_index = 0;
    for (size_t i = 0; i < sizeof(START_MARKER); i++) {
        message[msg_index++] = START_MARKER[i];
    }

    // FILL DATA ARRAY
    for (int i = 0; i < NUM_VAL; i++) {

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

    // // PRINT DATA MESSAGE BYTES
    // for (size_t i = 0; i < MESSAGE_SIZE; i++)
    // {
    //     printf("%02X ",message[i]);
    // }
    // printf("  \n");

    
    int result = 0;
    result = pi_uart_write(UART_device,message,MESSAGE_SIZE);
    return result;

}

static void test_gap8(void)
{
    printf("Entering main controller...\n");

    struct pi_device UART_device;
    if (open_uart(&UART_device))
    {
        printf("Failed to open UART\n");
        pmsis_exit(-1);
    }

    // WRITE JUNK MSG BECAUSE OF EXTRA 00 SENT
    pi_uart_write(&UART_device,&JUNK_MSG,3);

    // SEND NOISE MESSAGES FOR TEST
    for (int i = 0; i < 10; i++)
    {
        pi_uart_write(&UART_device,&JUNK_MSG2,4); 
        pi_time_wait_us(1000);
    }

    while (1)
    {

        // INCREMENT FIRST DATA VALUE
        data[0]++;

        send_uart_arr(&UART_device,data);
        pi_time_wait_us(100000);
    }
    
    pmsis_exit(0);
}

int main(void)
{
    return pmsis_kickoff((void *)test_gap8);
}

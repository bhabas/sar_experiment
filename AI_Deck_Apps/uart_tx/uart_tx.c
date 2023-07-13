#include "stdlib.h"
#include "stdio.h"
#include "pmsis.h"
#define NUM_VAL 4

// Define your markers
uint8_t START_MARKER[] = {0xAA, 0xBB};
uint8_t END_MARKER[] = {0xBB, 0xAA};


// Initialize your data array
int32_t data[NUM_VAL] = {0, 1, 3456, 4567}; // Fill your data here



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

static void test_gap8(void)
{
    printf("Entering main controller...\n");

    struct pi_device UART_device;
    if (open_uart(&UART_device))
    {
        printf("Failed to open UART\n");
        pmsis_exit(-1);
    }
    
    // Send START_MARKER
    for(size_t i=0; i < sizeof(START_MARKER); i++) 
    {
        pi_uart_write_byte(&UART_device,&START_MARKER[i]);
        printf("%02X ",START_MARKER[i]);
    }
    printf("  ");
    
    // Send each int32_t value as 4-byte sequence
    for(int i=0; i < NUM_VAL; i++) {

        uint8_t *byte_array = int32_to_bytes(data[i]);

        for(int j=0; j<4; j++) {
            pi_uart_write_byte(&UART_device,&byte_array[j]);
            printf("%02X ",byte_array[j]);
        }
        printf("  ");
    }
    printf(" ");

    // Send END_MARKER
    for(size_t i=0; i < sizeof(END_MARKER); i++) 
    {
        pi_uart_write_byte(&UART_device,&START_MARKER[i]);
        printf("%02X ",END_MARKER[i]);
    }
    

    // while(1)
    // {
    //     int result = pi_uart_write(&UART_device, buffer, bufferSize);
    //     pi_time_wait_us(1*1000000);
    // }

    pmsis_exit(0);
}

int main(void)
{
    return pmsis_kickoff((void *)test_gap8);
}

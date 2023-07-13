#include "stdlib.h"
#include "stdio.h"
#include "pmsis.h"
#define NUM_VAL 10

uint8_t startMarker = '\3';
uint8_t endMarker = '\4';

int32_t val_arr[NUM_VAL] = {0, 1, 3456, 4567, 5678, 6789, 7890, 8901, 9012, 10123}; 

uint8_t *buffer;

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

static void test_gap8(void)
{
    printf("Entering main controller...\n");

    struct pi_device UART_device;
    if (open_uart(&UART_device))
    {
        printf("Failed to open UART\n");
        pmsis_exit(-1);
    }
    
    buffer = pmsis_l2_malloc(NUM_VAL * (1 + sizeof(int32_t)) * sizeof(uint8_t));  

    int bufferSize = 0;
    buffer[bufferSize++] = startMarker;

    for (size_t valueIndex = 0; valueIndex < NUM_VAL; valueIndex++) 
    {
        int32_t valueToSend = val_arr[valueIndex];

        buffer[bufferSize++] = sizeof(int32_t);

        uint8_t *valueBytes = (uint8_t *)&valueToSend;

        memcpy(&buffer[bufferSize], valueBytes, sizeof(int32_t));
        bufferSize += sizeof(int32_t);
    }

    buffer[bufferSize++] = endMarker;

    while(1)
    {
        int result = pi_uart_write(&UART_device, buffer, bufferSize);
        pi_time_wait_us(1*1000000);
    }

    pmsis_exit(0);
}

int main(void)
{
    return pmsis_kickoff((void *)test_gap8);
}

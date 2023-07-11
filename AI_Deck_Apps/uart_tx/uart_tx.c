#include "stdlib.h"
#include "stdio.h"
#include "pmsis.h"
#define reverse_bytes_32(num) ( ((num & 0xFF000000) >> 24) | ((num & 0x00FF0000) >> 8) | ((num & 0x0000FF00) << 8) | ((num & 0x000000FF) << 24) )

uint8_t delimiter = '\1';
uint8_t escape = '\2';
int32_t valuesToSend[10] = {1234, 2345, 3456, 4567, 5678, 6789, 7890, 8901, 9012, 10123}; 

static void test_gap8(void)
{
    printf("Entering main controller...\n");

    // set configurations in uart
    struct pi_uart_conf conf;
    struct pi_device device;
    pi_uart_conf_init(&conf);
    conf.baudrate_bps = 115200;
    conf.enable_tx = 1;
    conf.enable_rx = 0;

    // Open uart
    pi_open_from_conf(&device, &conf);
    printf("[UART] Open\n");
    if (pi_uart_open(&device))
    {
        printf("[UART] open failed !\n");
        pmsis_exit(-1);
    }

    pi_uart_open(&device);

    // Make sure to allocate enough space
    uint8_t *buffer = pmsis_l2_malloc(10 * 6 * sizeof(uint8_t));  
    int bufferSize = 0;

    for (size_t valueIndex = 0; valueIndex < 10; ++valueIndex) {
        int32_t valueToSend = valuesToSend[valueIndex];
        uint8_t *valueBytes = (uint8_t *)&valueToSend;

        for (size_t i = 0; i < sizeof(int32_t); i++) {
            if (valueBytes[i] == delimiter || valueBytes[i] == escape) {
                buffer[bufferSize++] = escape;
                buffer[bufferSize++] = valueBytes[i] + '\2';
            } else {
                buffer[bufferSize++] = valueBytes[i];
            }
        }

        buffer[bufferSize++] = delimiter;
    }

    // while(1)
    // {
        int result = pi_uart_write(&device, buffer, bufferSize);

        if (result == 0) {
            printf("Values successfully sent!\n");
        } else {
            printf("Failed to send values, error code: %d\n", result);
        }

        pi_time_wait_us(500000);
    // }

    pmsis_exit(0);
}

int main(void)
{
    return pmsis_kickoff((void *)test_gap8);
}

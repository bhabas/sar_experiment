/*
 * Copyright (C) 2018 GreenWaves Technologies
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "pmsis.h"
#include "stdio.h"
#define reverse_bytes_32(num) ( ((num & 0xFF000000) >> 24) | ((num & 0x00FF0000) >> 8) | ((num & 0x0000FF00) << 8) | ((num & 0x000000FF) << 24) )


static PI_L2 uint8_t command;

static PI_L2 int32_t  valueToSend = 1234;
static PI_L2 uint8_t buffer[sizeof(int32_t) + 1];
int result;

static pi_task_t led_task;
static int led_val = 0;

static struct pi_device gpio_device;



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

    // configure LED
    pi_gpio_pin_configure(&gpio_device, 2, PI_GPIO_OUTPUT);

    // Open uart
    pi_open_from_conf(&device, &conf);
    printf("[UART] Open\n");
    if (pi_uart_open(&device))
    {
        printf("[UART] open failed !\n");
        pmsis_exit(-1);
    }

    pi_uart_open(&device);
    valueToSend = reverse_bytes_32(valueToSend);
    
    memcpy(buffer, &valueToSend, sizeof(int32_t));
    buffer[sizeof(int32_t)] = '\0';  // Append the delimiter

    while(1)
    {

        // Send the value
        result = pi_uart_write(&device, buffer, sizeof(buffer));

        if (result == 0) {
            printf("Value successfully sent!\n");
        } else {
            printf("Failed to send value, error code: %d\n", result);
        }



        pi_time_wait_us(500000);
    }

    pmsis_exit(0);
}

int main(void)
{
    return pmsis_kickoff((void *)test_gap8);
}

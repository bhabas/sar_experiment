#pragma once

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/* PMSIS includes */
#include "pmsis.h"

#define UART_ARR_SIZE 16
#define MESSAGE_SIZE (sizeof(START_MARKER) + UART_ARR_SIZE*sizeof(int32_t) + sizeof(END_MARKER))

// Define your markers
static uint8_t START_MARKER[4] = {0xAA, 0xBB, 0xCC, 0xCD};
static uint8_t END_MARKER[4] = {0xFF, 0xEE, 0xDD, 0xDC};
static uint8_t OFFSET_MSG[3] = {0x00, 0x00, 0x00};
static uint8_t JUNK_MSG_TEST[4] = {0x55, 0xE3, 0x3D, 0x8C};


static int32_t open_uart(struct pi_device *device);
uint8_t* int32_to_bytes(int32_t value);
int send_uart_arr(struct pi_device *UART_device, int32_t uart_arr[]);





/**
 * @brief Opens UART connection that communicates between the AI-Deck and Crazyflie. 
 * Connection is over the UART1 pins/functions and is used to pass an int32_t array containing 
 * gradient dot products and image capture data
 * 
 * @param device 
 * @return int32_t - Returns 1 if successful
 */
static int32_t open_uart(struct pi_device *device)
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
    printf("[UART] Open\n");
    
    // WRITE JUNK MSG BECAUSE OF EXTRA 00 SENT
    pi_uart_write(device,&OFFSET_MSG,3);

    return 0;
}
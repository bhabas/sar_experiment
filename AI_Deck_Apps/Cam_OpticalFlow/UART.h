#pragma once

/* Standard includes */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/* PMSIS includes */
#include "pmsis.h"

#define UART_ARR_SIZE 16
#define MESSAGE_SIZE (sizeof(START_MARKER) + UART_ARR_SIZE*sizeof(int32_t) + sizeof(END_MARKER))

// DEFINE MARKERS
static uint8_t START_MARKER[4] = {0xAA, 0xBB, 0xCC, 0xCD};
static uint8_t END_MARKER[4] = {0xFF, 0xEE, 0xDD, 0xDC};
static uint8_t OFFSET_MSG[3] = {0x00, 0x00, 0x00};
static uint8_t JUNK_MSG_TEST[4] = {0x55, 0xE3, 0x3D, 0x8C};

// FUNCTION PROTOTYPES
int32_t open_uart(struct pi_device *device);
uint8_t* int32_to_bytes(int32_t value);
int send_uart_arr(struct pi_device *UART_device, int32_t uart_arr[]);
uint8_t* create_uart_msg(int32_t uart_arr[]);



#pragma once 

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>


/* PMSIS includes */
#include "pmsis.h"

/* PMSIS BSP includes. */
#include "bsp/bsp.h"
#include "bsp/camera.h"

#include "CalcFuncs.h"
#include "params.h"


// CAMERA BUFFERS AND TASKS
struct pi_device CL_device;
struct pi_device Cam_device;
struct pi_device UART_device;
static pi_task_t Cam_Capture_Task;



uint8_t* ImgBuff[NUM_BUFFERS];
int32_t G_up[N_up*N_vp] = {0};
int32_t G_vp[N_up*N_vp] = {0};
int32_t G_rp[N_up*N_vp] = {0};
int32_t G_tp[N_up*N_vp] = {0};

uint8_t prev_img_index = 0;
uint8_t cur_img_index = 1;
uint8_t capture_index = 2;

uint32_t t_delta[NUM_BUFFERS] = {0};
uint32_t t = 0;


uint32_t time_before = 0;
uint32_t time_after = 0;

// PERFORMANCE MEASURING VARIABLES
volatile uint8_t buffer_index = 0;
volatile uint8_t img_count = 0;

typedef struct ClusterImageData{
    uint8_t* Cur_img_buff;
    uint8_t* Prev_img_buff;

    uint8_t  Rows_Per_Core;
    int32_t  stride;

    // DOT PRODUCT TERMS
    int32_t* DP_Vec1;
    int32_t* DP_Vec2;
    int32_t  DP_Sum_array[NUM_CORES];
    int32_t* DP_Sum;

    int32_t  UART_array[UART_ARR_SIZE];
} ClusterImageData_t;

// CONVOLUATION KERNEL U_p-DIRECTION
int32_t Ku[9] = {-1, 0, 1,
                 -2, 0, 2,
                 -1, 0, 1};

// CONVOLUATION KERNEL V_p-DIRECTION
int32_t Kv[9] = {-1,-2,-1,
                  0, 0, 0,
                  1, 2, 1};

// SEPERABLE CONVOLUTION KERNES U_p-DIRECTION
int32_t Ku_v[3] = { 1, 2, 1};
int32_t Ku_h[3] = {-1, 0, 1};

// SEPERABLE CONVOLUTION KERNES V_p-DIRECTION
int32_t Kv_v[3] = {-1, 0, 1};
int32_t Kv_h[3] = { 1, 2, 1};


void Delegate_Gradient_Calcs(void *arg);
void Cluster_GradientCalcs(void *arg);
void Delegate_DotProduct_Calcs(void *arg);
void Cluster_DotProduct(void *arg);




static int32_t open_pi_camera_himax(struct pi_device *device)
{
    // CAMERA CONFIG
    struct pi_himax_conf cam_config;
    pi_himax_conf_init(&cam_config);
    cam_config.format = PI_CAMERA_QQVGA;

    // OPEN CAMERA
    pi_open_from_conf(device, &cam_config);
    if (pi_camera_open(device))
    {
        return 1;
    }


    // ROTATE CAMERA IMAGE
    pi_camera_control(&Cam_device, PI_CAMERA_CMD_START, 0);

    uint8_t set_value = 3;
    uint8_t reg_value;

    pi_camera_reg_set(&Cam_device, IMG_ORIENTATION, &set_value);
    pi_time_wait_us(500000);
    pi_camera_reg_get(&Cam_device, IMG_ORIENTATION, &reg_value);

    if (set_value != reg_value)
    {
        printf("[CAMERA] Couldn't rotate image\n");
        return 1;
    }
                
    pi_camera_control(&Cam_device, PI_CAMERA_CMD_STOP, 0);
    // pi_camera_control(device, PI_CAMERA_CMD_AEG_INIT, 0);
    pi_time_wait_us(1000000); // Give time for Cam_device to adjust exposure

    printf("[CAMERA] Open\n");
    return 0;
}

static int32_t open_cluster(struct pi_device *device)
{

    // CLUSTER CONFIG
    struct pi_cluster_conf CL_Config;
    pi_cluster_conf_init(&CL_Config);
    CL_Config.id = 0;
    
    // OPEN CLUSTER DEVICE
    pi_open_from_conf(device, &CL_Config);
    if (pi_cluster_open(device))
    {
        return 1;
    }
        

    pi_freq_set(PI_FREQ_DOMAIN_CL, CLOCK_FREQ_CLUSTER);
    printf("[CLUSTER] Open\n");

    return 0;
}

static int32_t open_uart(struct pi_device *device)
{
    // UART CONFIG
    struct pi_uart_conf UART_config;
    pi_uart_conf_init(&UART_config);
    UART_config.baudrate_bps = 115200;
    UART_config.enable_tx = 1;
    UART_config.enable_rx = 0;

    // OPEN UART DEVICE
    pi_open_from_conf(device, &UART_config);
    if (pi_uart_open(device))
    {
        return 1;
    }

    printf("[UART] Open\n");
    return 0;
}
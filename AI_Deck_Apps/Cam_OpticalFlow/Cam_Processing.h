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
#include "UART.h"


// CAMERA BUFFERS AND TASKS
struct pi_device CL_device;
struct pi_device Cam_device;
struct pi_device UART_device;
static pi_task_t Cam_Capture_Task;

// IMAGE BUFFERS AND INDEXES
uint8_t* ImgBuff[NUM_BUFFERS];
uint8_t prev_img_index = 0;
uint8_t cur_img_index = 1;
uint8_t capture_index = 2;

// IMAGE GRADIENTS
int32_t G_up[N_up*N_vp] = {0};
int32_t G_vp[N_up*N_vp] = {0};
int32_t G_rp[N_up*N_vp] = {0};
int32_t G_tp[N_up*N_vp] = {0};
uint32_t t_delta[NUM_BUFFERS] = {0};
uint32_t t_capture[NUM_BUFFERS] = {0};
uint32_t t_cap = 0;
uint32_t t_start = 0;



typedef struct ClusterCompData{
    uint8_t* Cur_img_buff;
    uint8_t* Prev_img_buff;

    uint8_t  Rows_Per_Core;
    int32_t  Stride;

    // DOT PRODUCT TERMS
    int32_t* DP_Vec1;
    int32_t* DP_Vec2;
    int32_t  DP_Sum_array[NUM_CORES];
    int32_t* DP_Sum;

    int32_t  UART_array[UART_ARR_SIZE];
} ClusterCompData_t;

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


// PERFORMANCE MEASURING VARIABLES
uint32_t time_before = 0;
uint32_t time_after = 0;
volatile uint32_t img_count = 0;


void Delegate_Gradient_Calcs(void *arg);
void Cluster_GradientCalcs(void *arg);
void Delegate_DotProduct_Calcs(void *arg);
void Cluster_DotProduct(void *arg);

/**
 * @brief Opens camera device on the AI-Deck for future use. Camera configuration is defined here
 * and is set to QQVGA (162 x 122) pixel format to minimize image size and maximize refresh rate.
 * 
 * @param device 
 * @return int32_t 
 */
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


    // ROTATE CAMERA IMAGE TO BE UPRIGHT
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
    // pi_camera_control(device, PI_CAMERA_CMD_AEG_INIT, 0);  // Auto-Exposure Gain (Not sure if good or bad yet)
    pi_time_wait_us(1*1000000); // Give time for Cam_device to adjust exposure

    printf("[CAMERA] \tOpen\n");
    return 0;
}

/**
 * @brief Opens cluster device on GAP8 processor for future use. The cluster consists of 8 cores where tasks
 * can be allocated and split between them for parallel computation
 * 
 * @param device 
 * @return int32_t - Returns 1 if successful
 */
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
    printf("[CLUSTER] \tOpen\n");

    return 0;
}


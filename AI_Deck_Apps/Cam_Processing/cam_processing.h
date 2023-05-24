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

#include "cam_processing.h"
#include "test.h"
#include "params.h"


// CAMERA BUFFERS AND TASKS
struct pi_device Cam_device;
struct pi_device CL_device;

// CLUSTER SETUP

uint32_t time_before = 0;
uint32_t time_after = 0;


uint8_t* ImgBuff[NUM_BUFFERS];

volatile int current_idx = 0;
volatile int next_idx = 0;


int32_t G_up[CAM_WIDTH*CAM_HEIGHT];
int32_t G_vp[CAM_WIDTH*CAM_HEIGHT];
int32_t G_rp[CAM_WIDTH*CAM_HEIGHT];
int32_t G_tp[CAM_WIDTH*CAM_HEIGHT];


// PERFORMANCE MEASURING VARIABLES
volatile uint8_t buffer_index = 0;
volatile uint8_t img_num_async = 0;

typedef struct ClusterImageData{
    uint8_t  Rows_Per_Core;
    uint8_t* Cur_img_buff;
    uint8_t* Prev_img_buff;
    int32_t  UART_array[10];

    int32_t* DP_Vec1;
    int32_t* DP_Vec2;
    int32_t  DP_Sum_array[NUM_CORES];
    int32_t* DP_Sum;
} ClusterImageData_t;

int32_t Ku[9] = {-1, 0, 1,
                 -2, 0, 2,
                 -1, 0, 1};
int32_t Ku_v[3] = { 1, 2, 1};
int32_t Ku_h[3] = {-1, 0, 1};

int32_t Kv[9] = {-1,-2,-1,
                  0, 0, 0,
                  1, 2, 1};

int32_t Kv_v[3] = {-1, 0, 1};
int32_t Kv_h[3] = { 1, 2, 1};


void delegate_GradCalcs(void *arg);
void CL_GradCalcs(void *arg);
void delegate_DotProducts(void *arg);
void CL_DotProduct(void *arg);




static int32_t open_pi_camera_himax(struct pi_device *device)
{
    // CAMERA CONFIG
    struct pi_himax_conf cam_config;
    pi_himax_conf_init(&cam_config);
    cam_config.format = PI_CAMERA_QQVGA;

    // OPEN CAMERA
    pi_open_from_conf(device, &cam_config);
    if (pi_camera_open(device))
        return -1;

    // ROTATE CAMERA IMAGE
    pi_camera_control(&Cam_device, PI_CAMERA_CMD_START, 0);

 
    uint8_t set_value = 3;
    uint8_t reg_value;
    pi_camera_reg_set(&Cam_device, IMG_ORIENTATION, &set_value);
    pi_time_wait_us(500000);
    pi_camera_reg_get(&Cam_device, IMG_ORIENTATION, &reg_value);

    if (set_value != reg_value)
    {
        printf("Failed to rotate Cam_device image\n");
        return -1;
    }
                
    pi_camera_control(&Cam_device, PI_CAMERA_CMD_STOP, 0);
    // pi_camera_control(device, PI_CAMERA_CMD_AEG_INIT, 0);
    pi_time_wait_us(1000000); // Give time for Cam_device to adjust exposure


    return 0;
}

static int32_t open_cluster(struct pi_device *device)
{
    struct pi_cluster_conf CL_Config;

    // CLUSTER CONFIG
    pi_cluster_conf_init(&CL_Config);
    CL_Config.id = 0;
    
    // OPEN CLUSTER
    pi_open_from_conf(&CL_device, &CL_Config);
    if (pi_cluster_open(device))
        return -1;

    pi_freq_set(PI_FREQ_DOMAIN_CL, CLOCK_FREQ_CL);

    return 0;
}
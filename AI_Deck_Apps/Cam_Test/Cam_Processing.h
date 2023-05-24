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
static pi_task_t Cam_task;



uint8_t* ImgBuff[NUM_BUFFERS];
int32_t G_up[CAM_WIDTH*CAM_HEIGHT] = {0};
int32_t G_vp[CAM_WIDTH*CAM_HEIGHT] = {0};
int32_t G_rp[CAM_WIDTH*CAM_HEIGHT] = {0};
int32_t G_tp[CAM_WIDTH*CAM_HEIGHT] = {0};

uint8_t process_index1 = 0;
uint8_t process_index2 = 1;
uint8_t fill_index = 2;

uint32_t t_delta[NUM_BUFFERS] = {0};
uint32_t t = 0;


uint32_t time_before = 0;
uint32_t time_after = 0;

// PERFORMANCE MEASURING VARIABLES
volatile uint8_t buffer_index = 0;
volatile uint8_t img_num_async = 0;

typedef struct ClusterImageData{
    uint8_t  Rows_Per_Core;
    uint8_t* Cur_img_buff;
    uint8_t* Prev_img_buff;
    int32_t  UART_array[10];
    int32_t  stride;

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
    {
        printf("[Camera] open failed !\n");
        return -1;
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
    {
        return -1;
    }
        

    pi_freq_set(PI_FREQ_DOMAIN_CL, CLOCK_FREQ_CL);

    return 0;
}

static int32_t open_uart(struct pi_device *device)
{
    // SET UART CONFIGURATIONS
    struct pi_uart_conf conf;
    pi_uart_conf_init(&conf);

    conf.baudrate_bps = 9600;   // Baud Rate - Must match receiver
    conf.enable_tx = 1;         // Enable data transfer (TX)
    conf.enable_rx = 0;         // Disable data reception (RX)

    // OPEN UART CONNECTION
    pi_open_from_conf(&UART_device, &conf);
    if (pi_uart_open(&UART_device))
    {
        return -1;
    }

    float value_arr[3] = {1.0f,2.0f,3.0f};

    while(1)
    {
        // WRITE VALUE ARRAY TO CF OVER UART1
        value_arr[0] += 1; // Increment first value of array
        pi_uart_write(&UART_device, &value_arr, sizeof(value_arr));

        // WAIT
        pi_time_wait_us(500000); // Wait 0.5s [500,000 us]
    }


    return 0;
}
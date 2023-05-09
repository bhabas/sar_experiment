#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>


/* PMSIS includes */
#include "pmsis.h"
#include "bsp/bsp.h"
#include "bsp/camera.h"
#include "bsp/camera/himax.h"
#include "bsp/buffer.h"

#define IMG_ORIENTATION 0x0101
#define CAM_WIDTH 162
#define CAM_HEIGHT 122
#define CLOCK_FREQ 250000000

#define NUM_BUFFERS 4
#define RESOLUTION CAM_WIDTH*CAM_HEIGHT
#define BUFFER_SIZE CAM_WIDTH*CAM_HEIGHT*sizeof(uint8_t)

void capture_callback(void *arg);

// CAMERA BUFFERS AND TASKS
static struct pi_device camera;

uint8_t *buffers[NUM_BUFFERS];
pi_buffer_t pi_buffers[NUM_BUFFERS];

// PERFORMANCE MEASURING VARIABLES
volatile uint8_t buffer_index = 0;
volatile uint8_t img_num_async = 0;
volatile uint32_t clock_cycles_async = 0;


static int open_pi_camera_himax(struct pi_device *device)
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
    pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
    uint8_t set_value = 3;
    uint8_t reg_value;
    pi_camera_reg_set(&camera, IMG_ORIENTATION, &set_value);
    pi_time_wait_us(500000);
    pi_camera_reg_get(&camera, IMG_ORIENTATION, &reg_value);

    if (set_value != reg_value)
    {
        printf("Failed to rotate camera image\n");
        return -1;
    }
                
    pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);
    pi_camera_control(device, PI_CAMERA_CMD_AEG_INIT, 0);
    pi_time_wait_us(1000000); // Give time for camera to adjust exposure


    return 0;
}

void capture_callback(void *arg)
{
    // pi_time_wait_us(100000);
    img_num_async++;

    pi_task_t capture_task = {0};
    pi_task_callback(&capture_task, capture_callback, NULL);
    pi_camera_capture_async(&camera, buffers[0], BUFFER_SIZE, &capture_task);
        
}



void Cam_Example(void)
{
    printf("-- Starting Camera Test --\n");
    pi_perf_conf(1 << PI_PERF_CYCLES);

    
    // INITIALIZE CAMERA
    if (open_pi_camera_himax(&camera))
    {
        printf("Failed to open camera\n");
        return;
    }

    for (int i = 0; i < NUM_BUFFERS; i++)
    {
        // INITIALIZE BUFFERS
        buffers[i] = (uint8_t *)pmsis_l2_malloc(BUFFER_SIZE);
        if (buffers[i] == NULL)
        {
            printf("Failed to allocate memory for image buffer for %d\n",i);
            return;
        }
        pi_buffer_init(&pi_buffers[i],PI_BUFFER_TYPE_L2, buffers[i]);
        pi_buffer_set_format(&pi_buffers[i], CAM_WIDTH, CAM_HEIGHT, 1, PI_BUFFER_FORMAT_GRAY);
    }

    pi_task_t capture_task = {0};
    pi_task_callback(&capture_task, capture_callback, NULL);
    pi_camera_capture_async(&camera, buffers[0], BUFFER_SIZE, &capture_task);
    printf("Allocated buffer\n");

    


    // ASYNC CAMERA CAPTURE
    printf("Async Camera Start...\n");
    uint32_t time_before = pi_time_get_us();
    pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);

    while (pi_time_get_us() - time_before < 1000000)
    {

        pi_yield();
        
    }
    pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);

    uint32_t time_after = pi_time_get_us();
    printf("Async Camera End...\n\n");

    float capture_time = (float)(time_after-time_before)/1000000;
    float FPS_async = (float)img_num_async/capture_time;
    printf("Async Capture: %d images\n",img_num_async);
    printf("Async Capture: %.3f FPS\n",FPS_async);
    printf("Val: %.6f\n",capture_time);


    pmsis_exit(0);

}

/* Program Entry. */
int main(void)
{
    pi_bsp_init();

    // Increase the FC freq to 250 MHz
    pi_freq_set(PI_FREQ_DOMAIN_FC, CLOCK_FREQ);
    __pi_pmu_voltage_set(PI_PMU_DOMAIN_FC, 1200); // Not sure on why set voltage?

    return pmsis_kickoff((void *)Cam_Example);
}


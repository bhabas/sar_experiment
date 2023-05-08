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

#define NUM_BUFFERS 1
#define RESOLUTION CAM_WIDTH*CAM_HEIGHT
#define BUFFER_SIZE CAM_WIDTH*CAM_HEIGHT*sizeof(uint8_t)

static void capture_callback1(void *arg);
static void capture_callback2(void *arg);

// CAMERA BUFFERS AND TASKS
static struct pi_device camera;

uint8_t *buffers[NUM_BUFFERS];
pi_buffer_t pi_buffers[NUM_BUFFERS];
int current_buffer = 0;

// PERFORMANCE MEASURING VARIABLES
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
    pi_camera_control(device, PI_CAMERA_CMD_START, 0);
    uint8_t set_value = 3;
    uint8_t reg_value;
    pi_camera_reg_set(device, IMG_ORIENTATION, &set_value);
    vTaskDelay(500);
    pi_camera_control(device, PI_CAMERA_CMD_STOP, 0);

    return 0;
}

static void capture_callback2(void *arg)
{
    printf("HERE\n");
}

static void capture_callback(void *arg)
{
    int buffer_index = (int)arg;
    printf("buffer index %d\n",buffer_index);
    printf("Img %d\n",img_num_async);    
    img_num_async++;

    // QUEUE THE NEXT BUFFER FOR CAPTURING
    current_buffer = 0;
    printf("here \n");    
    pi_task_t capture_task2;
    pi_task_callback(&capture_task2, capture_callback2, (void *)current_buffer);
    // pi_camera_capture_async(&camera, buffers[current_buffer], BUFFER_SIZE, &capture_task2);
}



void cam_example(void)
{
    printf("-- Starting Camera Test --\n");
    pi_perf_conf(1 << PI_PERF_CYCLES);

    
    // INITIALIZE CAMERA
    if (open_pi_camera_himax(&camera))
    {
        printf("Failed to open camera\n");
        return;
    }


    // INITIALIZE BUFFERS
    for (int i = 0; i < NUM_BUFFERS; i++)
    {
        buffers[i] = (uint8_t *)pmsis_l2_malloc(BUFFER_SIZE);
        if (buffers[i] == NULL)
        {
            printf("Failed to allocate memory for image buffer for %d\n",i);
            return ;
        }
        pi_buffer_init(&pi_buffers[i],PI_BUFFER_TYPE_L2, buffers[i]);
        pi_buffer_set_format(&pi_buffers[i], CAM_WIDTH, CAM_HEIGHT, 1, PI_BUFFER_FORMAT_GRAY);

        pi_task_t capture_task;
        pi_task_callback(&capture_task, capture_callback, (void *)i);
        printf("Allocated buffer %d\n",i);

        pi_camera_capture_async(&camera, buffers[i], BUFFER_SIZE, &capture_task);
        
    }

    // ASYNC CAMERA CAPTURE
    printf("Async Camera Start...\n");
    pi_perf_stop();
    pi_perf_reset();

    pi_perf_start();
    pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
    while (pi_perf_read(PI_PERF_CYCLES) < CLOCK_FREQ)
    {

        // printf("cycles %d\n",pi_perf_read(PI_PERF_CYCLES));
        pi_yield();
        
    }
    pi_perf_stop();
    pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);

    clock_cycles_async = pi_perf_read(PI_PERF_CYCLES);
    printf("Async Camera End...\n\n");

    float FPS_async = ((float)CLOCK_FREQ/(float)clock_cycles_async)*img_num_async;
    printf("Async Capture: %d cycles\n",clock_cycles_async);
    printf("Async Capture: %d images\n",img_num_async);
    printf("Async Capture: %.3f FPS\n",FPS_async);


    pmsis_exit(0);

}

/* Program Entry. */
int main(void)
{
    pi_bsp_init();

    // Increase the FC freq to 250 MHz
    pi_freq_set(PI_FREQ_DOMAIN_FC, CLOCK_FREQ);
    __pi_pmu_voltage_set(PI_PMU_DOMAIN_FC, 1200); // Not sure on why set voltage?

    return pmsis_kickoff((void *)cam_example);
}


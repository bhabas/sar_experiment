#include <stdio.h>
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
#define CAPTURE_NUM 2

// CAMERA BUFFERS AND TASKS
static struct pi_device camera;

static unsigned char *imgBuff1;
static volatile uint8_t done1 = 0;
static pi_buffer_t buffer1;
static pi_task_t task1;

static unsigned char *imgBuff2;
static volatile uint8_t done2 = 0;
static pi_buffer_t buffer2;
static pi_task_t task2;

volatile uint8_t img_num_sync = 0;
volatile uint8_t img_num_async = 0;

volatile uint32_t clock_cycles_sync = 0;
volatile uint32_t clock_cycles_async = 0;



// PERFORMANCE MEASURING VARIABLES



uint32_t resolution = CAM_WIDTH * CAM_HEIGHT;
uint32_t captureSize = CAM_WIDTH * CAM_HEIGHT * sizeof(unsigned char);



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

static void capture_done_cb1(void *arg)
{
    done1++;
}

static void capture_done_cb2(void *arg)
{
    done2++;

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


    // INITIALIZE BUFFER 1
    imgBuff1 = (unsigned char *)pmsis_l2_malloc(captureSize);
    if (imgBuff1 == NULL)
    {
        printf("Failed to allocate memory_1 for image \n");
        return;
    }
    pi_buffer_init(&buffer1, PI_BUFFER_TYPE_L2, imgBuff1);
    pi_buffer_set_format(&buffer1, CAM_WIDTH, CAM_HEIGHT, 1, PI_BUFFER_FORMAT_GRAY);

    // INITIALIZE BUFFER 2
    imgBuff2 = (unsigned char *)pmsis_l2_malloc(captureSize);
    if (imgBuff2 == NULL)
    {
        printf("Failed to allocate memory_2 for image \n");
        return;
    }
    pi_buffer_init(&buffer2, PI_BUFFER_TYPE_L2, imgBuff2);
    pi_buffer_set_format(&buffer2, CAM_WIDTH, CAM_HEIGHT, 1, PI_BUFFER_FORMAT_GRAY);

    


    // STANDARD CAMERA CAPTURE
    printf("Sync Camera Start...\n");
    pi_perf_stop();
    pi_perf_reset();

    pi_perf_start();
    pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
    while (pi_perf_read(PI_PERF_CYCLES) < CLOCK_FREQ)
    {
        pi_camera_capture(&camera, imgBuff1, resolution);
        img_num_sync++;
    }
    pi_perf_stop();
    pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);

    clock_cycles_sync = pi_perf_read(PI_PERF_CYCLES);
    printf("Sync Camera End...\n\n");



    // ASYNC CAMERA CAPTURE
    printf("Async Camera Start...\n");
    pi_perf_stop();
    pi_perf_reset();

    pi_perf_start();
    pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
    while (pi_perf_read(PI_PERF_CYCLES) < CLOCK_FREQ)
    {
        done1 = 0;
        pi_camera_capture_async(&camera, imgBuff1, resolution, pi_task_callback(&task1, capture_done_cb1, NULL));

        while (done1 == 0)
        {
            pi_yield();
        }
        
        img_num_async++;
    }
    pi_perf_stop();
    pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);

    clock_cycles_async = pi_perf_read(PI_PERF_CYCLES);
    printf("Async Camera End...\n\n");


    // PRINT CAPTURE TIMES AND FPS
    float FPS_sync = ((float)CLOCK_FREQ/(float)clock_cycles_sync)*img_num_sync;
    printf("Sync Capture: %d cycles\n",clock_cycles_sync);
    printf("Sync Capture: %d images\n",img_num_sync);
    printf("Sync Capture: %.3f FPS\n",FPS_sync);

    printf("\n");

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
    pi_pmu_voltage_set(PI_PMU_DOMAIN_FC, 1200); // Not sure on why set voltage?

    return pmsis_kickoff((void *)cam_example);
}


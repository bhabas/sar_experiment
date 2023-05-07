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
#define CAPTURE_NUM 5

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


// PERFORMANCE MEASURING VARIABLES
static uint32_t start_capture = 0;
static uint32_t captureTime = 0;
uint32_t capture_arr[CAPTURE_NUM] = {0};
uint32_t capture_count = 0;
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
    done1 = 1;
}

static void capture_done_cb2(void *arg)
{
    done2 = 1;

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


    // START PERFORMANCE CAPTURE
    printf("Camera Start...\n");
    pi_perf_stop();
    pi_perf_reset();


    while (capture_count < CAPTURE_NUM)
    {
        // RESET DONE FLAGS
        done1 = 0;
        done2 = 0;
        
        // INITIALIZE CAMERA CAPTURE TASKS


        // START CAMERA AND WAIT TILL IMAGE BUFFERS ARE FILLED
        pi_perf_start();
        pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
        // pi_camera_capture(&camera, imgBuff1, resolution);
        pi_camera_capture_async(&camera, imgBuff1, resolution, pi_task_callback(&task1, capture_done_cb1, NULL));
        // pi_camera_capture_async(&camera, imgBuff2, resolution, pi_task_callback(&task2, capture_done_cb2, NULL));
        while(done1 == 0)
        {
            pi_yield();
        }
        pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);
        pi_perf_stop();

        capture_arr[capture_count] = pi_perf_read(PI_PERF_CYCLES);
        pi_perf_reset();

        // RECORD CYCLES TO TAKE BOTH IMAGES
        vTaskDelay(60); // Needs delay for an unknown reason


        capture_count++;
    }
    
    

    printf("Camera End...\n");
    

    // PRINT CAPTURE TIMES AND FPS
    for (uint8_t i = 0; i < CAPTURE_NUM; i++)
    {
        float FPS = ((float)CLOCK_FREQ/(float)capture_arr[i]);
        printf("Capture Set %d = %d cycles\n",i,capture_arr[i]);
        printf("Capture Set %d = %.3f FPS\n",i,FPS);


    }
    // printf("Total Capture Time: %d\n",captureTime);
    // printf("FPS: %.3f\n",FPS);
   

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


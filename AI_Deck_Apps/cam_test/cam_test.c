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
#define IMAGE_NUM 15

static pi_task_t task1;
static unsigned char *imgBuff;
static struct pi_device camera;
static pi_buffer_t buffer;

// Performance menasuring variables
static uint32_t start_full = 0;
static uint32_t start_img = 0;
static uint32_t captureTime = 0;
uint32_t capture_arr[IMAGE_NUM] = {0};
uint32_t image = 0;

static volatile uint8_t done;

static int open_pi_camera_himax(struct pi_device *device)
{
    struct pi_himax_conf cam_conf;

    pi_himax_conf_init(&cam_conf);

    cam_conf.format = PI_CAMERA_QQVGA;
    cam_conf.skip_pads_config = 0;


    pi_open_from_conf(device, &cam_conf);
    if (pi_camera_open(device))
        return -1;

    // rotate image
    pi_camera_control(device, PI_CAMERA_CMD_START, 0);
    uint8_t set_value = 3;
    uint8_t reg_value;
    pi_camera_reg_set(device, IMG_ORIENTATION, &set_value);
    vTaskDelay(500);
    pi_camera_control(device, PI_CAMERA_CMD_STOP, 0);
    // pi_camera_control(device, PI_CAMERA_CMD_AEG_INIT, 0);

    return 0;
}

static void capture_done_cb(void *arg)
{
//   xEventGroupSetBits(evGroup, CAPTURE_DONE_BIT);
  done = 1;
}


void start_example(void)
{
    printf("-- Starting Camera Test --\n");
    pi_perf_conf(1 << PI_PERF_CYCLES);

    uint32_t imgSize = 0;
    uint32_t resolution = CAM_WIDTH * CAM_HEIGHT;
    uint32_t captureSize = resolution * sizeof(unsigned char);

    imgBuff = (unsigned char *)pmsis_l2_malloc(captureSize);
    if (imgBuff == NULL)
    {
        printf("Failed to allocate Memory for Image \n");
        return;
    }

    if (open_pi_camera_himax(&camera))
    {
        printf("Failed to open camera\n");
        return;
    }


    pi_buffer_init(&buffer, PI_BUFFER_TYPE_L2, imgBuff);
    pi_buffer_set_format(&buffer, CAM_WIDTH, CAM_HEIGHT, 1, PI_BUFFER_FORMAT_GRAY);
    pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);

    

    done = 0;
    printf("Camera Start...\n");
    pi_perf_start();
    start_full = pi_perf_read(PI_PERF_CYCLES);
    while (image < IMAGE_NUM)
    {
        start_img = pi_perf_read(PI_PERF_CYCLES);
        pi_camera_capture_async(&camera, imgBuff, resolution, pi_task_callback(&task1, capture_done_cb, NULL));
        pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
        while(!done){pi_yield();}
        capture_arr[image] = pi_perf_read(PI_PERF_CYCLES) - start_img;
        done = 0;
        pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);

        vTaskDelay(5);
        
        image++;

    }
    pi_perf_stop();
    printf("Camera End...\n");
    

    for (uint8_t i = 0; i < IMAGE_NUM; i++)
    {
        printf("Image %d = %d cycles\n",i,capture_arr[i]);

    }

    captureTime = pi_perf_read(PI_PERF_CYCLES) - start_full;
    printf("Capture_Time %d\n",captureTime);

    float FPS = ((float)CLOCK_FREQ/(float)captureTime)*IMAGE_NUM;
    printf("FPS: %.3f\n",FPS);
   

    pmsis_exit(0);

}

/* Program Entry. */
int main(void)
{
    pi_bsp_init();

    // Increase the FC freq to 250 MHz
    pi_freq_set(PI_FREQ_DOMAIN_FC, CLOCK_FREQ);
    pi_pmu_voltage_set(PI_PMU_DOMAIN_FC, 1200);

    return pmsis_kickoff((void *)start_example);
}


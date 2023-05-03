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

static unsigned char *imgBuff;
static struct pi_device camera;
static pi_buffer_t buffer;

// Performance menasuring variables
static uint32_t start = 0;
static uint32_t captureTime = 0;

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


void start_example(void)
{
    printf("-- Starting Camera Test --\n");
    pi_perf_conf(1 << PI_PERF_CYCLES);

    if (open_pi_camera_himax(&camera))
    {
        printf("Failed to open camera\n");
        return;
    }

    uint32_t imgSize = 0;
    uint32_t resolution = CAM_WIDTH * CAM_HEIGHT;
    uint32_t captureSize = resolution * sizeof(unsigned char);

    imgBuff = (unsigned char *)pmsis_l2_malloc(captureSize);
    if (imgBuff == NULL)
    {
        printf("Failed to allocate Memory for Image \n");
        return;
    }

    pi_buffer_init(&buffer, PI_BUFFER_TYPE_L2, imgBuff);
    pi_buffer_set_format(&buffer, CAM_WIDTH, CAM_HEIGHT, 1, PI_BUFFER_FORMAT_GRAY);
    pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);

    uint8_t i = 0;
    while (i < 5)
    {
        // pi_perf_start();
        // start = pi_perf_read(PI_PERF_CYCLES);
        uint32_t time_before = pi_time_get_us();
        pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
        pi_camera_capture(&camera,imgBuff,resolution);
        // captureTime = pi_perf_read(PI_PERF_CYCLES) - start;
        pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);

        // pi_time_wait_us(500);
        // pi_perf_stop();
        // pi_perf_reset();
        // printf("\n");
        vTaskDelay(5);
        i++;
    }
    printf("dsfsdf\n\n");
    
    

    


    


    pmsis_exit(0);

}

/* Program Entry. */
int main(void)
{
    pi_bsp_init();

    // Increase the FC freq to 250 MHz
    pi_freq_set(PI_FREQ_DOMAIN_FC, 250000000);
    pi_pmu_voltage_set(PI_PMU_DOMAIN_FC, 1200);

    return pmsis_kickoff((void *)start_example);
}


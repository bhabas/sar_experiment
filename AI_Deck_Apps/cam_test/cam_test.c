/* PMSIS includes */
#include "pmsis.h"

#include "bsp/bsp.h"
#include "bsp/camera.h"
#include "bsp/camera/himax.h"
#include "bsp/buffer.h"
#include "stdio.h"

#define IMG_ORIENTATION 0x0101
#define WIDTH 324
#define HEIGHT 244
#define BUFF_SIZE (WIDTH*HEIGHT)
PI_L2 unsigned char *buff;
static struct pi_device camera;

static int open_camera(struct pi_device *device)
{
    printf("Opening Himax camera\n");
    struct pi_himax_conf cam_conf;
    pi_himax_conf_init(&cam_conf);

    cam_conf.format = PI_CAMERA_QVGA;

    pi_open_from_conf(device, &cam_conf);
    if (pi_camera_open(device))
    {
        return -1;
    }
    pi_camera_control(device, PI_CAMERA_CMD_AEG_INIT,0);

    return 0;
}

void test_camera(void)
{
    printf("Entering main controller\n");
    pi_freq_set(PI_FREQ_DOMAIN_FC, 50000000);

    printf("Testing normal camera capture\n");

    // Open the Himax camera
    if (open_camera(&camera))
    {
        printf("Failed to open camera\n");
        pmsis_exit(-1);
    }

    // ROTATE CAMERA ORIENTATION
    uint8_t set_value=3;
    uint8_t reg_value;

    pi_camera_reg_set(&camera, IMG_ORIENTATION, &set_value);
    pi_camera_reg_get(&camera, IMG_ORIENTATION, &reg_value);
    printf("Img orientation %d\n",reg_value);

    buff = pmsis_l2_malloc(BUFF_SIZE);
    printf("Initialized buffers\n");

     // Start the camera
    pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
    pi_camera_capture(&camera, buff, BUFF_SIZE);

    // Stop the camera and immediately close it
    pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);
    pi_camera_close(&camera);

    printf("Val: %d\n",buff[0]);


    printf("Test success !\n");

    pmsis_exit(0);
}

/* Program Entry. */
int main(void)
{
    printf("\n\n\t *** PMSIS Camera Example Test ***\n\n");
    return pmsis_kickoff((void *) test_camera);
}


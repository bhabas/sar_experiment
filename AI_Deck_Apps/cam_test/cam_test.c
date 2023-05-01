/* PMSIS includes */
#include "pmsis.h"

#include "bsp/bsp.h"
#include "bsp/camera.h"
#include "bsp/camera/himax.h"
#include "bsp/buffer.h"
#include "stdio.h"

#define IMG_ORIENTATION 0x0101
#define CAM_WIDTH 324
#define CAM_HEIGHT 244

static unsigned char *imgBuff;
static struct pi_device camera;
static pi_buffer_t buffer;

static int open_pi_camera_himax(struct pi_device *device)
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

    // ROTATE IMAGE
    pi_camera_control(device, PI_CAMERA_CMD_START, 0);
    uint8_t set_value = 3;
    uint8_t reg_value;
    pi_camera_reg_set(device, IMG_ORIENTATION, &set_value);
    pi_time_wait_us(1000000);
    pi_camera_reg_get(device, IMG_ORIENTATION, &reg_value);

    if (set_value != reg_value)
    {
        printf( "Failed to rotate camera image\n");
        return -1;
    }

    pi_camera_control(device, PI_CAMERA_CMD_STOP, 0);

    // TURN ON/OFF AUTO EXPOSURE GAIN (NOT SURE IF ON?)
    pi_camera_control(device, PI_CAMERA_CMD_AEG_INIT, 0);
    printf("Camera opened successfully\n");


    return 0;
}

void test_camera(void)
{
    printf("\n\n\t *** PMSIS Camera Example Test ***\n\n");

    // SET UP IMAGE BUFFER AND CAMERA CAPTURE SETTINGS
    uint32_t resolution = CAM_WIDTH*CAM_HEIGHT;
    uint32_t captureSize = resolution*sizeof(unsigned char);
    imgBuff = (unsigned char *)pmsis_l2_malloc(captureSize);
    if (imgBuff == NULL)
    {
        printf( "Failed to allocate Memory for Image \n");
        return;
    }

    // CONFIGURE CAMERA
    if (open_pi_camera_himax(&camera))
    {
        printf( "Failed to open camera\n");
        return;
    }
    
    pi_buffer_init(&buffer, PI_BUFFER_TYPE_L2, imgBuff);
    pi_buffer_set_format(&buffer, CAM_WIDTH, CAM_HEIGHT, 1, PI_BUFFER_FORMAT_GRAY);

    // CAPTURE CAMERA IMAGE
    pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
    pi_camera_capture(&camera, imgBuff, resolution);
    pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);



    //  // Start the camera
    // pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
    // pi_camera_capture(&camera, buff, BUFF_SIZE);

    // // Stop the camera and immediately close it
    // pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);
    // pi_camera_close(&camera);

    printf("Print Image:\n\n");

    for (size_t i = 324*50; i < 324*52; i++)
    {
        printf("%d ",imgBuff[i]);
    }
    printf("\n\nPrint Image:\n");

    


    // printf("Test success !\n");

    pmsis_exit(0);
}

/* Program Entry. */
int main(void)
{
    pi_bsp_init();


    // Increase the FC freq to 250 MHz
    pi_freq_set(PI_FREQ_DOMAIN_FC, 250000000);
    __pi_pmu_voltage_set(PI_PMU_DOMAIN_FC, 1200);

    return pmsis_kickoff((void *) test_camera);
}


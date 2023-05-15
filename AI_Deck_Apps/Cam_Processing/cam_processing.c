#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>


/* PMSIS includes */
#include "pmsis.h"

/* PMSIS BSP includes. */
#include "bsp/bsp.h"
#include "bsp/camera.h"


#define IMG_ORIENTATION 0x0101
#define CAM_WIDTH 162
#define CAM_HEIGHT 122
#define CLOCK_FREQ 250*1000000 // [MHz]

#define NUM_BUFFERS 2
#define RESOLUTION CAM_WIDTH*CAM_HEIGHT
#define BUFFER_SIZE CAM_WIDTH*CAM_HEIGHT*sizeof(uint8_t)

// CLUSTER SETUP
struct pi_device cl_dev;
struct pi_cluster_task cl_task;


// CAMERA BUFFERS AND TASKS
static struct pi_device camera;

static pi_task_t task;
static uint8_t* img_buffers[NUM_BUFFERS];

static volatile int current_idx = 0;
static volatile int next_idx = 0;


// PERFORMANCE MEASURING VARIABLES
volatile uint8_t buffer_index = 0;
volatile uint8_t img_num_async = 0;

uint32_t value[8] = {0};

/* Task executed by cluster cores. */
void cluster_helloworld(void *arg)
{
    uint32_t core_id = pi_core_id(), cluster_id = pi_cluster_id();
    value[pi_cluster_id()] = pi_cluster_id();
    pi_cl_team_barrier();
}

/* Cluster main entry, executed by core 0. */
void cluster_delegate(void *arg)
{
    // Cluster master core entry
    pi_cl_team_fork(pi_cl_cluster_nb_cores(), cluster_helloworld, arg);
    // Cluster master core exit

}

static int32_t open_cluster(struct pi_device *device)
{
    struct pi_cluster_conf cl_conf;

    // CLUSTER CONFIG
    pi_cluster_conf_init(&cl_conf);
    cl_conf.id = 0;
    
    // OPEN CLUSTER
    pi_open_from_conf(&cl_dev, &cl_conf);
    if (pi_cluster_open(device))
        return -1;


    return 0;
}


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
    // pi_camera_control(device, PI_CAMERA_CMD_AEG_INIT, 0);
    pi_time_wait_us(1000000); // Give time for camera to adjust exposure


    return 0;
}

static void process_image(uint8_t* image_buffer)
{

    // uint32_t time_before = pi_time_get_us();
    // uint32_t sum = 0;
    // for (int i = 0; i < CAM_HEIGHT*CAM_WIDTH; i++)
    // {
    //     sum += image_buffer[i];
    // }
    // uint32_t time_after = pi_time_get_us();

    // printf("Pixel Sum: %d\n",time_after-time_before);    
    pi_time_wait_us(9000);
    // img_num_async++;
}


void test_camera_double_buffer(void)
{
    printf("-- Starting Camera Test --\n");
    
    // ALLOCATE MEMORY FOR IMAGES
    for (int i = 0; i < NUM_BUFFERS; i++)
    {
        img_buffers[i] = (uint8_t *)pmsis_l2_malloc(BUFFER_SIZE);
        if (img_buffers[i] == NULL)
        {
            printf("Failed to allocate memory for image\n");
            pmsis_exit(-1);
        }
        
    }

    // INITIALIZE CAMERA
    if (open_pi_camera_himax(&camera))
    {
        printf("Failed to open camera\n");
        pmsis_exit(-1);
    }

    // INITIALIZE CLUSTER
    if (open_cluster(&cl_dev))
    {
        printf("Failed to open cluster\n");
        pmsis_exit(-1);
    }

    // MAKE SURE CAMEAR IS NOT SENDING DATA
    pi_camera_control(&camera, PI_CAMERA_CMD_STOP,0);

    // CAPTURE FIRST IMAGE BEFORE LAUNCHING DOUBLE BUFFER LOOP
    pi_task_block(&task);
    pi_camera_capture_async(&camera, img_buffers[current_idx],CAM_WIDTH*CAM_HEIGHT, &task);
    pi_camera_control(&camera,PI_CAMERA_CMD_START,0);
    pi_task_wait_on(&task);

    
    pi_cluster_send_task_to_cl(&cl_dev, pi_cluster_task(&cl_task, cluster_delegate, NULL));


    // CAPTURE IMAGES
    printf("Main Loop start\n");

    uint32_t time_before = pi_time_get_us();
    while (pi_time_get_us() - time_before < 1000000)
    {

        current_idx = next_idx;
        next_idx ^= 1;

        // LAUNCH CAPTURE OF NEXT IMAGE
        pi_task_block(&task);
        pi_camera_capture_async(&camera, img_buffers[next_idx],CAM_WIDTH*CAM_HEIGHT, &task);

        // PROCESS THE CURRENT IMAGE
        // process_image(img_buffers[current_idx]);
        // pi_cluster_send_task_to_cl(&cl_dev, pi_cluster_task(&cl_task, cluster_delegate, NULL));
        pi_task_wait_on(&task);
        img_num_async++;

        
        
    }
    uint32_t time_after = pi_time_get_us();
    float capture_time = (float)(time_after-time_before)/1000000;
    float FPS_async = (float)img_num_async/capture_time;
    printf("Capture FPS:        %.6f FPS\n",FPS_async);
    printf("Capture Duration:   %.6f ms\n",1/FPS_async);
    printf("Capture Count:      %d images\n",img_num_async);
    printf("Capture Time:       %.6f s\n",capture_time);
    printf("Exiting... \n");
    pi_cluster_close(&cl_dev);


    pmsis_exit(0);

}

/* Program Entry. */
int main(void)
{
    pi_bsp_init();

    // Increase the FC freq to 250 MHz
    pi_freq_set(PI_FREQ_DOMAIN_FC, CLOCK_FREQ);
    __pi_pmu_voltage_set(PI_PMU_DOMAIN_FC, 1200); // Not sure on why set voltage?

    return pmsis_kickoff((void *)test_camera_double_buffer);
}


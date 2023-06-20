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

// CAMERA BUFFERS AND TASKS
static struct pi_device camera;

uint8_t *buffers[NUM_BUFFERS];
pi_task_t capture_task[NUM_BUFFERS];

// PERFORMANCE MEASURING VARIABLES
volatile uint8_t buffer_index = 0;
volatile uint8_t img_num_async = 0;
volatile uint32_t clock_cycles_async = 0;



typedef struct{
    uint32_t task_index;
    uint32_t buffer_index;
    uint32_t time_complete;
} Index;

Index test_index[NUM_BUFFERS];



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
    // UPDATE IMAGE COUNT
    img_num_async++;

    // LOAD TASK STRUCT
    Index *index_ptr = (Index *)arg;

    // GET CURRENT BUFFER INDEX
    uint32_t current_buffer = index_ptr->buffer_index;
    
    // UPDATE BUFFER INDEX IN STRUCT
    index_ptr->buffer_index = (index_ptr->buffer_index + 1) % NUM_BUFFERS;
    uint32_t next_buffer = index_ptr->buffer_index;

    index_ptr->time_complete = pi_time_get_us();


    // printf("Task:\t%d\n",index_ptr->task_index);
    // printf("Buffer:\t%d\n",current_buffer);

    // PASS UPDATED STRUCT TO NEXT CALL
    pi_task_callback(&capture_task[index_ptr->task_index], capture_callback, arg);
    pi_camera_capture_async(&camera, buffers[next_buffer], BUFFER_SIZE, &capture_task[index_ptr->task_index]);

        
}



void Cam_Example(void)
{
    printf("-- Starting Camera Test --\n");
    
    // INITIALIZE CAMERA
    if (open_pi_camera_himax(&camera))
    {
        printf("Failed to open camera\n");
        return;
    }

    for (uint32_t i = 0; i < NUM_BUFFERS; i++)
    {
        // INITIALIZE BUFFERS
        buffers[i] = (uint8_t *)pmsis_l2_malloc(BUFFER_SIZE);
        if (buffers[i] == NULL)
        {
            printf("Failed to allocate memory for image buffer for %d\n",i);
            return;
        }
        test_index[i].task_index = i;
        test_index[i].buffer_index = i;

        
    }

    for (uint32_t i = 0; i < NUM_BUFFERS; i++)
    {
        // uint32_t i = 1;
        pi_task_callback(&capture_task[i], capture_callback, (void*) &test_index[i]);
        pi_camera_capture_async(&camera, buffers[i], BUFFER_SIZE, &capture_task[i]);
    }
    
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
    printf("Async Capture: %.6f\n",capture_time);


    for (uint32_t i = 0; i < NUM_BUFFERS; i++)
    {
        printf("Time:\t%d\n",test_index[i].time_complete);
    }

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

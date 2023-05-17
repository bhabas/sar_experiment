#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>


/* PMSIS includes */
#include "pmsis.h"

/* PMSIS BSP includes. */
#include "bsp/bsp.h"
#include "bsp/camera.h"

#include "processing.h"


#define IMG_ORIENTATION 0x0101
#define CAM_WIDTH 8
#define CAM_HEIGHT 10
#define CLOCK_FREQ 250*1000000 // [MHz]

#define NUM_BUFFERS 2
#define RESOLUTION CAM_WIDTH*CAM_HEIGHT
#define BUFFER_SIZE CAM_WIDTH*CAM_HEIGHT*sizeof(uint8_t)

// CLUSTER SETUP
struct pi_device cl_dev;
// struct pi_cluster_task cl_task;


// CAMERA BUFFERS AND TASKS
static struct pi_device camera;

static pi_task_t task;
uint8_t* ImgBuff[NUM_BUFFERS];

volatile int current_idx = 0;
volatile int next_idx = 0;


int32_t G_up[CAM_WIDTH*CAM_HEIGHT];
int32_t G_vp[CAM_WIDTH*CAM_HEIGHT];
int32_t G_rp[CAM_WIDTH*CAM_HEIGHT];
int32_t G_tp[CAM_WIDTH*CAM_HEIGHT];


int32_t O_up = CAM_HEIGHT/2;  // Pixel Y_offset [pixels]
int32_t O_vp = CAM_WIDTH/2;   // Pixel X_offset [pixels]



// PERFORMANCE MEASURING VARIABLES
volatile uint8_t buffer_index = 0;
volatile uint8_t img_num_async = 0;

typedef struct cluster_stuff{
    uint8_t* Cur_img_buff;
    uint8_t* Prev_img_buff;
    uint8_t rows_per_core;
} cluster_stuff_t;

int32_t Ku[9] = {-1, 0, 1,
                 -2, 0, 2,
                 -1, 0, 1};

int32_t Kv[9] = {-1,-2, 1,
                  0, 0, 0,
                  1, 2, 1};


void convolve2D(uint8_t* img, int32_t* result, int32_t* kernel, int startRow, int endRow, int stride);

/* Task executed by cluster cores. */
void cluster_processing(void *arg)
{
    uint32_t core_id = pi_core_id();
    cluster_stuff_t* test_struct = (cluster_stuff_t *)arg;
    int start_row = core_id * test_struct->rows_per_core + 1;
    int end_row = start_row + test_struct->rows_per_core - 1;

    // TO-DO: SEP CONVOLVE
    // DOT PRODUCT
    // VALIDATE G_RP

    convolve2D(test_struct->Cur_img_buff,G_up,Ku,start_row,end_row,1);
    convolve2D(test_struct->Cur_img_buff,G_vp,Kv,start_row,end_row,1);

    for (int32_t v_p = start_row; v_p <= end_row; v_p += 1)
    {
        for (int32_t u_p = 1; u_p < CAM_WIDTH -1; u_p += 1)
        {
            int32_t curPos = v_p* CAM_WIDTH + u_p;
            G_rp[curPos] = (2*(u_p - O_up) + 1)*G_up[curPos] + (2*(v_p - O_vp) + 1)*G_vp[curPos];
        }
    }


    for (int i = 0; i < CAM_WIDTH; i++)
    {
        G_tp[core_id*CAM_WIDTH + i] = test_struct->Cur_img_buff[core_id*CAM_WIDTH + i] - test_struct->Prev_img_buff[core_id*CAM_WIDTH + i];
    }

    pi_cl_team_barrier();

}

void convolve2D(uint8_t* img, int32_t* result, int32_t* kernel, int startRow, int endRow, int stride)
{
    for (int32_t v_p = startRow; v_p <= endRow; v_p += stride)
    {
        for (int32_t u_p = 1; u_p < CAM_WIDTH -1; u_p += stride)
        {
            int32_t sum = 0;
            for (int32_t i = 0; i <= 2; i++)
            {
                for (int32_t j = 0; j <= 2; j++)
                {

                    // Handle image boundaries
                    if (v_p + i-1 < 0 || v_p + i-1 >= CAM_HEIGHT)
                    {
                        continue;
                    }

                    int32_t curPos = (v_p + i-1) * CAM_WIDTH + (u_p + j-1);
                    int32_t kerPos = i*3 + j;
                    sum += img[curPos] * kernel[kerPos];            
                }
            }
            result[v_p*CAM_WIDTH + u_p] = sum;
        }
    }

}

/* Cluster main entry, executed by core 0. */
void cluster_delegate(void *arg)
{
    cluster_stuff_t* test_struct = (cluster_stuff_t *)arg;

    pi_cl_team_fork(pi_cl_cluster_nb_cores(), cluster_processing, arg);


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




static void process_images(uint8_t* Cur_img_buff, uint8_t* Prev_img_buff)
{

    struct cluster_stuff test_struct;
    test_struct.rows_per_core = (CAM_HEIGHT - 2)/pi_cl_cluster_nb_cores();
    test_struct.Cur_img_buff = Cur_img_buff;
    test_struct.Prev_img_buff = Prev_img_buff;

    struct pi_cluster_task cl_task;
    pi_cluster_task(&cl_task, cluster_delegate, &test_struct);

    uint32_t time_before = pi_time_get_us();
    pi_cluster_send_task(&cl_dev,&cl_task);
    uint32_t time_after = pi_time_get_us();

    printf("Calc Time: %d us\n",(time_after-time_before));   

    if (CAM_WIDTH < 30)
    {
        print_image_int32(G_rp,CAM_WIDTH,CAM_HEIGHT);
    }
    
    

}



void Cam_Processing(void)
{
    printf("-- Starting Camera Test --\n");
    
    // ALLOCATE MEMORY FOR IMAGES
    for (int i = 0; i < NUM_BUFFERS; i++)
    {
        ImgBuff[i] = (uint8_t *)pmsis_l2_malloc(BUFFER_SIZE);
        if (ImgBuff[i] == NULL)
        {
            printf("Failed to allocate memory for image\n");
            pmsis_exit(-1);
        }
        
    }

    // INITIALIZE CLUSTER
    if (open_cluster(&cl_dev))
    {
        printf("Failed to open cluster\n");
        pmsis_exit(-1);
    }


    // CAPTURE FIRST IMAGE (BUFFER 1)
    for (int i = 0; i < CAM_HEIGHT; i++)
    {
       for (int j = 0; j < CAM_WIDTH; j++)
        {
            ImgBuff[0][j + CAM_WIDTH*i] = j;
        }
    }

    // CAPTURE NEXT IMAGE (BUFFER 2)
    for (int i = 0; i < CAM_HEIGHT; i++)
    {
       for (int j = 0; j < CAM_WIDTH; j++)
        {
            ImgBuff[1][j + CAM_WIDTH*i] = j+1;
        }
    }
    

    // PROCESS IMAGES
    process_images(ImgBuff[1],ImgBuff[0]);


    // // PRINT IMAGE
    // printf("Prev Image:\n");
    // print_image_uint8(ImgBuff[0],CAM_WIDTH,CAM_HEIGHT);


    // printf("Curr Image:\n");
    // print_image_uint8(ImgBuff[1],CAM_WIDTH,CAM_HEIGHT);

    // printf("G_tp:\n");
    // print_image_uint32(G_tp,CAM_WIDTH,CAM_HEIGHT);

    




    pmsis_exit(0);

}

/* Program Entry. */
int main(void)
{
    pi_bsp_init();

    // Increase the FC freq to 250 MHz
    pi_freq_set(PI_FREQ_DOMAIN_FC, CLOCK_FREQ);
    __pi_pmu_voltage_set(PI_PMU_DOMAIN_FC, 1200); // Not sure on why set voltage?

    return pmsis_kickoff((void *)Cam_Processing);
}


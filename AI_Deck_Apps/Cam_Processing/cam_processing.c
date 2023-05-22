#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "test.h"
#include "params.h"




/* PMSIS includes */
#include "pmsis.h"

/* PMSIS BSP includes. */
#include "bsp/bsp.h"
#include "bsp/camera.h"




// CLUSTER SETUP
struct pi_device cl_dev;
// struct pi_cluster_task cl_task;
uint32_t time_before = 0;
uint32_t time_after = 0;

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
    uint8_t rows_per_core;
    uint8_t* Cur_img_buff;
    uint8_t* Prev_img_buff;
    int32_t* Vec1;
    int32_t* Vec2;
    int32_t* target_sum;
    int32_t temp_array[8];
} cluster_stuff_t;

int32_t G_up_G_up = 0;


int32_t Ku[9] = {-1, 0, 1,
                 -2, 0, 2,
                 -1, 0, 1};
int32_t Ku_v[3] = { 1, 2, 1};
int32_t Ku_h[3] = {-1, 0, 1};

int32_t Kv[9] = {-1,-2,-1,
                  0, 0, 0,
                  1, 2, 1};

int32_t Kv_v[3] = {-1, 0, 1};
int32_t Kv_h[3] = { 1, 2, 1};




/* Task executed by cluster cores. */
void cl_GradCalcs(void *arg)
{
    uint32_t core_id = pi_core_id();
    cluster_stuff_t* test_struct = (cluster_stuff_t *)arg;
    int start_row = core_id * test_struct->rows_per_core + 1;
    int end_row = start_row + test_struct->rows_per_core - 1;

    // if (core_id == 7)   
    // {
    //     printf("Core: %d \t Start_Row: %d \t End_Row: %d \t Num_Rows: %d\n",core_id,start_row,end_row,test_struct->rows_per_core);
    //     // convolve2DSeparable(test_struct->Cur_img_buff, G_up, Ku_v, Ku_h, 1, CAM_HEIGHT-2);



    // }
    convolve2D(test_struct->Cur_img_buff,G_up,Ku,start_row,end_row);
    convolve2D(test_struct->Cur_img_buff,G_vp,Ku,start_row,end_row);
    temporalGrad(test_struct->Cur_img_buff,test_struct->Prev_img_buff,G_tp,start_row,end_row);
    radialGrad(test_struct->Cur_img_buff,G_rp,G_up,G_vp,start_row,end_row);


    // convolve2DSeparable(test_struct->Cur_img_buff, G_vp, Kv_v, Kv_h, start_row, end_row);
    pi_cl_team_barrier();

}

void printVal(int32_t val)
{
    printf("Val: %d\n",val);           

}



/* Cluster main entry, executed by core 0. */
void delegate_GradCalcs(void *arg)
{
    cluster_stuff_t* test_struct = (cluster_stuff_t *)arg;
    pi_cl_team_fork(pi_cl_cluster_nb_cores(), cl_GradCalcs, arg);
}

void cl_DotProducts(void *arg)
{
    uint32_t core_id = pi_core_id();
    cluster_stuff_t* test_struct = (cluster_stuff_t *)arg;
    int start_row = core_id * test_struct->rows_per_core + 1;
    int end_row = start_row + test_struct->rows_per_core - 1;

    int32_t sum = 0;
    for (int i = start_row; i <= end_row; i++)
    {
        for (int j = 1; j < CAM_WIDTH-1; j++)
        {
            sum += test_struct->Vec1[CAM_WIDTH*i + j] * test_struct->Vec2[CAM_WIDTH*i + j];
        }
    }
    
    test_struct->temp_array[core_id] = sum;
    pi_cl_team_barrier();

    if (core_id == 0)
    {
        int32_t sum = 0;
        for (int i = 0; i < pi_cl_cluster_nb_cores(); i++)
        {
            sum += test_struct->temp_array[i];
        }
        *(test_struct->target_sum) = sum;
    }
}

    

void delegate_DotProducs(void *arg)
{
    cluster_stuff_t* test_struct = (cluster_stuff_t *)arg;

    for (int i = 0; i < 12; i++)
    {
        // dot(G_vp,G_vp)
        test_struct->Vec1 = G_up;
        test_struct->Vec2 = G_up;
        test_struct->target_sum = &G_up_G_up;
    }
    
    pi_cl_team_fork(pi_cl_cluster_nb_cores(), cl_DotProducts, test_struct);

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
    pi_cluster_task(&cl_task, delegate_GradCalcs, &test_struct);

    printf("Start Processing... \n");   
    time_before = pi_time_get_us();
    pi_cluster_send_task(&cl_dev,&cl_task);
    // temporalGrad(test_struct.Cur_img_buff,test_struct.Prev_img_buff,G_tp,1,CAM_HEIGHT-2);
    // convolve2DSeparable(test_struct.Cur_img_buff, G_up, Ku_v, Ku_h, 1, CAM_HEIGHT-2);
    
    // convolve2DSeparable(test_struct.Cur_img_buff, G_up, Ku_v, Ku_h, 1,3);
    // printf("asdfas\n");
    // convolve2D(test_struct.Cur_img_buff,G_up,Ku,1,3);
    // convolve2DSeparable(test_struct.Cur_img_buff, G_vp, Kv_v, Kv_h, 1,CAM_HEIGHT-2);
    // radialGrad(test_struct.Cur_img_buff,G_rp,G_up,G_vp,1,CAM_HEIGHT-2);
    


    // #ifdef DEBUG
    // print_image_int32(G_up,CAM_WIDTH,CAM_HEIGHT);
    // print_image_int32(G_vp,CAM_WIDTH,CAM_HEIGHT);
    // print_image_int32(G_rp,CAM_WIDTH,CAM_HEIGHT);
    // print_image_int32(G_tp,CAM_WIDTH,CAM_HEIGHT);
    // #endif
    // print_image_int32(G_up,CAM_WIDTH,CAM_HEIGHT);

    // pi_cluster_task(&cl_task, delegate_DotProducs, &test_struct);
    // pi_cluster_send_task(&cl_dev,&cl_task);
    // printVal(G_up_G_up);
    time_after = pi_time_get_us();
    printf("Calc Time: %d us\n",(time_after-time_before));   



    // int32_t G_vp_G_vp = dotProduct(G_vp,G_vp,CAM_WIDTH*CAM_HEIGHT);
    // int32_t G_vp_G_up = dotProduct(G_vp,G_up,CAM_WIDTH*CAM_HEIGHT);
    // int32_t G_vp_G_rp = dotProduct(G_vp,G_rp,CAM_WIDTH*CAM_HEIGHT);
    // int32_t G_up_G_vp = dotProduct(G_up,G_vp,CAM_WIDTH*CAM_HEIGHT);
    // int32_t G_up_G_up = dotProduct(G_up,G_up,CAM_WIDTH*CAM_HEIGHT);
    // int32_t G_up_G_rp = dotProduct(G_up,G_rp,CAM_WIDTH*CAM_HEIGHT);
    // int32_t G_rp_G_vp = dotProduct(G_rp,G_vp,CAM_WIDTH*CAM_HEIGHT);
    // int32_t G_rp_G_up = dotProduct(G_rp,G_up,CAM_WIDTH*CAM_HEIGHT);
    // int32_t G_rp_G_rp = dotProduct(G_rp,G_rp,CAM_WIDTH*CAM_HEIGHT);

    // int32_t G_tp_G_vp = dotProduct(G_tp,G_vp,CAM_WIDTH*CAM_HEIGHT);
    // int32_t G_tp_G_up = dotProduct(G_tp,G_up,CAM_WIDTH*CAM_HEIGHT);
    // int32_t G_tp_G_rp = dotProduct(G_tp,G_rp,CAM_WIDTH*CAM_HEIGHT);
    // int32_t delta_t = 500;



    


    // printVal(G_vp_G_vp);
    // printVal(G_vp_G_up);
    // printVal(G_vp_G_rp);
    // printVal(G_up_G_vp);
    // printVal(G_up_G_up);
    // printVal(G_up_G_rp);
    // printVal(G_rp_G_vp);
    // printVal(G_rp_G_up);
    // printVal(G_rp_G_rp);
    // printf("\n\n");
    // printVal(G_tp_G_vp);
    // printVal(G_tp_G_up);
    // printVal(G_tp_G_rp);


    printf("End Processing... \n");   
    printf("Calc Time: %d us\n",(time_after-time_before));   
    // Need ~30,000 us calc

    // if (CAM_WIDTH < 30)
    // {
    //     print_image_int32(G_up,CAM_WIDTH,CAM_HEIGHT);
    // }
    
    

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

    // uint8_t img_cur[64] = {
    //     5,8,4,6,1,8,7,0,
    //     0,0,0,5,0,0,4,2,
    //     2,4,8,8,3,1,0,1,
    //     0,6,0,8,2,9,8,5,
    //     7,1,9,6,1,5,5,3,
    //     8,2,0,3,1,3,8,1,
    //     3,0,8,8,0,7,6,1,
    //     8,0,8,1,9,9,3,5,
    //     };
    
    // uint8_t img_prev[64] = {
    //     9,2,2,2,9,7,6,8,
    //     8,1,3,8,2,2,2,9,
    //     2,6,4,4,1,5,8,9,
    //     2,6,1,0,5,3,3,4,
    //     8,5,4,2,9,3,9,8,
    //     8,2,9,3,0,7,3,2,
    //     0,4,3,3,8,0,4,6,
    //     1,0,8,7,6,8,5,7,
    //     };
    
    // // PRINT IMAGE
    // printf("Prev Image:\n");
    // print_image_uint8(img_prev,CAM_WIDTH,CAM_HEIGHT);

    printf("Curr Image:\n");
    // print_image_uint8(ImgBuff[1],CAM_WIDTH,6);

    // // PROCESS IMAGES
    process_images(ImgBuff[1],ImgBuff[0]);


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


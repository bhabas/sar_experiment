#include "Cam_Processing.h"





/* Cluster main entry, executed by core 0. */
void delegate_GradCalcs(void *arg)
{
    ClusterImageData_t* CL_ImageData = (ClusterImageData_t *)arg;
    // int32_t* temp = (int32_t*) pi_cl_l1_malloc(&CL_device,CAM_WIDTH * sizeof(int32_t));
    
    pi_cl_team_fork(pi_cl_cluster_nb_cores(), CL_GradCalcs, arg);
}


/* Task executed by cluster cores. */
void CL_GradCalcs(void *arg)
{
    uint32_t core_id = pi_core_id();
    ClusterImageData_t* CL_ImageData = (ClusterImageData_t *)arg;
    int32_t start_row = core_id * CL_ImageData->Rows_Per_Core + 1;
    int32_t end_row = start_row + CL_ImageData->Rows_Per_Core - 1;
    int32_t stride = CL_ImageData->stride;


    convolve2D(CL_ImageData->Cur_img_buff,G_up,Ku,start_row,end_row,stride);
    convolve2D(CL_ImageData->Cur_img_buff,G_vp,Kv,start_row,end_row,stride);
    radialGrad(CL_ImageData->Cur_img_buff,G_rp,G_up,G_vp,start_row,end_row,stride);
    temporalGrad(CL_ImageData->Cur_img_buff,CL_ImageData->Prev_img_buff,G_tp,start_row,end_row,stride);

    pi_cl_team_barrier();

}

  
/**
 * @brief Calc all the necessary dot products from image gradients
 * | G_tp G_vp |   | G_vp G_vp  G_up G_vp G_rp G_vp | | Theta_x |
 * | G_tp G_up | = | G_vp G_up  G_up G_up G_rp G_up | | Theta_y |
 * | G_tp G_rp |   | G_vp G_rp  G_up G_rp G_rp G_rp | | Theta_z |
 * 
 * @param arg 
 */
void delegate_DotProducts(void *arg)
{
    ClusterImageData_t* CL_ImageData = (ClusterImageData_t *)arg;

    // G_vp*G_vp, G_vp*G_up, G_vp*G_rp, G_vp*G_tp, G_up*G_up, G_up*G_rp, G_up*G_tp, G_rp*G_rp, G_rp*G_tp

    int32_t* arrays[4] = {G_vp, G_up, G_rp, G_tp};
    uint8_t result_index = 0;
    uint8_t array_len = 4;

    for (int i = 0; i < array_len; i++)
    {
        for (int j = i; j < array_len; j++)
        {
            // Ensure there is enough space in the results array
            if (result_index > 9)
            {
                printf("Results array is full, cannot compute more dot products.\n");
                return;
            }

            // Avoid duplicate pairings and exclude dot(G_tp, G_tp)
            if ((i != j && i > j) || (i == j && i == 3))
            {
                continue;
            }

            CL_ImageData->DP_Vec1 = arrays[i];
            CL_ImageData->DP_Vec2 = arrays[j];
            CL_ImageData->DP_Sum = &(CL_ImageData->UART_array[result_index]);
            pi_cl_team_fork(pi_cl_cluster_nb_cores(), CL_DotProduct, CL_ImageData);

            // printf("Dot product of array %d and %d: %f\n", i, j, CL_ImageData->UART_array[result_index]);
            result_index++;
        }
        
    }

    // INCLUDE TIME BETWEEN IMAGES
    CL_ImageData->UART_array[result_index] = 100; // Delta_t
    
    // SEND CALC DATA TO CRAZYFLIE FOR FINAL COMPUTATION
    /* Do UART1 Stuff*/
}

void CL_DotProduct(void *arg)
{
    // CALC LIMITS FOR CORE
    ClusterImageData_t* CL_ImageData = (ClusterImageData_t *)arg;
    uint32_t core_id = pi_core_id();
    int32_t start_row = core_id * CL_ImageData->Rows_Per_Core + 1;
    int32_t end_row = start_row + CL_ImageData->Rows_Per_Core - 1;


    // CALC DOT PRODUCT INSIDE CORE LIMITS
    int32_t sum = 0;
    for (int32_t i = start_row; i <= end_row; i++)
    {
        for (int32_t j = 1; j < CAM_WIDTH-1; j++)
        {
            sum += CL_ImageData->DP_Vec1[CAM_WIDTH*i + j] * CL_ImageData->DP_Vec2[CAM_WIDTH*i + j];
        }
    }
    CL_ImageData->DP_Sum_array[core_id] = sum;
    pi_cl_team_barrier(); // Wait for all cores to finish


    // COMBINE RESULTS TO SINGLE DOT PRODUCT VALUE
    if (core_id == 0)
    {
        int32_t sum = 0;
        for (int i = 0; i < pi_cl_cluster_nb_cores(); i++)
        {
            sum += CL_ImageData->DP_Sum_array[i];
        }
        *(CL_ImageData->DP_Sum) = sum;
    }
}

  







void process_images(uint8_t* Cur_img_buff, uint8_t* Prev_img_buff)
{

    struct ClusterImageData CL_ImageData;
    CL_ImageData.Rows_Per_Core = (CAM_HEIGHT - 2)/NUM_CORES;
    CL_ImageData.Cur_img_buff = Cur_img_buff;
    CL_ImageData.Prev_img_buff = Prev_img_buff;
    CL_ImageData.stride = 2;


    struct pi_cluster_task CL_Grad_task;
    pi_cluster_task(&CL_Grad_task, delegate_GradCalcs, &CL_ImageData);
    pi_cluster_send_task(&CL_device,&CL_Grad_task);


    struct pi_cluster_task CL_DotProducts_task;
    pi_cluster_task(&CL_DotProducts_task, delegate_DotProducts, &CL_ImageData);
    pi_cluster_send_task(&CL_device,&CL_DotProducts_task);

}

void System_Init(void)
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

    // INITIALIZE CAMERA
    if (open_pi_camera_himax(&Cam_device))
    {
        printf("Failed to open camera\n");
        return;
    }

    // INITIALIZE CLUSTER
    if (open_cluster(&CL_device))
    {
        printf("Failed to open cluster\n");
        pmsis_exit(-1);
    }

    // INITIALIZE UART
    if (open_uart(&UART_device))
    {
        printf("Failed to open UART connection\n");
        pmsis_exit(-1);
    }

    // // MAKE SURE CAMEAR IS NOT SENDING DATA
    // pi_camera_control(&Cam_device, PI_CAMERA_CMD_STOP,0);

    // // CAPTURE FIRST IMAGE 
    // pi_task_block(&Cam_task);
    // pi_camera_capture_async(&Cam_device, ImgBuff[process_index1],CAM_WIDTH*CAM_HEIGHT, &Cam_task);
    // pi_camera_control(&Cam_device,PI_CAMERA_CMD_START,0);
    // pi_task_wait_on(&Cam_task);
    // t_delta[process_index1] = pi_time_get_us();


    // // CAPTURE SECOND IMAGE
    // pi_task_block(&Cam_task);
    // pi_camera_capture_async(&Cam_device, ImgBuff[process_index2],CAM_WIDTH*CAM_HEIGHT, &Cam_task);
    // pi_task_wait_on(&Cam_task);
    // t_delta[process_index2] = pi_time_get_us() - t_delta[process_index1];
}



void Cam_Processing(void)
{
    System_Init();
    printf("Main Loop start\n");

    // // CAPTURE IMAGES
    // uint32_t time_before = pi_time_get_us();
    // while (pi_time_get_us() - time_before < 1000000)
    // {
    //     // LAUNCH CAPTURE OF NEXT IMAGE
    //     pi_task_block(&Cam_task);
    //     pi_camera_capture_async(&Cam_device, ImgBuff[fill_index],CAM_WIDTH*CAM_HEIGHT, &Cam_task);


    //     // PROCESS THE CURRENT AND PREV IMAGES
    //     process_images(ImgBuff[process_index2],ImgBuff[process_index1]);
    //     pi_task_wait_on(&Cam_task);


    //     // ADVANCE BUFFER INDICES
    //     fill_index = (fill_index + 1) % NUM_BUFFERS;
    //     process_index1 = (process_index1 + 1) % NUM_BUFFERS;
    //     process_index2 = (process_index2 + 1) % NUM_BUFFERS;
    //     img_num_async++;
        
    // }
    // uint32_t time_after = pi_time_get_us();
    // float capture_time = (float)(time_after-time_before)/1000000;
    // float FPS_async = (float)img_num_async/capture_time;
    // printf("Capture FPS:        %.3f FPS\n",FPS_async);
    // printf("Capture Duration:   %.3f ms\n",capture_time/img_num_async*1000);
    // printf("Capture Count:      %d images\n",img_num_async);
    // printf("Capture Time:       %.6f s\n",capture_time);
    // printf("Exiting... \n");


    // PROCESS IMAGES
    // process_images(ImgBuff[1],ImgBuff[0]);

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


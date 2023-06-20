#include "cam_processing.h"





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
    int start_row = core_id * CL_ImageData->Rows_Per_Core + 1;
    int end_row = start_row + CL_ImageData->Rows_Per_Core - 1;


    // convolve2DSeparable(CL_ImageData->Cur_img_buff, G_up, Ku_v, Ku_h, start_row, end_row);
    // convolve2DSeparable(CL_ImageData->Cur_img_buff, G_vp, Kv_v, Kv_h, start_row, end_row);
    convolve2D(CL_ImageData->Cur_img_buff,G_up,Ku,start_row,end_row,2);
    convolve2D(CL_ImageData->Cur_img_buff,G_vp,Kv,start_row,end_row,2);
    radialGrad(CL_ImageData->Cur_img_buff,G_rp,G_up,G_vp,start_row,end_row,2);
    temporalGrad(CL_ImageData->Cur_img_buff,CL_ImageData->Prev_img_buff,G_tp,start_row,end_row,2);

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

  







static void process_images(uint8_t* Cur_img_buff, uint8_t* Prev_img_buff)
{

    struct ClusterImageData CL_ImageData;
    CL_ImageData.Rows_Per_Core = (CAM_HEIGHT - 2)/NUM_CORES;
    CL_ImageData.Cur_img_buff = Cur_img_buff;
    CL_ImageData.Prev_img_buff = Prev_img_buff;


    struct pi_cluster_task CL_Grad_task;
    struct pi_cluster_task CL_DotProducts_task;


    printf("Start Processing... \n");   
    time_before = pi_time_get_us();
    pi_cluster_task(&CL_Grad_task, delegate_GradCalcs, &CL_ImageData);
    pi_cluster_send_task(&CL_device,&CL_Grad_task);

    // convolve2D(CL_ImageData.Cur_img_buff,G_up,Ku,1,CAM_HEIGHT-2,2);
    // convolve2D(CL_ImageData.Cur_img_buff,G_vp,Kv,1,CAM_HEIGHT-2,2);
    // radialGrad(CL_ImageData.Cur_img_buff,G_rp,G_up,G_vp,1,CAM_HEIGHT-2,2);
    // temporalGrad(CL_ImageData.Cur_img_buff,CL_ImageData.Prev_img_buff,G_tp,1,CAM_HEIGHT-2,2);
    

    pi_cluster_task(&CL_DotProducts_task, delegate_DotProducts, &CL_ImageData);
    pi_cluster_send_task(&CL_device,&CL_DotProducts_task);
    time_after = pi_time_get_us();
    print_image_int32(CL_ImageData.UART_array,9,1);
    printf("Calc Time: %d us\n",(time_after-time_before));   
    // print_image_int32(G_up,CAM_WIDTH,CAM_HEIGHT);


    // Need ~30,000 us calc

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
    if (open_cluster(&CL_device))
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

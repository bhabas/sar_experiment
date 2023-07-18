#include "Cam_Processing.h"










/**
 * @brief Delegates image gradient task to each cluster core. This function is executed by core 0.
 * 
 * @param arg (ClusterCompData_t)
 */
void Delegate_Gradient_Calcs(void *arg)
{
    ClusterCompData_t* CL_ImageData = (ClusterCompData_t *)arg;
    pi_cl_team_fork(pi_cl_cluster_nb_cores(), Cluster_GradientCalcs, arg);
}


/**
 * @brief This function assigns each core a specific image section and stride to 
 * compute brightness gradients (G_up and G_vp), radial image gradient G_rp, and temporal gradient (G_tp)
 * 
 * @param arg (ClusterCompData_t)
 */
void Cluster_GradientCalcs(void *arg)
{
    // DETERMINE STRIDE AND CALCULATION RANGE FOR CLUSTER CORE
    ClusterCompData_t* CL_ImageData = (ClusterCompData_t *)arg;
    uint32_t core_id = pi_core_id();
    int32_t stride = CL_ImageData->Stride;
    int32_t start_row = core_id * CL_ImageData->Rows_Per_Core + 1;
    int32_t end_row = start_row + CL_ImageData->Rows_Per_Core - 1;

    // CALCULATE IMAGE GRADIENTS OVER SPECIFIED RANGE
    convolve2D(CL_ImageData->Cur_img_buff,G_up,Ku,start_row,end_row,stride);
    convolve2D(CL_ImageData->Cur_img_buff,G_vp,Kv,start_row,end_row,stride);
    pi_cl_team_barrier(); // Wait for all cores to finish

    // CALCULATE RADIAL AND TEMPORAL GRADIENTS OVER SPECIFIED RANGE
    radialGrad(CL_ImageData->Cur_img_buff,G_rp,G_up,G_vp,start_row,end_row,stride);
    temporalGrad(CL_ImageData->Cur_img_buff,CL_ImageData->Prev_img_buff,G_tp,start_row,end_row,stride);
    pi_cl_team_barrier();

}





/**
 * @brief Calculates all the necessary dot products for every combination of gradient 
 * vectors. Calculations are done in parallel with each core handling a row range of 
 * the image and then combined to a total sum value.
 * 
 * | G_tp•G_vp |   | G_vp•G_vp  G_up•G_vp G_rp•G_vp | | Theta_x |
 * | G_tp•G_up | = | G_vp•G_up  G_up•G_up G_rp•G_up | | Theta_y |
 * | G_tp•G_rp |   | G_vp•G_rp  G_up•G_rp G_rp•G_rp | | Theta_z |
 * 
 * @param arg (ClusterCompData_t)
 */
void Delegate_DotProduct_Calcs(void *arg)
{
    ClusterCompData_t* CL_ImageData = (ClusterCompData_t *)arg;

    uint8_t array_len = 4; // Four gradient terms
    int32_t* arrays[4] = {G_vp, G_up, G_rp, G_tp};
    uint8_t UART_index = 0;
    // UART_Array_Order: (G_vp•G_vp, G_vp•G_up, G_vp•G_rp, G_vp•G_tp, G_up•G_up, G_up•G_rp, G_up•G_tp, G_rp•G_rp, G_rp•G_tp)

    for (int i = 0; i < array_len; i++)
    {
        for (int j = i; j < array_len; j++)
        {
            // Avoid duplicate pairings and exclude G_tp•G_tp
            if ((i != j && i > j) || (i == j && i == 3))
            {
                continue;
            }

            CL_ImageData->DP_Vec1 = arrays[i];
            CL_ImageData->DP_Vec2 = arrays[j];
            CL_ImageData->DP_Sum = &(CL_ImageData->UART_array[UART_index]);
            pi_cl_team_fork(pi_cl_cluster_nb_cores(), Cluster_DotProduct, CL_ImageData);

            // printf("Dot product of array %d and %d: %f\n", i, j, CL_ImageData->UART_array[UART_index]);
            UART_index++;
        }
        
    }

    
}

/**
 * @brief This function assigns each core a specific image section and stride to 
 * compute brightness gradients (G_up and G_vp), radial image gradient G_rp, and temporal gradient (G_tp)
 * 
 * @param arg (ClusterCompData_t)
 */
void Cluster_DotProduct(void *arg)
{
    // CALC LIMITS FOR CORE
    ClusterCompData_t* CL_ImageData = (ClusterCompData_t *)arg;
    uint32_t core_id = pi_core_id();
    int32_t start_row = core_id * CL_ImageData->Rows_Per_Core + 1;
    int32_t end_row = start_row + CL_ImageData->Rows_Per_Core - 1;


    // CALC DOT PRODUCT INSIDE CORE LIMITS
    int32_t sum = 0;
    for (int32_t i = start_row; i <= end_row; i++)
    {
        for (int32_t j = 1; j < N_up-1; j++)
        {
            sum += CL_ImageData->DP_Vec1[N_up*i + j] * CL_ImageData->DP_Vec2[N_up*i + j];
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

  
void Process_Images(uint8_t* Cur_img_buff, uint8_t* Prev_img_buff, uint32_t t_delta)
{

    struct ClusterCompData CL_ImageData;
    CL_ImageData.Rows_Per_Core = (N_vp - 2)/NUM_CORES;
    CL_ImageData.Cur_img_buff = Cur_img_buff;
    CL_ImageData.Prev_img_buff = Prev_img_buff;
    CL_ImageData.Stride = 2;

    // CALC IMAGE GRADIENTS VIA CLUSTER PARALLEL COMPUTATION
    struct pi_cluster_task CL_Grad_task;
    pi_cluster_task(&CL_Grad_task, Delegate_Gradient_Calcs, &CL_ImageData);
    pi_cluster_send_task(&CL_device,&CL_Grad_task);

    // CALC GRADIENT DOT PRODUCTS VIA CLUSTER PARALLEL COMPUTATION
    struct pi_cluster_task CL_DotProducts_task;
    pi_cluster_task(&CL_DotProducts_task, Delegate_DotProduct_Calcs, &CL_ImageData);
    pi_cluster_send_task(&CL_device,&CL_DotProducts_task);

    // INCLUDE TIME BETWEEN IMAGES
    CL_ImageData.UART_array[10] = (int32_t)t_delta/1000; // Delta_t
    CL_ImageData.UART_array[11] = (int32_t)N_up;
    CL_ImageData.UART_array[12] = (int32_t)N_vp;

    CL_ImageData.UART_array[13] = 0;
    CL_ImageData.UART_array[14] = 0;
    CL_ImageData.UART_array[15] = 0;


    
    // SEND CALC DATA TO CRAZYFLIE FOR FINAL COMPUTATION
    /* Do UART1 Stuff*/
    // send_uart_arr(&UART_device,CL_ImageData.UART_array);


}

void test_func(void* arg)
{
    uint32_t* t_end = (uint32_t*)arg;
    *t_end = pi_time_get_us();
}

void System_Init(void)
{
    printf("-- Starting Camera Test --\n");

    // INITIALIZE CAMERA
    if (open_pi_camera_himax(&Cam_device))
    {
        printf("[CAMERA] Failed to open camera\n");
        pmsis_exit(-1);
    }

    // INITIALIZE CLUSTER
    if (open_cluster(&CL_device))
    {
        printf("[CLUSTER] Failed to open cluster device\n");
        pmsis_exit(-1);
    }

    // INITIALIZE UART
    if (open_uart(&UART_device))
    {
        printf("[UART] Failed to open UART device\n");
        pmsis_exit(-1);
    }


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

    // MAKE SURE CAMERA IS NOT SENDING DATA
    pi_camera_control(&Cam_device, PI_CAMERA_CMD_STOP,0);
    pi_camera_control(&Cam_device,PI_CAMERA_CMD_START,0);

    // CAPTURE FIRST IMAGE 
    pi_task_callback(&Cam_Capture_Task, test_func, (void *)&t_cap);
    pi_camera_capture_async(&Cam_device, ImgBuff[prev_img_index],N_up*N_vp, &Cam_Capture_Task);
    t_start = pi_time_get_us();
    pi_task_wait_on(&Cam_Capture_Task);

    // INITIAL CAPTURE TIME
    t_capture[prev_img_index] = t_cap;

    // CAPTURE SECOND IMAGE
    pi_task_callback(&Cam_Capture_Task, test_func, (void *)&t_cap);
    pi_camera_capture_async(&Cam_device, ImgBuff[cur_img_index],N_up*N_vp, &Cam_Capture_Task);
    pi_task_wait_on(&Cam_Capture_Task);

    // NEW CAPTURE TIME
    t_capture[cur_img_index] = t_cap;
    t_delta[cur_img_index] = t_capture[cur_img_index] - t_capture[prev_img_index];
}



void OpticalFlow_Processing_Test(void)
{
    System_Init();
    printf("Main Loop start\n");

    // Initialize your data array
    int32_t data[UART_ARR_SIZE] = {0,0,0,0,0,0,0,0,0,0,N_up,N_vp,0,0,0,0};
    
    // CAPTURE IMAGES
    uint32_t time_before = pi_time_get_us();
    while (pi_time_get_us() - time_before < 3*1000000)
    {
        // START CAPTURE OF NEXT IMAGE
        pi_task_callback(&Cam_Capture_Task, test_func, (void*)&t_cap);
        pi_camera_capture_async(&Cam_device, ImgBuff[capture_index],N_up*N_vp, &Cam_Capture_Task);

        // PROCESS THE CURRENT AND PREV IMAGES
        Process_Images(ImgBuff[cur_img_index],ImgBuff[prev_img_index],t_delta[cur_img_index]);


        pi_task_wait_on(&Cam_Capture_Task);

        // GRAB NEW CAPTURE TIME
        t_capture[capture_index] = t_cap;
        t_delta[capture_index] = t_capture[capture_index] - t_capture[cur_img_index];


        data[prev_img_index] = (t_capture[prev_img_index]-t_start)/1000;
        data[cur_img_index] =  (t_capture[cur_img_index]-t_start)/1000;
        data[capture_index] =  (t_capture[capture_index]-t_start)/1000;

        data[prev_img_index+3] = (t_delta[prev_img_index])/1000;
        data[cur_img_index+3] =  (t_delta[cur_img_index])/1000;
        data[capture_index+3] =  (t_delta[capture_index])/1000;

        send_uart_arr(&UART_device,data);


        


        // INCREMENT IMAGE COUNT
        img_count++;


        // DEBUG PRINT
        if (img_count == 60)
        {
            for (size_t i = 0; i < NUM_BUFFERS; i++)
            {
                printf("idx: %d \t t_cap:   %lu\n",i,t_capture[i]);
            }

            for (size_t i = 0; i < NUM_BUFFERS; i++)
            {
                printf("idx: %d \t t_delta: %lu\n",i,t_delta[i]);
            }
            break;

        }

        // ADVANCE BUFFER INDICES
        capture_index =  (capture_index + 1) % NUM_BUFFERS;
        prev_img_index = (prev_img_index + 1) % NUM_BUFFERS;
        cur_img_index =  (cur_img_index + 1) % NUM_BUFFERS;

        
    }
    uint32_t time_after = pi_time_get_us();
    float capture_time = (float)(time_after-time_before)/1000000;
    float FPS_async = (float)img_count/capture_time;
    printf("Capture FPS:        %.3f FPS\n",FPS_async);
    printf("Capture Duration:   %.3f ms\n",capture_time/img_count*1000);
    printf("Capture Count:      %d images\n",img_count);
    printf("Capture Time:       %.6f s\n",capture_time);
    printf("Exiting... \n");

    
    
    
    // while (1)
    // {

    //     // INCREMENT FIRST DATA VALUE
    //     data[0]++;

    //     send_uart_arr(&UART_device,data);
    //     pi_time_wait_us(33000);
    // }

    pmsis_exit(0);

}

/* Program Entry. */
int main(void)
{
    pi_bsp_init();
    


    // Increase the FC freq to 250 MHz
    pi_freq_set(PI_FREQ_DOMAIN_FC, CLOCK_FREQ);
    __pi_pmu_voltage_set(PI_PMU_DOMAIN_FC, 1200); // Not sure on why set voltage?

    return pmsis_kickoff((void *)OpticalFlow_Processing_Test);
}


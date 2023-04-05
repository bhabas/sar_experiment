// PMSIS INCLUDES
#include "pmsis.h"
#include "bsp/bsp.h" 

// CF INCLUDES
#include "cpx.h" 

#define FC_CLOCK_FREQ_50_MHZ    (50*1000000)
#define FC_CLOCK_FREQ_100_MHZ   (100*1000000)
#define FC_CLOCK_FREQ_150_MHZ   (150*1000000)
#define FC_CLOCK_FREQ_200_MHZ   (200*1000000)
#define FC_CLOCK_FREQ_250_MHZ   (250*1000000)

#define CL_CLOCK_MIN_FREQ       (87*1000000)
#define CL_CLOCK_MAX_FREQ       (175*1000000)

#define MS_2_US 1000




void helloworld(void)
{
    cpxPrintToConsole(LOG_TO_CRTP,"\n\n\t *** PMSIS Perf Counter2 ***\n\n");
    
    // INIT PERFORMANCE COUNTERS
    pi_perf_conf(1 << PI_PERF_CYCLES | 1 << PI_PERF_INSTR);

    // PRINT FABRIC CONTROLLER CLUSTER/CORE DETAILS
    cpxPrintToConsole(LOG_TO_CRTP,"Entering Fabric Controller\n");
    uint32_t core_id = pi_core_id();
    uint32_t cluster_id = pi_cluster_id();
    cpxPrintToConsole(LOG_TO_CRTP,"[%d %d] Hello World!\n\n", cluster_id, core_id);


    // INIT CLUSTER CONFIGURATION STRUCTURE
    cpxPrintToConsole(LOG_TO_CRTP,"Initializing Main Cluster Core\n\n");
    struct pi_device cl_device = {0};       // Define all fields to be '0'
    struct pi_cluster_conf cl_config = {0}; // Define all fields to be '0'
    pi_cluster_conf_init(&cl_config);       // Init default cluster configs

    // CONFIGURE AND OPEN CLUSTER
    pi_open_from_conf(&cl_device, &cl_config);
    if (pi_cluster_open(&cl_device))
    {
        cpxPrintToConsole(LOG_TO_CRTP,"[DEBUG] Cluster open failed!\n");
        pmsis_exit(-1);
    }



    // PRINT CLOCK FREQ. DETAILS
    cpxPrintToConsole(LOG_TO_CRTP,"Initial Clock Frequency\n");
    int32_t cur_fc_freq = pi_freq_get(PI_FREQ_DOMAIN_FC);
    int32_t cur_cl_freq = pi_freq_get(PI_FREQ_DOMAIN_CL);
    cpxPrintToConsole(LOG_TO_CRTP,"FC frequency : %ld\nCL frequency : %ld\n\n", cur_fc_freq, cur_cl_freq);

    // UPDATE AND RE-PRINT CLOCK FREQ. DETAILS
    cpxPrintToConsole(LOG_TO_CRTP,"Updated Clock Frequency\n");
    // pi_pmu_voltage_set(PI_PMU_DOMAIN_FC, 1200);
    // pi_freq_set(PI_FREQ_DOMAIN_FC,51000000);
    pi_freq_set(PI_FREQ_DOMAIN_CL,CL_CLOCK_MAX_FREQ);
    cur_fc_freq = pi_freq_get(PI_FREQ_DOMAIN_FC);
    cur_cl_freq = pi_freq_get(PI_FREQ_DOMAIN_CL);
    cpxPrintToConsole(LOG_TO_CRTP,"FC frequency : %ld\nCL frequency : %ld\n\n", cur_fc_freq, cur_cl_freq);


    // PERFORMANCE COUNTER CHECK
    pi_perf_start();
    pi_time_wait_us(1000*MS_2_US);
    pi_perf_stop();

    cpxPrintToConsole(LOG_TO_CRTP,"Instructions Counted: %d\n",pi_perf_read(PI_PERF_INSTR));
    cpxPrintToConsole(LOG_TO_CRTP,"Cycles Counted: %d\n",pi_perf_read(PI_PERF_CYCLES));
    // cpxPrintToConsole(LOG_TO_CRTP,"Time paused: %.2f [s]\n",(double)pi_perf_read(PI_PERF_CYCLES)/51000000);


    // EXIT PROGRAM EXECUTION
    cpxPrintToConsole(LOG_TO_CRTP,"\n\n\t *** End of program ***\n\n");
    pmsis_exit(-1);

}

// PROGRAM ENTRY
int main(void)
{
    cpxInit();  
    return pmsis_kickoff((void *) helloworld);
}
// PMSIS INCLUDES
#include "pmsis.h"
#include "bsp/bsp.h" 

// CF INCLUDES
#include "cpx.h" 

int cluster_id_array[8] = {0};
int core_id_array[8] = {0};

// TASK EXECUTED BY CLUSTER CORES
void cluster_helloworld(void *arg)
{
    int cluster_id = pi_cluster_id();
    cluster_id_array[cluster_id] = cluster_id;

    int core_id = pi_core_id();
    core_id_array[core_id] = core_id;
    
}

// CLUSTER MAIN TASK EXECUTED BY CORE 0
void cluster_delegate(void *arg)
{
    // SEND SECONDARY TASK TO ALL CLUSTER CORES IN PARALLEL
    pi_cl_team_fork(pi_cl_cluster_nb_cores(), cluster_helloworld, arg);
}


void helloworld(void)
{
    cpxPrintToConsole(LOG_TO_CRTP,"\n\n\t *** PMSIS HelloWorld ***\n\n");


    cpxPrintToConsole(LOG_TO_CRTP,"Entering Fabric Controller\n");
    uint32_t core_id = pi_core_id();
    uint32_t cluster_id = pi_cluster_id();
    cpxPrintToConsole(LOG_TO_CRTP,"[%d %d] Hello World!\n\n", cluster_id, core_id);


    // INIT MAIN CLUSTER CORE
    cpxPrintToConsole(LOG_TO_CRTP,"Entering Main Cluster Core [Cluster ID,Core ID]\n");

    struct pi_device cl_device = {0};       // Define all fields to be '0'
    struct pi_cluster_conf cl_config = {0}; // Define all fields to be '0'
    pi_cluster_conf_init(&cl_config);       // Init default cluster configs
    cl_config.id = 0;                       // Set cluster ID

    // OPEN CLUSTER
    pi_open_from_conf(&cl_device, &cl_config);
    if (pi_cluster_open(&cl_device))
    {
        cpxPrintToConsole(LOG_TO_CRTP,"[DEBUG] Cluster open failed !\n");
        pmsis_exit(-1);
    }
    else
    {
        cpxPrintToConsole(LOG_TO_CRTP,"[DEBUG] Cluster open succeeded!\n");
    }

    
    // PREPARE CLUSTER TASK AND SEND IT TO CLUSTER
    struct pi_cluster_task cl_task = {0};
    cl_task.entry = cluster_delegate;
    cl_task.arg = NULL;
    pi_cluster_send_task_to_cl(&cl_device, &cl_task);
    pi_cluster_close(&cl_device);

    // PRINT ACTIVATED CLUSTERS
    for (int i = 0; i < pi_cl_cluster_nb_cores(); i++) 
    {
        cpxPrintToConsole(LOG_TO_CRTP,"[%d %d] Hello World!\n", cluster_id_array[i], core_id_array[i]);
    }

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
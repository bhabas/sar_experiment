#include "pmsis.h" // for the drivers
#include "bsp/bsp.h" // for some configuration parameters (pad configurations for connecting to memory, camera, etc.)
#include "cpx.h" // for using the CPX functions to send our hello world to the console

void start_example(void)
{
    pi_bsp_init();
    cpxInit();   

    while (1)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Hello World\n");
        pi_time_wait_us(1000*1000);
    }


}

int main(void)
{
    return pmsis_kickoff((void *)start_example);
}
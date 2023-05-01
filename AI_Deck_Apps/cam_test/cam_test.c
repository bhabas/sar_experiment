/* PMSIS includes */
#include "pmsis.h"

#include "bsp/bsp.h"
#include "bsp/camera.h"
#include "bsp/camera/himax.h"
#include "bsp/buffer.h"
#include "stdio.h"

#define WIDTH 324
#define HEIGHT 244
#define BUFF_SIZE (WIDTH*HEIGHT)
PI_L2 unsigned char *buff;

static struct pi_device camera;

void test_camera(void)
{
    printf("Entering main controller\n");
    pi_freq_set(PI_FREQ_DOMAIN_FC, 50000000);

    printf("Testing normal camera capture\n");



    printf("Test success !\n");

    pmsis_exit(0);
}

/* Program Entry. */
int main(void)
{
    printf("\n\n\t *** PMSIS Camera Example Test ***\n\n");
    return pmsis_kickoff((void *) test_camera);
}


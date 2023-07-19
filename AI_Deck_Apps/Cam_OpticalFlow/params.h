#pragma once

#define IMG_ORIENTATION 0x0101

#define N_up 162
#define N_vp 22


#define CLOCK_FREQ 250*1000000 // [MHz]
#define CLOCK_FREQ_CLUSTER 150*1000000
#define NUM_CORES 8



#define NUM_BUFFERS 3
#define RESOLUTION N_up*N_vp
#define BUFFER_SIZE N_up*N_vp*sizeof(uint8_t)






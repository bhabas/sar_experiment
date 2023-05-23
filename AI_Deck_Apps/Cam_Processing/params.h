#pragma once

#define IMG_ORIENTATION 0x0101
#define CAM_WIDTH 162 
#define CAM_HEIGHT 122
#define CLOCK_FREQ 250*1000000 // [MHz]
#define CLOCK_FREQ_CL 150*1000000
#define NUM_CORES 8



#define NUM_BUFFERS 2
#define RESOLUTION CAM_WIDTH*CAM_HEIGHT
#define BUFFER_SIZE CAM_WIDTH*CAM_HEIGHT*sizeof(uint8_t)

struct pi_device cl_dev;



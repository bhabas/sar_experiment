#pragma once

#define IMG_ORIENTATION 0x0101
#define CAM_WIDTH 8
#define CAM_HEIGHT 18
#define CLOCK_FREQ 250*1000000 // [MHz]
#define ROWS_PER_CORE 2

#define NUM_BUFFERS 2
#define RESOLUTION CAM_WIDTH*CAM_HEIGHT
#define BUFFER_SIZE CAM_WIDTH*CAM_HEIGHT*sizeof(uint8_t)

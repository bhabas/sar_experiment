#pragma once

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "params.h"
#include "pmsis.h"

void test_func();
void print_image_int32(int32_t* ImgBuff, uint8_t Cam_Width, uint8_t Cam_Height);


void convolve2D(uint8_t* img, int32_t* result, int32_t* kernel, int startRow, int endRow);
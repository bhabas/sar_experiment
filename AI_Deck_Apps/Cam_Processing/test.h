#pragma once

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "params.h"
#include "pmsis.h"

void printVal(int32_t val);
void print_image_int32(int32_t* ImgBuff, uint8_t Cam_Width, uint8_t Cam_Height);
void print_image_uint8(uint8_t* ImgBuff, uint8_t Cam_Width, uint8_t Cam_Height);



void convolve2D(uint8_t* img, int32_t* result, int32_t* kernel, int32_t startRow, int32_t endRow, int32_t stride);
void convolve2DSeparable(uint8_t* img, int32_t* result, int32_t* Kv, int32_t* Kh, int32_t startRow, int32_t numRows);
void radialGrad(uint8_t* img, int32_t* result, int32_t* G_up, int32_t* G_vp, int32_t startRow, int32_t endRow, int32_t stride);
void temporalGrad(uint8_t* Cur_img_buff, uint8_t* Prev_img_buff, int32_t* result, int32_t startRow, int32_t endRow, int32_t stride);


int32_t dotProduct(int32_t* Vec1, int32_t* Vec2, int32_t size);
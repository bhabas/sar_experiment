#pragma once

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "params.h"
#include "pmsis.h"

void print_image_int32(int32_t* ImgBuff, uint8_t Cam_Width, uint8_t Cam_Height);
void print_image_uint8(uint8_t* ImgBuff, uint8_t Cam_Width, uint8_t Cam_Height);



void convolve2D(uint8_t* img, int32_t* result, int32_t* kernel, int startRow, int endRow);
void convolve2DSeparable(uint8_t* img, int32_t* result, int32_t* Kv, int32_t* Kh, int startRow, int endRow);
void temporalGrad(uint8_t* Cur_img_buff, uint8_t* Prev_img_buff, int32_t* result, int startRow, int endRow);
void radialGrad(uint8_t* img, int32_t* result, int32_t* G_up, int32_t* G_vp, int startRow, int endRow);

int32_t dotProduct(int32_t* Vec1, int32_t* Vec2, int32_t size);
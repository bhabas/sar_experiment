#pragma once

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

void print_image_uint8(uint8_t* ImgBuff, uint8_t Cam_Width, uint8_t Cam_Height)
{
    for (int i = 0; i < Cam_Height; i++) {
        for (int j = 0; j < Cam_Width; j++) {
            printf("%3d ",ImgBuff[i*Cam_Width + j]);
        }
        printf("\n");
    }
    printf("\n");

}

void print_image_uint32(uint32_t* ImgBuff, uint8_t Cam_Width, uint8_t Cam_Height)
{
    for (int i = 0; i < Cam_Height; i++) {
        for (int j = 0; j < Cam_Width; j++) {
            printf("%3d ",ImgBuff[i*Cam_Width + j]);
        }
        printf("\n");
    }
    printf("\n");

}

void print_image_int32(int32_t* ImgBuff, uint8_t Cam_Width, uint8_t Cam_Height)
{
    for (int i = 0; i < Cam_Height; i++) {
        for (int j = 0; j < Cam_Width; j++) {
            printf("%3d ",ImgBuff[i*Cam_Width + j]);
        }
        printf("\n");
    }
    printf("\n");

}
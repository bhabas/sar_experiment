#include "test.h"


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


void convolve2D(uint8_t* img, int32_t* result, int32_t* kernel, int startRow, int endRow)
{
    for (int32_t v_p = startRow; v_p <= endRow; v_p += 1)
    {
        for (int32_t u_p = 1; u_p < CAM_WIDTH -1; u_p += 1)
        {       
            int32_t sum = 0;
            for (int32_t i = 0; i <= 2; i++)
            {
                for (int32_t j = 0; j <= 2; j++)
                {
                    // Handle image boundaries
                    if (v_p + i-1 < 0 || v_p + i-1 >= CAM_HEIGHT)
                    {
                        continue;
                    }

                    int32_t curPos = (v_p + i-1) * CAM_WIDTH + (u_p + j-1);
                    int32_t kerPos = i*3 + j;
                    sum += img[curPos] * kernel[kerPos];            
                }
            }
            result[v_p*CAM_WIDTH + u_p] = sum;
        }
    }
}

int32_t dotProduct(int32_t* Vec1, int32_t* Vec2, int32_t size)
{
    int32_t result = 0;
    for (int i = 0; i < size; i++)
    {
        result += Vec1[i] * Vec2[i];
    }
    return result;
}

void radialGrad(uint8_t* img, int32_t* result, int32_t* G_up, int32_t* G_vp, int startRow, int endRow)
{
    for (int32_t v_p = startRow; v_p <= endRow; v_p += 1)
    {
        for (int32_t u_p = 1; u_p < CAM_WIDTH -1; u_p += 1)
        {
            int32_t curPos = v_p* CAM_WIDTH + u_p;
            result[curPos] = (2*u_p - CAM_WIDTH + 1)*G_up[curPos] + (2*v_p - CAM_HEIGHT + 1)*G_vp[curPos];
        }
    }
}

void temporalGrad(uint8_t* Cur_img_buff, uint8_t* Prev_img_buff, int32_t* result, int startRow, int endRow)
{
    for (int i = startRow; i <= endRow; i++)
    {
        for (int j = 1; j < CAM_WIDTH-1; j++)
        {
            result[i*CAM_WIDTH + j] = Cur_img_buff[i*CAM_WIDTH + j] - Prev_img_buff[i*CAM_WIDTH + j];
        }
    }
}

void convolve2DSeparable(uint8_t* img, int32_t* result, int32_t* Kv, int32_t* Kh, int32_t startRow, int32_t endRow)
{

    // Temporary result after vertical convolution
    int32_t numRows = (endRow - startRow) + 1;
    int32_t temp_result[CAM_WIDTH*numRows];

    printf("here\n");

    for (int32_t v_p = startRow; v_p <= endRow; v_p += 1)
    {
        for (int32_t u_p = 0; u_p < CAM_WIDTH; u_p += 1)
        {
            int32_t sum = 0;
            for (int32_t i = 0; i <= 2; i++)
            {
                // Handle image boundaries
                if (v_p + i-1 < 0 || v_p + i-1 >= CAM_HEIGHT)
                {
                    continue;
                }

                int32_t imgPos = (v_p + i-1) * CAM_WIDTH + u_p;
                sum += img[imgPos] * Kv[i]; 
            }
            // temp_result[v_p*CAM_WIDTH + u_p] = sum;
            // temp_result[(v_p-startRow)*CAM_WIDTH + u_p] = 0;
            temp_result[u_p] = 0;
        }
    }
    printf("here\n");
    return;

    // // Horizontal convolution
    // for (int32_t v_p = startRow; v_p <= endRow; v_p += 1)
    // {
    //     for (int32_t u_p = 1; u_p < CAM_WIDTH -1; u_p += 1)
    //     {
    //         int32_t sum = 0;
    //         for (int32_t j = 0; j <= 2; j++)
    //         {
    //             // Handle image boundaries
    //             if (u_p + j-1 < 0 || u_p + j-1 >= CAM_WIDTH)
    //             {
    //                 continue;
    //             }

    //             int32_t curPos = (v_p - startRow) * CAM_WIDTH + (u_p + j-1);
    //             sum += temp_result[curPos] * Kh[j];            
    //         }
    //         result[v_p*CAM_WIDTH + u_p] = sum;
    //     }
    // }


}
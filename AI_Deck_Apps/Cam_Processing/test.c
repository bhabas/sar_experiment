#include "test.h"

void test_func()
{
    printf("Apple\n");
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

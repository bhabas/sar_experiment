#include "CalcFuncs.h"


// ========================
//     DEBUG FUNCTIONS
// ========================

void printVal(int32_t val)
{
    printf("Val: %d\n",val);           

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


// ========================
//     IMAGE PROCESSING
// ========================
int32_t dotProduct(int32_t* Vec1, int32_t* Vec2, int32_t size)
{
    int32_t result = 0;
    for (int i = 0; i < size; i++)
    {
        result += Vec1[i] * Vec2[i];
    }
    return result;
}

void radialGrad(uint8_t* img, int32_t* result, int32_t* G_up, int32_t* G_vp, int32_t startRow, int32_t endRow, int32_t stride)
{
    for (int32_t v_p = startRow; v_p <= endRow; v_p += stride)
    {
        for (int32_t u_p = 1; u_p < N_up - 1; u_p += stride)
        {
            int32_t curPos = v_p*N_up + u_p;
            result[curPos] = (2*u_p - N_up + 1)*G_up[curPos] + (2*v_p - N_vp + 1)*G_vp[curPos];
        }
    }
}

void temporalGrad(uint8_t* Cur_img_buff, uint8_t* Prev_img_buff, int32_t* result, int32_t startRow, int32_t endRow, int32_t stride)
{
    for (int i = startRow; i <= endRow; i += stride)
    {
        for (int j = 1; j < N_up-1; j += stride)
        {
            result[i*N_up + j] = Cur_img_buff[i*N_up + j] - Prev_img_buff[i*N_up + j];
        }
    }
}

void convolve2D(uint8_t* img, int32_t* result, int32_t* kernel, int32_t startRow, int32_t endRow, int32_t stride)
{
    for (int32_t v_p = startRow; v_p <= endRow; v_p += stride)
    {
        for (int32_t u_p = 1; u_p < N_up - 1; u_p += stride)
        {       
            int32_t sum = 0;
            for (int32_t i = 0; i <= 2; i++)
            {
                for (int32_t j = 0; j <= 2; j++)
                {
                    // Handle image boundaries
                    if (v_p + i-1 < 0 || v_p + i-1 >= N_vp)
                    {
                        continue;
                    }

                    int32_t curPos = (v_p + i-1) * N_up + (u_p + j-1);
                    int32_t kerPos = i*3 + j;
                    sum += img[curPos] * kernel[kerPos];            
                }
            }
            result[v_p*N_up + u_p] = sum;
        }
    }
}


void convolve2DSeparable(uint8_t* img, int32_t* result, int32_t* Kv, int32_t* Kh, int32_t startRow, int32_t numRows)
{

    // int32_t* temp = (int32_t*) pi_l2_malloc(N_up * sizeof(int32_t));
    // int32_t* temp = (int32_t*) pi_cl_l1_malloc(&CL_device,N_up * sizeof(int32_t));
    int32_t temp[N_up] = {0};




    // Ensure we don't go beyond the image height
    int32_t endRow = startRow + numRows;
    if (endRow > N_vp) {
        endRow = N_vp;
    }



    // Loop through the rows
    for (int32_t r = startRow; r < endRow; ++r)
    {
        // Vertical convolution pass
        for (int32_t c = 0; c < N_up - 0; ++c) // Skip the first and last column
        {
            temp[c] = 0;
            for (int32_t kr = -1; kr <= 1; ++kr)
            {
                // Skip out of bounds
                if (r+kr < 0 || r+kr >= N_vp) continue;
                
                // Apply vertical kernel
                temp[c] += Kv[kr+1] * img[(r+kr)*N_up + c];
            }
        }

        // Horizontal convolution pass, ensuring we don't convolve the left and right borders
        for (int32_t c = 1; c < N_up - 1; ++c) // Skip the first and last column
        {
            result[r*N_up + c] = 0;
            for (int32_t kc = -1; kc <= 1; ++kc)
            {
                // Apply horizontal kernel
                result[r*N_up + c] += Kh[kc+1] * temp[c+kc];
            }
        }
    }

    // pi_cl_l1_free(&CL_device,temp,N_up * sizeof(int32_t));
    // pi_l2_free(temp,N_up * sizeof(int32_t));

}
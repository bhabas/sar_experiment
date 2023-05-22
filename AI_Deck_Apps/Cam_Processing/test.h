#pragma once

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "params.h"

void test_func();

void convolve2D(uint8_t* img, int32_t* result, int32_t* kernel, int startRow, int endRow);
/**
Copyright 20201 Andrei N. Ciobanu

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

// https://www.andreinc.net/2021/01/20/writing-your-own-linear-algebra-matrix-library-in-c#add-two-matrices

#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>

#include "nml.h"

#define DEFAULT_VALUE 0.0

#define CANNOT_ADD "Cannot add two matrices with different dimensions.\n"

#define CANNOT_SUBTRACT "Cannot subctract two matrices with different dimensions.\n"

#define CANNOT_MULITPLY \
  "Cannot multiply two matrices where \
  the number of columns of the first one \
  is different than the number of rows of the second one.\n" \

#define CANNOT_REMOVE_COLUMN "Cannot remove matrix column %d. The value should be less than %d.\n" 

#define CANNOT_REMOVE_ROW "Cannot remove matrix row %d. The value should be less than %d.\n" 

#define INVALID_ROWS \
  "Cannot create matrix with 0 number of rows. Aborting.\n" \

#define INVALID_COLS \
    "Cannot create matrix with 0 number of cols. Aborting.\n" \

#define CANNOT_TRACE \
    "Cannot calculate trace. Matrix needs to be square.\n" \

#define CANNOT_CROUT \
    "Cannot apply crout algorithm. Matrix needs to be square.\n" \

#define CANNOT_SWAP_ROWS \
     "Cannot swap rows (%d, %d) because the matrix number of rows is %d.\n" \

#define CANNOT_SWAP_COLUMNS \
      "Cannot swap columns (%d, %d) because the matrix number or columns is %d.\n" \

#define CANNOT_ROW_MULTIPLY \
      "Cannot multiply row (%d), maximum number of rows is %d.\n" \

#define CANNOT_COL_MULTIPLY "Cannot multiply col (%d), maximum number of columns is %d.\n" 
  
#define CANNOT_ADD_TO_ROW \
      "Cannot add %2.2f x (row=%d) to row=%d. Total number of rows is: %d.\n" \

#define CANNOT_LU_MATRIX_SQUARE \
      "Canot LU. Matrix (%d, %d) needs to be square.\n" \

#define CANNOT_LU_MATRIX_DEGENERATE \
      "Cannot LU. Matrix is degenerate or almost degenerate.\n" \

#define CANNOT_SOLVE_LIN_SYS_INVALID_B \
      "Cannot solve system. b[%d][%d] should have size b[%d][%d].\n" \

#define CANNOT_SET_DIAG \
      "Cannot set diag with value(=%2.2f). Matrix is not square.\n" \

#define CANNOT_CONCATENATE_H \
      "Cannot concatenate. Matrices have a different number of rows. Expected %d, found: %d.\n" \

#define CANNOT_CONCATENATE_V \
      "Cannot concatenate. Matrices have a different number of cols. Expected %d, found: %d.\n" \

#define CANNOT_GET_COLUMN \
      "Cannot get column (%d). The matrix has %d number of columns.\n" \

#define CANNOT_GET_ROW \
      "Cannot get row (%d). The matrix has %d number of rows.\n" \

#define INCONSITENT_ARRAY \
      "Cannot found element %d in the array (NULL). Expected a total of : %d elements.\n"  \

#define INCONSITENT_VARGS \
      "Cannot find element %d in the varargs. Expecteda total of : %d varargs.\n" \

#define CANNOT_REF_MATRIX_DEGENERATE \
      "Cannot compute REF. Matrix is degenerate or near degenerate.\n" \

#define CANNOT_OPEN_FILE "Cannot open file '%s'. Please check the path is correct and you have reading rights.\n"

#define INVALID_MATRIX_FILE \
      "Invalid matrix file: %s. Cannot read data.\n" \

#define VECTOR_J_DEGENERATE \
      "Vector on colum %d is generate or near degenerate. Cannot proceed further.\n" \

#define CANNOT_QR_NON_SQUARE \
      "We cannot QA non-square matrix[%d, %d].\n" \

#define CANNOT_COLUMN_L2NORM \
      "Cannot get column (%d). The matrix has %d numbers of columns.\n" \

#define CANNOT_VECT_DOT_DIMENSIONS \
      "The two vectors have different dimensions: %d and %d.\n" \


// Dynamically allocates a new matrix struct
nml_mat *nml_mat_new(int num_rows, int num_cols) {
    if (num_rows == 0) {
        // NML_ERROR(INVALID_ROWS);
        return NULL;
    }
    if (num_cols == 0) {
        // NML_ERROR(INVALID_ROWS);
        return NULL;
    }

    nml_mat *m = malloc(1*sizeof(*m));
    m->num_rows = num_rows;
    m->num_cols = num_cols;
    m->is_square = (num_rows == num_cols) ? 1 : 0;
    m->data = malloc(m->num_rows*sizeof(*m->data));
    for(int i = 0; i < m->num_rows; ++i) {
        m->data[i] = malloc(m->num_cols*sizeof(**m->data));
    }

    for(int i = 0; i < num_rows; i++) {
        for(int j = 0; j < num_cols; j++) {
        m->data[i][j] = 0;
        }
    }
    char* myStr = "Hello, world!";
    NML_ERROR(CANNOT_ADD);


  return m;
}

nml_mat *nml_mat_rnd(int num_rows, int num_cols, double min, double max) {
  nml_mat *r = nml_mat_new(num_rows, num_cols);
  int i, j;
  for(i = 0; i < num_rows; i++) {
    for(j = 0; j < num_cols; j++) {
      r->data[i][j] = nml_rand_interval(min, max);
    }
  }
  return r;
}

void nml_mat_print_CF(nml_mat *matrix) {
    consolePrintf("\n=========================\n\n");
    for (int i = 0; i < matrix->num_rows; i++)
    {
        for (int j = 0; j < matrix->num_cols; j++)
        {
            consolePrintf("%.5f ",(double)matrix->data[i][j]);
        }
        consolePrintf("\n");
    }
    consolePrintf("\n=========================\n\n");
}

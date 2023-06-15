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
#include "debug.h"

#define DEFAULT_VALUE 0.0


// Dynamically allocates a new matrix struct
nml_mat *nml_mat_new(int num_rows, int num_cols) {
    if (num_rows == 0) {
        NML_ERROR();
        return NULL;
    }
    if (num_cols == 0) {
        NML_ERROR();
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
    NML_ERROR();


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

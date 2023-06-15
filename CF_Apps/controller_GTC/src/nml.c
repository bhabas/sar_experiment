#include "nml.h"


// Dynamically allocates a new matrix struct
nml_mat *nml_mat_new(int num_rows, int num_cols) {

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

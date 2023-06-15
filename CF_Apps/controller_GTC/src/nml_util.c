#include "nml_util.h"

double nml_rand_interval(double min, double max) {
  double d;
  d = (double) rand() / ((double) RAND_MAX + 1);
  return (min + d * (max - min));
}

void NML_ERROR()
{
    DEBUG_PRINT("NML Error");
}

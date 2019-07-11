#include <math.h>

typedef struct Smooth_filter smooth_filter_t;

struct Smooth_filter
{
  float buffer[20];
  float weight[20];
  int len;
};

float smooth_filter(int length,float input,float weight[]);

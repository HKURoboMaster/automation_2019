#include "smooth_filter.h"

float smooth_filter(int length,float input,float weight[])
{
  static smooth_filter_t filter;
  static  int flag  = 0; //This flag indicate whether this function was called for the first time
  float output;
  if (flag == 0)
  {
    filter.len = length;
    for(int i=0;i<filter.len;i++)
    {
      filter.buffer[i] = input;
      filter.weight[i] = weight[i];
    }
    flag = 1;
		return input;
  }
  else
  {
    for(int i=0;i<filter.len-1;i++)
    {
      filter.buffer[i] = filter.buffer[i+1];
      output += filter.buffer[i]*filter.weight[i];
    }
    filter.buffer[filter.len-1] = input;
    output+=filter.buffer[filter.len-1]*filter.weight[filter.len-1];
  }
  return output;
}

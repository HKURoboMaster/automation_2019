#ifndef __CHASSIS_CALC_H__
#define __CHASSIS_CALC_H__

#ifdef CHASSIS_CALC_H_GLOBAL
  #define CHASSIS_CALC_H_EXTERN 
#else
  #define CHASSIS_CALC_H_EXTERN extern
#endif

#include "chassis_task.h"

typedef struct chassis_movement {
    int spd_ind; // speed indicator, 1 or -1
    int duration; // movement duration (ms)
    uint32_t start_time; // movement start time (ticks)
} chassis_movement_t;

float chassis_random_movement(float speed);
void generate_movement(void);
void forced_movement(int spd_ind, int duration);
void set_duration_ceiling(int new_duration_ceiling);
void set_duration_floor(int new_duration_floor);

#endif

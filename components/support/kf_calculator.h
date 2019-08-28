/****************************************************************************
 *  Copyright (C) HKU Robomaster 2018-2019 Herkules.
 *  Author: Y.H. Liu
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#ifndef __KF_CALCULATOR_H__
#define __KF_CALCULATOR_H__

#include "kalman_filter.h"

typedef struct 
{
  uint32_t tim_ms;
  int32_t speed_calc_time;
}kf_time_t;

typedef struct 
{
  float angle;
  float speed;
}input_t;

typedef struct
{
  int delay_cnt;
  int freq;
  int last_time;
  float last_position;
  float speed;
  float last_speed;
  float processed_speed;
} speed_calc_data_t;

typedef struct
{
	int delay_cnt;
	int freq;
	int last_time;
	float last_speed;
	float acceleration;
	float last_acceleration;
	float processed_acceleration;
}acc_calc_data_t;

typedef struct
{
  // private data
    kalman_filter_t _kf_body;
    kf_time_t _kf_tim;
    kf_time_t _kf_last_tim;
    input_t _raw_data;
    input_t _last_raw_data;
    speed_calc_data_t _speed_struct;
  // private method
    input_t (*_delta_input)(void *);
    kf_time_t (*_delta_time)(void *);
  // public data
    float filtered_value;
  // public method
    void (*update_current_time)(void *, uint32_t);
    void (*update_speed_time)(void *, int32_t);
    void (*update_raw_speed)(void *, float);
    void (*update_raw_angle)(void *, float);

    int32_t (*kf_calculator_init)(void *, kalman_filter_init_t *);
    int32_t (*speed_calc_data_init)(void *, speed_calc_data_t *);
    float (*speed_calculator)(void *);

    float (*filter)(void *);

    float (*get_value)(void *);
}kf_calculator_t;

int32_t kf_calculator_register(kf_calculator_t * kf_calculator_d);

#endif

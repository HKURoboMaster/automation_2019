/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
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
/** @file kalman_filter.h
 *  @version 1.0
 *  @date Apr 2018
 *
 *  @brief kalman filter realization
 *
 *  @copyright 2018 DJI RoboMaster. All rights reserved.
 *
 */
 
#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

#include "stm32f4xx_hal.h"
#include "arm_math.h"

#define mat         arm_matrix_instance_f32 
#define mat_64      arm_matrix_instance_f64
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32
#define mat_inv     arm_mat_inverse_f32
#define mat_inv_f64 arm_mat_inverse_f64



typedef struct
{
  float raw_value;
  float filtered_value[2];
  mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalman_filter_t;
// Edited By Ericcsr 3-d kalman

typedef struct
{
  float raw_value;
  float filtered_value[3];
  mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalman_filter_3d_t;


typedef struct
{
  float raw_value;
  float filtered_value[3];
  float xhat_data[3], xhatminus_data[3], z_data[3],Pminus_data[9], K_data[9];
  float P_data[9];
  float AT_data[9], HT_data[9];
  float A_data[9];
  float H_data[9];
  float Q_data[9];
  float R_data[9];
} kalman_filter_3d_init_t;

typedef struct
{
  float raw_value;
  float filtered_value[2];
  float xhat_data[2], xhatminus_data[2], z_data[2],Pminus_data[4], K_data[4];
  float P_data[4];
  float AT_data[4], HT_data[4];
  float A_data[4];
  float H_data[4];
  float Q_data[4];
  float R_data[4];
} kalman_filter_init_t;


void   kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2);
void   kalman_filter_3d_init(kalman_filter_3d_t *F, kalman_filter_3d_init_t *I);
float *kalman_filter_3d_calc(kalman_filter_3d_t *F, float signal1,float signal2, float signal3);











#endif

/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
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

#ifndef __RAMP_H__
#define __RAMP_H__

#ifdef RAMP_H_GLOBAL
  #define RAMP_H_EXTERN 
#else
  #define RAMP_H_EXTERN extern
#endif

#include "stdint.h"

typedef struct ramp_t
{
  int32_t count;
  int32_t scale;
  float   out;
  void  (*init)(struct ramp_t *ramp, int32_t scale);
  float (*calc)(struct ramp_t *ramp);
}ramp_t;

#define RAMP_GEN_DAFAULT     \
  {                          \
    .count = 0,              \
    .scale = 0,              \
    .out = 0,                \
    .init = &ramp_init,      \
    .calc = &ramp_calculate, \
  }

void  ramp_init(ramp_t *ramp, int32_t scale);
float ramp_calculate(ramp_t *ramp);

/**Added by Y.H. Liu
 * @Jul 7, 2019: definantion for chassis ramp
 * 
 * For chassis ramp
 */
#define RAMP_CHASSIS_CO 8.95924f
float chassis_ramp(float v_ref, float mec_ref);

#endif // __RAMP_H__


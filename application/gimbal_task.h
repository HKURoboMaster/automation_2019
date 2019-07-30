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

#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#ifdef GIMBAL_TASK_H_GLOBAL
  #define GIMBAL_TASK_H_EXTERN 
#else
  #define GIMBAL_TASK_H_EXTERN extern
#endif

#include "sys.h"
  
void gimbal_task(void const * argument);
void gimbal_auto_adjust_start(void);
uint8_t get_gimbal_init_state(void);
void gimbal_init_state_reset(void);

#define RC_CH_SCALE        660
#define GIMBAL_RC_PITCH    0.0008f
#define GIMBAL_RC_YAW      0.0007f
#define GIMBAL_MOUSE_PITCH 0.0015f
#define GIMBAL_MOUSE_YAW   0.0020f

#endif // __GIMBAL_TASK_H__

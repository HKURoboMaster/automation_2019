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

#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__
// #define HERO_ROBOT

#ifdef CHASSIS_TASK_H_GLOBAL
  #define CHASSIS_TASK_H_EXTERN 
#else
  #define CHASSIS_TASK_H_EXTERN extern
#endif

#include "chassis.h"

void chassis_task(void const * argument);
int32_t chassis_set_relative_angle(float angle);

struct chassis_power
{
	int current_debug;
	int voltage_debug;
	float current;
	float voltage;
	float power;
};
//Edited by Eric Chen 
int get_chassis_power(struct chassis_power *chassis_power); 

//Eric Chen Edition End
#define RC_CH_SCALE 660
#define CHASSIS_NETURAL_TH 10

#define CHASSIS_POWER_TH  75 //80-5, for protection
#define LOW_BUFFER  10
#define LOW_VOLTAGE   19
#define WORKING_VOLTAGE 24
#define NO_BUFFER_TIME_TH 3000
#define CHASSIS_POWER_CTRL

#ifdef HERO_ROBOT
#define DODGING_TH 25.0f
#endif
#ifdef CHASSIS_POWER_CTRL
#define MOTOR_TORQUE_CURRENT_CO 0.1F
#define SUPER_CAP_HOLDING_TIME 300u
#define NO_MOVEMENT_TH 30
#endif

#endif // __CHASSIS_TASK_H__

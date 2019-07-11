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

#ifdef CHASSIS_TASK_H_GLOBAL
  #define CHASSIS_TASK_H_EXTERN 
#else
  #define CHASSIS_TASK_H_EXTERN extern
#endif

#include "chassis.h"

#define IDLE_CONSTANT_SPEED 500  //speed of idle state
#define NORMAL_CONSTANT_SPEED 1000 //speed of normal state
#define BOOST_CONSTANT_SPEED 3000  //speed of boost state

typedef enum chassis_state_name {
  IDLE_STATE, NORMAL_STATE, BOOST_STATE
} chassis_state_name_t;

//enemy static state detection from tx2
typedef enum cv_static_event {
  ENEMY_NOT_DETECTED,  //no enemy detected
  ENEMY_DETECTED_LEFT,  //enemy detected on the left
  ENEMY_DETECTED_RIGHT   //enemy detected on the right
} cv_static_event_t;

//enemy dynamic state detection from tx2
typedef enum cv_dynamic_event {
  ENEMY_STAY_STILL,  //enemy has no tendency to move
  ENEMY_LEFT_TO_RIGHT,  //enemy moves from left to right
  ENEMY_RIGHT_TO_LEFT  //enemy moves from right to left
} cv_dynamic_event_t;

//power information from pc
typedef enum power_event {
  POWER_NORMAL, //no or little use of buffer power
  POWER_IN_DEBT,  //noticeable use of buffer power
  POWER_IN_SERIOUS_DEBT //considerable use of buffer power
} power_event_t;

//armor information from pc
typedef enum armor_event {
  NO_HIT_FOR_THREE_SEC, //armor has not been hit for at least 3 seconds
  HIT_WITHIN_THREE_SEC //armor has been hit within 3 seconds
} armor_event_t;

typedef struct chassis_state_t {
  chassis_state_name_t state_name;
  float constant_spd;
} chassis_state_t;

void set_state(chassis_state_t * state, chassis_state_name_t dest_state);
chassis_state_name_t get_state(const chassis_state_t * state);
float get_spd(const chassis_state_t * state);

void chassis_task(void const * argument);
int32_t chassis_set_relative_angle(float angle);
float direction_control(float v);
void check_ir_signal(void);

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

#define CHASSIS_POWER_TH  80
#define LOW_BUFFER  10
#define LOW_VOLTAGE   16
#define WORKING_VOLTAGE 21
#define NO_BUFFER_TIME_TH 3000
//#define CHASSIS_POWER_CTRL

#endif // __CHASSIS_TASK_H__

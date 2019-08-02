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

#ifndef __SHOOT_H__
#define __SHOOT_H__

#ifdef SHOOT_H_GLOBAL
#define SHOOT_H_EXTERN
#else
#define SHOOT_H_EXTERN extern
#endif

#include "motor.h"
#include "pid_controller.h"

#define SHOOT_STOP_CMD (0u)
#define SHOOT_ONCE_CMD (1u)
#define SHOOT_CONTINUOUS_CMD (2u)

#define FRIC_STOP_SPEED 100u
#define FRIC_MAX_SPEED 149u //MAX=200u, for safety set it to be 160 //160 results in overheater
#define FRIC_CON_SPEED 137u //140u
#define FRIC_MIN_SPEED 99u

#define BLOCK_CURRENT_DEFAULT 9500.0F
#define BLOCK_SPEED_DEFAULT -1650
#define BLOCK_TIMEOUT_DEFAULT 200
#define BLOCK_CHECK_TIMEOUT_DEFAULT 80
#define TURN_SPEED_DEFAULT 2700


enum trigger_state
{
  TRIG_PRESS_DOWN = 0,
  TRIG_BOUNCE_UP = 1,
};

enum shoot_state
{
  SHOOT_INIT = 0,
  SHOOT_READY,
  SHOOT_BLOCK,
  SHOOT_RUNNING
};

struct shoot_param
{
  float turn_speed;
  float block_speed;
  float block_timeout;
  float check_timeout;
  float block_current;
};

struct shoot_target
{
  uint32_t shoot_num;
  uint16_t fric_spd[2];
  float motor_speed;
};

typedef struct shoot *shoot_t;

struct shoot
{
  struct object parent;
  struct shoot_param param;

  enum shoot_state state;

  uint8_t cmd;

  uint8_t trigger_key;
  float fric_spd[2];

  uint32_t shoot_num;
  uint32_t block_time;

  struct shoot_target target;

  struct motor_device motor;
  struct pid motor_pid;
  struct pid_feedback motor_feedback;
  struct controller ctrl;
};

shoot_t shoot_find(const char *name);
int32_t shoot_pid_register(struct shoot *shoot, const char *name, enum device_can can);
int32_t shoot_pid_register2(struct shoot *shoot, const char *name, enum device_can can);
int32_t shoot_set_fric_speed(struct shoot *shoot, uint16_t fric_spd1, uint16_t fric_spd2);
int32_t shoot_get_fric_speed(struct shoot *shoot, float *fric_spd1, float *fric_spd2);
int32_t shoot_set_cmd(struct shoot *shoot, uint8_t cmd, uint32_t shoot_num);
int32_t shoot_execute(struct shoot *shoot);
int32_t shoot_state_update(struct shoot *shoot);
int32_t shoot_enable(struct shoot *shoot);
int32_t shoot_disable(struct shoot *shoot);
int32_t shoot_set_turn_speed(struct shoot *shoot, uint16_t speed);
int32_t laser_cmd(uint8_t cmd);
int32_t magazine_lid_cmd(uint8_t cmd);

#define open_lid()        magazine_lid_cmd(1u)
#define close_lid()       magazine_lid_cmd(0u)
#define turn_on_laser()   laser_cmd(1u)
#define turn_off_laser()  laser_cmd(0u)

#endif // __SHOOT_H__

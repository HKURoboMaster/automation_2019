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

#ifndef __INFANTRY_H__
#define __INFANTRY_H__

#ifdef INFANTRY_H_GLOBAL
  #define INFANTRY_H_EXTERN 
#else
  #define INFANTRY_H_EXTERN extern
#endif

#include "sys.h"

#define FIRMWARE_VERSION_0 6u
#define FIRMWARE_VERSION_1 1u
#define FIRMWARE_VERSION_2 0u
#define FIRMWARE_VERSION_3 1u

#define FIRMWARE_VERSION ((FIRMWARE_VERSION_3 << 24) | (FIRMWARE_VERSION_2 << 16) | (FIRMWARE_VERSION_1 << 8) | FIRMWARE_VERSION_0)

#define MANIFOLD2_ADDRESS      0x00
#define CHASSIS_ADDRESS        0x01
#define UPPER_ADDRESS         0x02

/* cmd id can not be set 0xffff */

#define CMD_MANIFOLD2_HEART                 (0x0001u)
#define CMD_REPORT_VERSION                  (0x0002u)

#define CMD_STUDENT_DATA                    (0x0101u)

#define CMD_PUSH_CHASSIS_INFO               (0x0201u)
#define CMD_SET_CHASSIS_MODE                (0x0202u)
#define CMD_SET_CHASSIS_SPEED               (0x0203u)
#define CMD_GET_CHASSIS_PARAM               (0x0204u)
#define CMD_SET_CHASSIS_SPD_ACC             (0x0205u)

#define CMD_UPPER_CTRL											(0x0206u)

#define CMD_RC_DATA_FORWORD                 (0x0401u)
#define CMD_PUSH_UWB_INFO                   (0x0402u)

#pragma pack(push,1)

struct cmd_chassis_param
{
  uint16_t wheel_perimeter; /* the perimeter(mm) of wheel */
  uint16_t wheel_track;     /* wheel track distance(mm) */
  uint16_t wheel_base;      /* wheelbase distance(mm) */
  int16_t rotate_x_offset;
  int16_t rotate_y_offset;
};

struct cmd_chassis_info
{
  int16_t gyro_angle;
  int16_t gyro_palstance;
  int32_t position_x_mm;
  int32_t position_y_mm;
  int16_t angle_deg;
  int16_t v_x_mm;
  int16_t v_y_mm;
};

struct cmd_upper_info
{
  uint8_t   mode;
};

struct cmd_chassis_speed
{
  int16_t vx; // forward/back
  int16_t vy; // left/right
  int16_t vw; // anticlockwise/clockwise
  int16_t rotate_x_offset;
  int16_t rotate_y_offset;
};

struct cmd_chassis_spd_acc
{
  int16_t   vx; 
  int16_t   vy;
  int16_t   vw; 

  int16_t   ax; 
  int16_t   ay; 
  int16_t   wz; 
  
  int16_t rotate_x_offset;
  int16_t rotate_y_offset;
};

#pragma pack(pop)

struct manifold_cmd
{
  struct cmd_chassis_speed chassis_speed;
  struct cmd_chassis_spd_acc chassis_spd_acc;
};

struct upper_cmd
{
	struct cmd_upper_info upper_info;
};

int32_t rc_data_forword_by_can(uint8_t *buff, uint16_t len);
void infantry_cmd_task(void const * argument);
int32_t chassis_push_info(void *argc);
int32_t upper_push_info(void *argc);
struct manifold_cmd *get_manifold_cmd(void);

#endif // __INFANTRY_H__

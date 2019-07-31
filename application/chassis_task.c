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

#include "dbus.h"
#include "chassis_task.h"
#include "timer_task.h"
#include "infantry_cmd.h"
#include "ahrs.h"
#include "drv_imu.h"
#include "engineer.h"

static float vx, vy, wz;

float follow_relative_angle;
struct pid pid_follow = {0}; //angle control
static void chassis_imu_update(void *argc);

extern Engineer engg;

void chassis_task(void const *argument)
{
  uint32_t period = osKernelSysTick();
  chassis_t pchassis = NULL;
  rc_device_t prc_dev = NULL;
  rc_info_t prc_info = NULL;
  pchassis = chassis_find("chassis");
  prc_dev = rc_device_find("uart_rc");

  if (prc_dev != NULL)
  {
    prc_info = rc_device_get_info(prc_dev);
  }
  else
  {
  }

  soft_timer_register(chassis_push_info, (void *)pchassis, 10);

  pid_struct_init(&pid_follow, MAX_CHASSIS_VW_SPEED, 50, 8.0f, 0.0f, 2.0f);

  while (1)
  {
		if (engg.HALT_CHASSIS) {
			chassis_set_speed(pchassis, 0, 0, 0);
		}
    else if (rc_device_get_state(prc_dev, RC_S1_DOWN) != RM_OK && rc_device_get_state(prc_dev, RC_S2_DOWN) != RM_OK) {
			
			int chassis_direction = 1;
			int flip_ctrl = 0;
			
			if (engg.ENGINEER_BIG_STATE == LOWERPART && engg.ENGINEER_SMALL_STATE == REVERSE_CHASSIS)
				chassis_direction = -1;
			else if (engg.ENGINEER_BIG_STATE == UPPERPART)
				flip_ctrl = 1;
			
      if (rc_device_get_state(prc_dev, RC_S2_MID) == RM_OK || rc_device_get_state(prc_dev, RC_S2_UP) == RM_OK)
      {
				if (flip_ctrl) {
					vx = (float)prc_info->ch2 / 660 * MAX_CHASSIS_VX_SPEED * chassis_direction;
					vy = -(float)prc_info->ch1 / 660 * MAX_CHASSIS_VY_SPEED * chassis_direction;
					wz = -(float)prc_info->ch3 / 660 * MAX_CHASSIS_VW_SPEED;
					chassis_set_offset(pchassis, 0, 0);
					chassis_set_speed(pchassis, -vy, vx, wz);
				}
				else {
					vx = (float)prc_info->ch2 / 660 * MAX_CHASSIS_VX_SPEED * chassis_direction;
					vy = -(float)prc_info->ch1 / 660 * MAX_CHASSIS_VY_SPEED * chassis_direction;
					wz = -(float)prc_info->ch3 / 660 * MAX_CHASSIS_VW_SPEED;
					chassis_set_offset(pchassis, 0, 0);
					chassis_set_speed(pchassis, vx, vy, wz);
				}
      }
			
      chassis_set_acc(pchassis, 0, 0, 0);
    }

    chassis_imu_update(pchassis);
    chassis_execute(pchassis);
    osDelayUntil(&period, 2);
  }
}

#define RAD_TO_DEG 57.3f
static void chassis_imu_update(void *argc)
{
  struct ahrs_sensor mpu_sensor;
  struct attitude mahony_atti;
  chassis_t pchassis = (chassis_t)argc;
  mpu_get_data(&mpu_sensor);
  mahony_ahrs_updateIMU(&mpu_sensor, &mahony_atti);
  chassis_gyro_update(pchassis, -mahony_atti.yaw, mpu_sensor.wz * RAD_TO_DEG);
}

int32_t chassis_set_relative_angle(float angle)
{
  follow_relative_angle = angle;
  return 0;
}

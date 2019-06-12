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

#include <math.h>
#define PI 3.142f

static float vx, vy, wz;

float follow_relative_angle;
struct pid pid_follow = {0}; //angle control
static void chassis_imu_update(void *argc);

/** Edited by Y.H. Liu
  * @Jun 12, 2019: modified the mode switch
  *
  * Implement the customized control logic and FSM, details in Control.md
*/
#define km_dodge          prc_info->kb.bit.C == 1
#define back_to_netural   follow_relative_angle < 10 && follow_relative_angle >= -10

uint8_t dodging = 0;
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
    if (rc_device_get_state(prc_dev, RC_S2_UP) == RM_OK || rc_device_get_state(prc_dev, RC_S2_MID) == RM_OK)
    { //not disabled

      float temp_vx = (float)prc_info->ch2 / 660 * MAX_CHASSIS_VX_SPEED;
      float temp_vy = -(float)prc_info->ch1 / 660 * MAX_CHASSIS_VY_SPEED;
      vx = temp_vx * cos(PI * follow_relative_angle / 180) - temp_vy * sin(PI * follow_relative_angle / 180);
      vy = temp_vx * sin(PI * follow_relative_angle / 180) + temp_vy * cos(PI * follow_relative_angle / 180);

      if(km_dodge || (dodging && !back_to_netural))
      {
        wz = 3 * MAX_CHASSIS_VW_SPEED / 5;
        dodging |= 1;
      }
      else
      {
        wz  = pid_calculate(&pid_follow, follow_relative_angle, 0);
        dodging &= 0;
      }

      chassis_set_offset(pchassis, ROTATE_X_OFFSET, ROTATE_Y_OFFSET);
      chassis_set_speed(pchassis, vx, vy, wz);
    }
    else
    {
      chassis_set_speed(pchassis, 0, 0, 0);
    }

    chassis_set_acc(pchassis, 0, 0, 0);

    /*
    if (rc_device_get_state(prc_dev, RC_S2_DOWN) != RM_OK)
    {
      if (rc_device_get_state(prc_dev, RC_S2_UP) == RM_OK)
      {
        vx = (float)prc_info->ch2 / 660 * MAX_CHASSIS_VX_SPEED;
        vy = -(float)prc_info->ch1 / 660 * MAX_CHASSIS_VY_SPEED;
        wz = -pid_calculate(&pid_follow, follow_relative_angle, 0);
        chassis_set_offset(pchassis, ROTATE_X_OFFSET, ROTATE_Y_OFFSET);
        chassis_set_speed(pchassis, vx, vy, wz);
      }

      if (rc_device_get_state(prc_dev, RC_S2_MID) == RM_OK)
      {
        vx = (float)prc_info->ch2 / 660 * MAX_CHASSIS_VX_SPEED;
        vy = -(float)prc_info->ch1 / 660 * MAX_CHASSIS_VY_SPEED;
        wz = -(float)prc_info->ch3 / 660 * MAX_CHASSIS_VW_SPEED;
        chassis_set_offset(pchassis, 0, 0);
        chassis_set_speed(pchassis, vx, vy, wz);
      }

      if (rc_device_get_state(prc_dev, RC_S2_MID2DOWN) == RM_OK)
      {
        chassis_set_speed(pchassis, 0, 0, 0);
      }

      if (rc_device_get_state(prc_dev, RC_S2_MID2UP) == RM_OK)
      {
        chassis_set_speed(pchassis, 0, 0, 0);
      }

      chassis_set_acc(pchassis, 0, 0, 0);
    }
  */

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
  // TODO: adapt coordinates to our own design
  chassis_gyro_update(pchassis, -mahony_atti.yaw, mpu_sensor.wz * RAD_TO_DEG);
  // TODO: adapt coordinates to our own design
}

int32_t chassis_set_relative_angle(float angle)
{
  follow_relative_angle = angle;
  return 0;
}

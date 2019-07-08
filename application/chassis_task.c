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

static float vx, vy, wz;

float follow_relative_angle;
struct pid pid_follow = {0}; //angle control
static void chassis_imu_update(void *argc);

int left_blocked = 0, right_blocked = 0;  //IR detects if left or right side is blocked
int left_ir_js = 0, right_ir_js = 0;  //for jscope
int using_rm = USING_RM;  //if using a remote control, if in use then 1

//=======chassis movement logic global var========
chassis_state_t state; //The state of chassis
//set_state(&state, LAZY_STATE);
cv_dynamic_event_t dynamic_eve = ENEMY_STAY_STILL;
cv_static_event_t static_eve = ENEMY_NOT_DETECTED;
power_event_t power_eve = POWER_NORMAL;
armor_event_t armor_eve = NO_HIT_FOR_THREE_SEC;

float direction_control(float v) {
  float res_v;
  if (left_blocked) {
    res_v = (v < 0 ? 0 : v);
  }
  if (right_blocked) {
    res_v = (v > 0 ? 0 : v);
  }
  return res_v;
}

void check_ir_signal() {
  left_blocked = (HAL_GPIO_ReadPin(IR_LEFT_Port, IR_LEFT_Pin) == GPIO_PIN_RESET);
  right_blocked = (HAL_GPIO_ReadPin(IR_RIGHT_Port, IR_RIGHT_Pin) == GPIO_PIN_RESET);
  left_ir_js = left_blocked ? 5000 : 0;
  right_ir_js = right_blocked ? -5000 : 0;
}

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
    check_ir_signal();
    if (rc_device_get_state(prc_dev, RC_S2_DOWN) != RM_OK)
    {
      if (rc_device_get_state(prc_dev, RC_S2_UP) == RM_OK)
      {
        vx = (float)prc_info->ch2 / 660 * MAX_CHASSIS_VX_SPEED;
        vy = -(float)prc_info->ch1 / 660 * MAX_CHASSIS_VY_SPEED;
        wz = -pid_calculate(&pid_follow, follow_relative_angle, 0);
        chassis_set_offset(pchassis, ROTATE_X_OFFSET, ROTATE_Y_OFFSET);
        vy = direction_control(vy);
        chassis_set_speed(pchassis, 0, vy, 0);
      }

      if (rc_device_get_state(prc_dev, RC_S2_MID) == RM_OK)
      {
        vx = (float)prc_info->ch2 / 660 * MAX_CHASSIS_VX_SPEED;
        vy = -(float)prc_info->ch1 / 660 * MAX_CHASSIS_VY_SPEED;
        wz = -(float)prc_info->ch3 / 660 * MAX_CHASSIS_VW_SPEED;
        chassis_set_offset(pchassis, 0, 0);
        vy = direction_control(vy);
        chassis_set_speed(pchassis, 0, vy, 0);
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

void set_state(chassis_state_t * state, chassis_state_name_t dest_state) {
  state->state_name = dest_state;
  switch (dest_state) {
    case LAZY_STATE:
      state->constant_spd = LAZY_CONSTANT_SPEED;
      break;
    case NORMAL_STATE:
      state->constant_spd = NORMAL_CONSTANT_SPEED;
      break;
    case BOOST_STATE:
      state->constant_spd = BOOST_CONSTANT_SPEED;
  }
}

chassis_state_name_t get_state(const chassis_state_t * state) {
  return state->state_name;
}

float get_spd(const chassis_state_t * state) {
  return state->constant_spd;
}
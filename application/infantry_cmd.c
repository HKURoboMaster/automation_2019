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

#include "board.h"
#include "dbus.h"
#include "chassis.h"

#include "init.h"
#include "infantry_cmd.h"
#include "chassis_task.h"

#include "protocol.h"
#include "referee_system.h"

#define MANIFOLD2_CHASSIS_SIGNAL (1 << 0)
#define MANIFOLD2_CHASSIS_ACC_SIGNAL (1 << 4)

extern osThreadId cmd_task_t;

struct cmd_chassis_info cmd_chassis_info;
struct manifold_cmd manifold_cmd;

struct manifold_cmd *get_manifold_cmd(void)
{
  return &manifold_cmd;
}

int32_t chassis_speed_ctrl(uint8_t *buff, uint16_t len);
int32_t chassis_spd_acc_ctrl(uint8_t *buff, uint16_t len);
int32_t student_data_transmit(uint8_t *buff, uint16_t len);

int32_t rc_data_forword_by_can(uint8_t *buff, uint16_t len)
{
  return 0;
}

void infantry_cmd_task(void const *argument)
{
  uint8_t app;
  osEvent event;
  app = get_sys_cfg();

  rc_device_t prc_dev = NULL;
  chassis_t pchassis = NULL;

  pchassis = chassis_find("chassis");

  if (app == CHASSIS_APP)
  {
    prc_dev = rc_device_find("uart_rc");
    protocol_rcv_cmd_register(CMD_STUDENT_DATA, student_data_transmit);
    protocol_rcv_cmd_register(CMD_SET_CHASSIS_SPEED, chassis_speed_ctrl);
    protocol_rcv_cmd_register(CMD_SET_CHASSIS_SPD_ACC, chassis_spd_acc_ctrl);
  }
  else
  {
    prc_dev = rc_device_find("can_rc");
  }

  while (1)
  {
    if (rc_device_get_state(prc_dev, RC_S2_DOWN) != RM_OK)
    {
      memset(&manifold_cmd, 0, sizeof(struct manifold_cmd));
      osDelay(100);
    }
    else
    {
      event = osSignalWait(MANIFOLD2_CHASSIS_SIGNAL |
                              MANIFOLD2_CHASSIS_ACC_SIGNAL,
                           500);

      if (event.status == osEventSignal)
      {
        if (event.value.signals & MANIFOLD2_CHASSIS_SIGNAL)
        {
          struct cmd_chassis_speed *pspeed;
          pspeed = &manifold_cmd.chassis_speed;
          chassis_set_offset(pchassis, pspeed->rotate_x_offset, pspeed->rotate_x_offset);
          chassis_set_acc(pchassis, 0, 0, 0);
          chassis_set_speed(pchassis, pspeed->vx, pspeed->vy, pspeed->vw / 10.0f);
        }

        if (event.value.signals & MANIFOLD2_CHASSIS_ACC_SIGNAL)
        {
          struct cmd_chassis_spd_acc *pacc;
          pacc = &manifold_cmd.chassis_spd_acc;
          chassis_set_offset(pchassis, pacc->rotate_x_offset, pacc->rotate_x_offset);
          chassis_set_acc(pchassis, pacc->ax, pacc->ay, pacc->wz / 10.0f);
          chassis_set_speed(pchassis, pacc->vx, pacc->vy, pacc->vw / 10.0f);
        }
      }
      else
      {
        chassis_set_speed(pchassis, 0, 0, 0);
        chassis_set_acc(pchassis, 0, 0, 0);
      }
    }
  }
}

int32_t student_data_transmit(uint8_t *buff, uint16_t len)
{
  uint16_t cmd_id = *(uint16_t *)buff;
  referee_protocol_tansmit(cmd_id, buff + 2, len - 2);
  return 0;
}

int32_t chassis_speed_ctrl(uint8_t *buff, uint16_t len)
{
  if (len == sizeof(struct cmd_chassis_speed))
  {
    memcpy(&manifold_cmd.chassis_speed, buff, len);
    osSignalSet(cmd_task_t, MANIFOLD2_CHASSIS_SIGNAL);
  }
  return 0;
}

int32_t chassis_spd_acc_ctrl(uint8_t *buff, uint16_t len)
{
  if (len == sizeof(struct cmd_chassis_spd_acc))
  {
    memcpy(&manifold_cmd.chassis_spd_acc, buff, len);
    osSignalSet(cmd_task_t, MANIFOLD2_CHASSIS_ACC_SIGNAL);
  }
  return 0;
}

int32_t chassis_push_info(void *argc)
{
  struct chassis_info info;
  chassis_t pchassis = (chassis_t)argc;
  chassis_get_info(pchassis, &info);

  cmd_chassis_info.angle_deg = info.angle_deg * 10;
  cmd_chassis_info.gyro_angle = info.yaw_gyro_angle * 10;
  cmd_chassis_info.gyro_palstance = info.yaw_gyro_rate * 10;
  cmd_chassis_info.position_x_mm = info.position_x_mm;
  cmd_chassis_info.position_y_mm = info.position_y_mm;
  cmd_chassis_info.v_x_mm = info.v_x_mm;
  cmd_chassis_info.v_y_mm = info.v_y_mm;

  protocol_send(MANIFOLD2_ADDRESS, CMD_PUSH_CHASSIS_INFO, &cmd_chassis_info, sizeof(cmd_chassis_info));

  return 0;
}

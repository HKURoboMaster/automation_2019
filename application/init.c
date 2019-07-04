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

#include "main.h"
#include "can.h"
#include "board.h"
#include "motor.h"
#include "dbus.h"
#include "detect.h"
#include "test.h"
#include "chassis.h"

#include "chassis_task.h"
#include "timer_task.h"
#include "communicate.h"
#include "infantry_cmd.h"
#include "init.h"

#include "protocol.h"
#include "ulog.h"
#include "param.h"
#include "offline_check.h"
#include "referee_system.h"

struct chassis chassis;
static struct rc_device rc_dev;

static uint8_t glb_sys_cfg;

extern int ulog_console_backend_init(void);

void system_config(void)
{
  glb_sys_cfg = HAL_GPIO_ReadPin(SYS_CFG_Port, SYS_CFG_Pin);
}

uint8_t get_sys_cfg(void)
{
  return glb_sys_cfg;
}

void hw_init(void)
{
  cali_param_init();
  board_config();
  test_init();
  system_config();
  ulog_init();
  ulog_console_backend_init();
  
  referee_param_init();
  usart3_rx_callback_register(referee_uart_rx_data_handle);
  referee_send_data_register(usart3_transmit);

  if(glb_sys_cfg == CHASSIS_APP)
  {
    rc_device_register(&rc_dev, "uart_rc", 0);
    dr16_forword_callback_register(rc_data_forword_by_can);
    chassis_pid_register(&chassis, "chassis", DEVICE_CAN1);
    chassis_disable(&chassis);
  }
  else
  {
    rc_device_register(&rc_dev, "can_rc", 0);
  }

  offline_init();
}

osThreadId timer_task_t;
osThreadId chassis_task_t;
osThreadId communicate_task_t;
osThreadId cmd_task_t;

void task_init(void)
{
  uint8_t app;
  app = get_sys_cfg();

  osThreadDef(TIMER_1MS, timer_task, osPriorityHigh, 0, 512);
  timer_task_t = osThreadCreate(osThread(TIMER_1MS), NULL);

  osThreadDef(COMMUNICATE_TASK, communicate_task, osPriorityHigh, 0, 4096);
  communicate_task_t = osThreadCreate(osThread(COMMUNICATE_TASK), NULL);

  osThreadDef(CMD_TASK, infantry_cmd_task, osPriorityNormal, 0, 4096);
  cmd_task_t = osThreadCreate(osThread(CMD_TASK), NULL);
  
  if (app == CHASSIS_APP)
  {
    osThreadDef(CHASSIS_TASK, chassis_task, osPriorityRealtime, 0, 512);
    chassis_task_t = osThreadCreate(osThread(CHASSIS_TASK), NULL);
  }
  else
  {
		
  }
}

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

#include "can.h"
#include "board.h"
#include "detect.h"
#include "chassis.h"
#include "init.h"
#include "offline_check.h"
#include "timer_task.h"
#include "infantry_cmd.h"
#include "dualmotor.h"
#include "moonrover.h"

#define BEEP_MAX_TIMES 20

static struct detect_device offline_dev;
static uint8_t offline_beep_times[BEEP_MAX_TIMES];
static int32_t offline_beep_set_times(void *argc);
static int32_t rc_offline_callback(void *argc);

struct detect_device *get_offline_dev(void)
{
  return &offline_dev;
}

static chassis_t pchassis = NULL;
extern Engineer engg;
extern upper_ctrl upper_controller;

void offline_init(void)
{
  uint8_t app;
  app = get_sys_cfg();

  for (int i = 0; i < BEEP_MAX_TIMES; i++)
  {
    offline_beep_times[i] = i;
  }

  pchassis = chassis_find("chassis");

  detect_device_register(&offline_dev, "detect", 0, 0);

  detect_device_add_event(&offline_dev, RC_OFFLINE_EVENT, 100, rc_offline_callback, NULL);

  if (app == CHASSIS_APP)
  {
    detect_device_add_event(&offline_dev, MOTOR1_OFFLINE_EVENT, 100, offline_beep_set_times, &offline_beep_times[1]);
    detect_device_add_event(&offline_dev, MOTOR2_OFFLINE_EVENT, 100, offline_beep_set_times, &offline_beep_times[2]);
    detect_device_add_event(&offline_dev, MOTOR3_OFFLINE_EVENT, 100, offline_beep_set_times, &offline_beep_times[3]);
    detect_device_add_event(&offline_dev, MOTOR4_OFFLINE_EVENT, 100, offline_beep_set_times, &offline_beep_times[4]);
		detect_device_add_event(&offline_dev, MOONROVER1_OFFLINE_EVENT, 100, offline_beep_set_times, &offline_beep_times[7]);
		detect_device_add_event(&offline_dev, MOONROVER2_OFFLINE_EVENT, 100, offline_beep_set_times, &offline_beep_times[8]);
  }
	else if (app == UPPER_APP)
	{
		detect_device_add_event(&offline_dev, DUALMOTOR1_OFFLINE_EVENT, 100, offline_beep_set_times, &offline_beep_times[5]);
		detect_device_add_event(&offline_dev, DUALMOTOR2_OFFLINE_EVENT, 100, offline_beep_set_times, &offline_beep_times[6]);
	}
 
  soft_timer_register(offline_check, NULL, 20);
  can_fifo0_rx_callback_register(&can1_manage, can1_detect_update);
  can_fifo0_rx_callback_register(&can2_manage, can2_detect_update);
}

int32_t offline_check(void *argc)
{
  detect_device_check(&offline_dev, 0xffffffff);
  if (detect_device_get_event(&offline_dev) == 0)
  {
    offline_beep_set_times(&offline_beep_times[0]);

    chassis_enable(pchassis);
		dualmotor_enable(&upper_controller);
		moonrover_enable(&engg);
		
		LED_R_OFF();
  }
  else
  {
    chassis_disable(pchassis);
		dualmotor_disable(&upper_controller);
		moonrover_disable(&engg);
  }
  return 0;
}

int32_t get_offline_state(void)
{
  return detect_device_get_event(&offline_dev);
}

int32_t rc_offline_callback(void *argc)
{
  beep_set_times(0);
	LED_R_ON();
  return 0;
}

int32_t offline_beep_set_times(void *argc)
{
  return beep_set_times(*(uint8_t *)argc);
}

int32_t can1_detect_update(CAN_RxHeaderTypeDef *header, uint8_t *rx_data)
{
  switch (header->StdId)
  {
  case 0x201:
    detect_device_update(&offline_dev, MOTOR1_OFFLINE_EVENT);
    break;
  case 0x202:
    detect_device_update(&offline_dev, MOTOR2_OFFLINE_EVENT);
    break;
  case 0x203:
    detect_device_update(&offline_dev, MOTOR3_OFFLINE_EVENT);
    break;
  case 0x204:
    detect_device_update(&offline_dev, MOTOR4_OFFLINE_EVENT);
    break;
  case 0x205:
		detect_device_update(&offline_dev, DUALMOTOR1_OFFLINE_EVENT);
    break;
  case 0x206:
		detect_device_update(&offline_dev, DUALMOTOR2_OFFLINE_EVENT);
    break;
  case 0x207:
    detect_device_update(&offline_dev, MOONROVER1_OFFLINE_EVENT);
    break;
	case 0x208:
    detect_device_update(&offline_dev, MOONROVER2_OFFLINE_EVENT);
    break;
  default:
    break;
  }

  return header->DLC;
}

int32_t can2_detect_update(CAN_RxHeaderTypeDef *header, uint8_t *rx_data)
{
  switch (header->StdId)
  {
  default:
    break;
  }

  return header->DLC;
}

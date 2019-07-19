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

#include "sys.h"
#include "shoot.h"
#include "dbus.h"
#include "shoot_task.h"
#include "referee_system.h"
#include "infantry_cmd.h"

int32_t shoot_firction_toggle(shoot_t pshoot, uint8_t toggled);
int32_t shoot_lid_toggle(shoot_t pshoot, uint8_t toggled);

enum shoot_state shoot_state_watch;
uint8_t shoot_cmd_watch;
uint8_t trigger_state_watch;
int32_t trigger_wheel_watch;

/**Edited by Y.H. Liu
 * @Jun 13, 2019: change the FSM for shooting
 * @Jun 20, 2019: adaption for hero
 * @Jul 3, 2019: retrieve the heat data from refree system
 * @Jul 7, 2019: modify the adaption for hero
 * @Jun 19, 2019: receive the shooter heat data from the referee system
 * 
 * Implement the control logic described in Control.md
 */
enum mouse_cmd{non, click, press};
typedef enum mouse_cmd mouse_cmd_e;
mouse_cmd_e mouse_shoot_control(rc_device_t rc_dev);
static uint16_t get_heat_limit(void);

void shoot_task(void const *argument)
{
  uint32_t period = osKernelSysTick();
  rc_device_t prc_dev = NULL;

  shoot_t pshoot = NULL;
	shoot_t pshoot2 = NULL;
  pshoot = shoot_find("shoot");
	pshoot2 = shoot_find("shoot2");//Leo
  prc_dev = rc_device_find("can_rc");

  if (prc_dev == NULL)
  {
  }

  //uint32_t shoot_time;

  static uint8_t fric_on = 0; //0x00 for off, 0xFF for on
  // static uint8_t lid_open = 0; //0x00 for closed, 0xFF for opened
  shoot_firction_toggle(pshoot, 1);
	shoot_firction_toggle(pshoot2, 1);
  shoot_lid_toggle(pshoot,1);
	shoot_lid_toggle(pshoot2,1);
  while (1)
  {
    if (rc_device_get_state(prc_dev, RC_S2_DOWN) == RM_OK)
    {
      shoot_disable(pshoot);
      shoot_disable(pshoot2);
      shoot_firction_toggle(pshoot,1); //assume that currently the fric is on
      shoot_firction_toggle(pshoot2,1); //Leo assume that currently the fric is on
      continue;
    }
    shoot_enable(pshoot);
    shoot_enable(pshoot2);
    if (rc_device_get_state(prc_dev, RC_S1_MID2UP) == RM_OK)
    {
      shoot_firction_toggle(pshoot, fric_on);
      shoot_firction_toggle(pshoot2, fric_on);
      fric_on = ~fric_on;
    }
    // if (rc_device_get_state(prc_dev, RC_S1_MID2DOWN) == RM_OK)
    // {
    //   shoot_set_cmd(pshoot, SHOOT_ONCE_CMD, 1);
    //   shoot_time = get_time_ms();
    // }
    if(rc_device_get_state(prc_dev, RC_S1_MID2DOWN) == RM_OK ||
     (prc_dev->rc_info.kb.bit.R && !prc_dev->last_rc_info.kb.bit.R))
    {
      shoot_lid_toggle(pshoot, 0);
      shoot_lid_toggle(pshoot2, 0);
    }
    if(rc_device_get_state(prc_dev, RC_S1_DOWN2MID) == RM_OK ||(
     !prc_dev->rc_info.kb.bit.R && prc_dev->last_rc_info.kb.bit.R))
    {
      shoot_lid_toggle(pshoot, 1);
      shoot_lid_toggle(pshoot2, 1);
    }

    /*------ implement the keyboard controlling over shooting ------*/
    if(prc_dev->rc_info.kb.bit.Z ) 
    {
      if(prc_dev->rc_info.kb.bit.F && !prc_dev->last_rc_info.kb.bit.F) //turn off the fric regardless the situation
      {
        shoot_firction_toggle(pshoot,1); //assume that currently the fric is on
        shoot_firction_toggle(pshoot2,1); //Leo assume that currently the fric is on
				fric_on &= ~fric_on; //set fric_on to be 0x00 i.e. off
      }
    }
    else
    {
      if(prc_dev->rc_info.kb.bit.F && !prc_dev->last_rc_info.kb.bit.F) //turn on the fric regardless the situation
      {
        shoot_firction_toggle(pshoot,0); //assume that currently the fric is off
        shoot_firction_toggle(pshoot2,0); //Leo assume that currently the fric is off
				fric_on |= ~fric_on; //set fric_on to be 0xFF i.e. on
      }
    }
    
    /*------ implement the function of a trigger ------*/
    uint16_t * shooter_heat_current = shooter_heat_get();
    uint16_t heatLimit = get_heat_limit();

    #ifndef HERO_ROBOT
    if (shooter_heat_current[0] < heatLimit && rc_device_get_state(prc_dev, RC_S2_DOWN) != RM_OK && fric_on) //not in disabled mode
    {
      if (rc_device_get_state(prc_dev, RC_WHEEL_UP) == RM_OK
        || mouse_shoot_control(prc_dev)==press)
      {
        // if (get_time_ms() - shoot_time > 2500)
        // {
        //   shoot_set_cmd(pshoot, SHOOT_CONTINUOUS_CMD, 0);
        // }
        shoot_set_cmd(pshoot, SHOOT_CONTINUOUS_CMD, CONTIN_BULLET_NUM);				
      }
      else if ((rc_device_get_state(prc_dev, RC_WHEEL_DOWN) == RM_OK && prc_dev->last_rc_info.wheel > -300)
            || mouse_shoot_control(prc_dev)==click)
      {
        shoot_set_cmd(pshoot, SHOOT_ONCE_CMD, 1);
      }
      else
      {
        shoot_set_cmd(pshoot, SHOOT_STOP_CMD, 0);
      }
    }
    #else
    if (shooter_heat_current[1] < heatLimit && rc_device_get_state(prc_dev, RC_S2_DOWN) != RM_OK && fric_on) //not in disabled mode
    {
      if (rc_device_get_state(prc_dev, RC_WHEEL_UP) == RM_OK && prc_dev->last_rc_info.wheel < 300)
      {
        shoot_set_cmd(pshoot, SHOOT_ONCE_CMD, 1);
				shoot_set_cmd(pshoot2, SHOOT_ONCE_CMD, 1);//Leo
      }
      else if ((rc_device_get_state(prc_dev, RC_WHEEL_DOWN) == RM_OK && prc_dev->last_rc_info.wheel > -300)
            || mouse_shoot_control(prc_dev)==click )
      {
        shoot_set_cmd(pshoot, SHOOT_ONCE_CMD, 1);
				shoot_set_cmd(pshoot2, SHOOT_ONCE_CMD, 1);//Leo

      }
      else
      {
        shoot_set_cmd(pshoot, SHOOT_STOP_CMD, 0);
				shoot_set_cmd(pshoot2, SHOOT_STOP_CMD, 0);//Leo
      }
    }
		shoot_execute(pshoot2);//Leo
    #endif
    shoot_execute(pshoot);
    osDelayUntil(&period, 5);

    /*-------- For shoot_task debug --------*/
    shoot_state_watch = pshoot->state;
    shoot_cmd_watch = pshoot->cmd;
    trigger_state_watch = pshoot->trigger_key;
    trigger_wheel_watch = prc_dev->rc_info.wheel;
  }
}

/**Modified by Y.H. Liu
 * @Jun 13, 2019: to support the control by keyboards
 * 
 * Switch on/off the friction wheel
 * @param toggled 0----not turned on yet, so turn the devices on
 *                1----turned on already, so turn off them now
 */
int32_t shoot_firction_toggle(shoot_t pshoot, uint8_t toggled)
{
  if (toggled)
  {
    shoot_set_fric_speed(pshoot, 100, 100);
    turn_off_laser();
	if(strncmp(pshoot->parent.name, "shoot",OBJECT_NAME_MAX_LEN))//Leo
		shoot_set_cmd(pshoot, SHOOT_STOP_CMD, 0);			//Leo	
  }
  else
  {
    shoot_set_fric_speed(pshoot, 160, 160);
    turn_on_laser();
  if(strncmp(pshoot->parent.name, "shoot",OBJECT_NAME_MAX_LEN))//Leo
		shoot_set_cmd(pshoot, SHOOT_CONTINUOUS_CMD, CONTIN_BULLET_NUM);	//Leo			
  }
  return 0;
}

/**Added by Y.H. Liu
 * @Jun 13, 2019: Declare the funtion, while leaving the defination empty
 * 
 * Send signals to the servo via interfaces to open/close the magazine lid
 * @param toggled 0----not turned on yet, so turn the devices on
 *                1----turned on already, so turn off them now
 */
int32_t shoot_lid_toggle(shoot_t pshoot, uint8_t toggled)
{
  if(toggled)
  {
    return close_lid();
  }
  else
  {
		return open_lid();
  }
}

/**Added by Y.H. Liu
 * @Jun 13, 2019: Define the function
 * 
 * Detect the left key of mouse to instruct the shoot
 */
mouse_cmd_e mouse_shoot_control(rc_device_t rc_dev)
{
  static mouse_cmd_e ret_val = non;
  static int32_t pressed_count = 0;
  uint8_t lKey = rc_dev->rc_info.mouse.l;

  if(lKey)
  {
    switch(ret_val)
    {
      case non:
        ret_val = click;
        pressed_count = 1;
        break;
      case click:
        pressed_count += 1;
        ret_val = pressed_count>CONTINUE_SHOOT_TH ? press : click;
        break;
      default:
        break;
    }
  }
  else
  {
    pressed_count = 0;
    ret_val = non;
  }
  return (pressed_count>=10 && ret_val==click) ? non : ret_val;
} 

/**Addd by Y. H. Liu
 * @Jul 3, 2019: declare the function and create the defination framework
 * 
 * Calculate the heat limit
 */
static uint16_t get_heat_limit(void)
{
  extGameRobotState_t * robotState = get_robot_state();
  uint16_t limit = 4096;
  if(robotState->robotLevel != 0)
  {
    #ifndef HERO_ROBOT
    limit = robotState->robotLevel * 120 + 120;
    #else
    limit = robotState->robotLevel * 100 + 100;
    #endif
  }
  return limit; 
}

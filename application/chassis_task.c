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
#include "smooth_filter.h"
#include <math.h>
#define RAD_TO_DEG 57.296f // 180/PI
#define MAPPING_INDEX_CRT 1.0f
#define MAPPING_INDEX_VTG 0.005f
static float vx, vy, wz;

float follow_relative_angle;
struct pid pid_follow = {0}; //angle control
static void chassis_imu_update(void *argc);

/**Eric Edited get data from ADC
  * @Jul 3, 2019: Add power gettter function: get_chassis_power
*/
extern ADC_HandleTypeDef hadc1,hadc2;
struct chassis_power chassis_power; // Using a struct to store related dara from chassis
float weight[] = {0.05f,0.05f,0.1f,0.1f,0.1f,0.1f,0.1f,0.1f,0.1f,0.2f};
int32_t current_js;
int32_t current_js_smooth;
int32_t power_pidout_js;
int32_t power_js;
uint8_t current_excess_flag_js;


/** Edited by Y.H. Liu
  * @Jun 12, 2019: modified the mode switch
  * @Jun 18, 2019: chassis current control
  * @Jun 20, 2019: adaption for hero
  *
  * Implement the customized control logic and FSM, details in Control.md
*/
#ifdef CHASSIS_POWER_CTRL
  #include "referee_system.h"
  static uint8_t superCapacitor_Ctrl(chassis_t pchassis, uint8_t low_cap_flag);
//TODO
#endif

#define km_dodge          prc_info->kb.bit.V == 1

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

  chassis_disable(pchassis);

  #ifdef HERO_ROBOT
  static uint32_t now_tick;
  static int32_t twist_count;
  static int8_t   twist_sign = 1;
  static uint32_t last_tick = 0;
  #endif
  while (1)
  {
    if (rc_device_get_state(prc_dev, RC_S2_UP) == RM_OK || rc_device_get_state(prc_dev, RC_S2_MID) == RM_OK)
    { //not disabled
      chassis_enable(pchassis);
      int32_t key_x_speed = MAX_CHASSIS_VX_SPEED/2;
      int32_t key_y_speed = MAX_CHASSIS_VY_SPEED/2;
      if(prc_info->kb.bit.SHIFT)
      {
        key_x_speed = MAX_CHASSIS_VX_SPEED;
        key_y_speed = MAX_CHASSIS_VY_SPEED;
      }
      else if (prc_info->kb.bit.CTRL)
      {
        key_x_speed /= 2;
        key_y_speed /= 2;
      }
      float square_ch4 = ((float)prc_info->ch4 * fabsf(prc_info->ch4) / RC_CH_SCALE) / RC_CH_SCALE;
      float square_ch3 = ((float)prc_info->ch3 * fabsf(prc_info->ch3) / RC_CH_SCALE) / RC_CH_SCALE;

      float temp_vx = square_ch4 * MAX_CHASSIS_VX_SPEED;
      temp_vx += (prc_info->kb.bit.W - prc_info->kb.bit.S)* key_x_speed;
      float temp_vy = -square_ch3 * MAX_CHASSIS_VY_SPEED;
      temp_vy += (prc_info->kb.bit.A - prc_info->kb.bit.D)* key_y_speed;
      vx = temp_vx * cos(-follow_relative_angle / RAD_TO_DEG) - temp_vy * sin(-follow_relative_angle / RAD_TO_DEG);
      vy = temp_vx * sin(-follow_relative_angle / RAD_TO_DEG) + temp_vy * cos(-follow_relative_angle / RAD_TO_DEG);

      if(km_dodge)
      {
        #ifndef HERO_ROBOT
        wz = 1.1f * MAX_CHASSIS_VW_SPEED;
        #else
          //time-based twist with a sin function
          now_tick = HAL_GetTick();
          twist_count += last_tick==0 ? twist_sign : twist_sign * (now_tick-last_tick);
          last_tick = now_tick;
          if(twist_count >= 500 || twist_count <= -500)
          {
		        twist_count = twist_count>0?500:-500;
            twist_sign *= -1;
          }
          wz = twist_sign * sin(PI * twist_count / 500) * MAX_CHASSIS_VW_SPEED;
        #endif
        dodging |= 1;
        if(vx!=0 || vy!=0)
        {
          vx*=0.6f;
          vy*=0.6f;
          wz*=0.8f;
        }
      }
      else
      {
        wz  = pid_calculate(&pid_follow, follow_relative_angle, 0);
        dodging &= 0;
        #ifdef HERO_ROBOT
        last_tick = 0;
        twist_count = 0;
        #endif
      }

      chassis_set_offset(pchassis, ROTATE_X_OFFSET, ROTATE_Y_OFFSET);
      chassis_set_speed(pchassis, vx, vy, wz);
    }
    else
    {
      chassis_set_speed(pchassis, 0, 0, 0);
      chassis_disable(pchassis);
    }
    chassis_set_acc(pchassis, 0, 0, 0);

    #ifdef CHASSIS_POWER_CTRL
      uint8_t current_excess_flag = 0;
      uint8_t low_volatge_flag = 0;
      do
      {
        chassis_imu_update(pchassis);
        chassis_execute(pchassis);
        current_js = get_chassis_power(&chassis_power); // Power Value Getter
        osDelayUntil(&period, 2);
        /*-------- Then, adjust the power --------*/
      //get the buffer
        extPowerHeatData_t * referee_power = get_heat_power();
      //set the current & voltage flags
        if(referee_power->chassisPowerBuffer > LOW_BUFFER && chassis_power.voltage>LOW_VOLTAGE && 
           chassis_power.power > (CHASSIS_POWER_TH+LOW_BUFFER)/WORKING_VOLTAGE)
        {
          current_excess_flag = 2;
        }
        else if(chassis_power.power > CHASSIS_POWER_TH/WORKING_VOLTAGE)
        {
          current_excess_flag = 1;
        }
        else
        {
          current_excess_flag = 0;
        }
        if(chassis_power.voltage<LOW_VOLTAGE)
          low_volatge_flag = 1;
        else
          low_volatge_flag = 0;
      //control the supercap
        uint8_t sw = superCapacitor_Ctrl(pchassis,low_volatge_flag);
      //control the speed ref if necessary
        if(current_excess_flag)
        {
          float prop = chassis_power.power / ((CHASSIS_POWER_TH+(current_excess_flag-1)*LOW_BUFFER)/WORKING_VOLTAGE);
          prop = sqrtf(prop);
          chassis_set_vx_vy(pchassis, pchassis->mecanum.speed.vx/prop, pchassis->mecanum.speed.vy/prop);
        }
        current_excess_flag_js = current_excess_flag;

        power_data_sent_by_can(current_excess_flag, low_volatge_flag, chassis_power.power, chassis_power.voltage, referee_power->chassisPowerBuffer);

      }while(current_excess_flag);
    #else
      chassis_imu_update(pchassis);
      chassis_execute(pchassis);
      get_chassis_power(&chassis_power); // Power Value Getter
      osDelayUntil(&period, 2);
      power_pidout_js = pchassis->motor[0].current +
                        pchassis->motor[1].current +
                        pchassis->motor[2].current +
                        pchassis->motor[3].current;
    #endif
  }
}


static void chassis_imu_update(void *argc)
{
  struct ahrs_sensor mpu_sensor;
  struct attitude mahony_atti;
  chassis_t pchassis = (chassis_t)argc;
  mpu_get_data(&mpu_sensor);
  mahony_ahrs_updateIMU(&mpu_sensor, &mahony_atti);
  chassis_gyro_update(pchassis, -mahony_atti.yaw, -mpu_sensor.wz * RAD_TO_DEG);
}

#ifdef CHASSIS_POWER_CTRL
static uint8_t superCapacitor_Ctrl(chassis_t pchassis, uint8_t low_cap_flag)
{
  if(!low_cap_flag)
  {
    for(int i=0; i<4; i++)
    {
      if(pchassis->motor[i].data.speed_rpm < pchassis->mecanum.wheel_rpm[i]/20)
        return 1;
    }
  }
  return 0; //charge the super capacitor
}
#endif

int32_t chassis_set_relative_angle(float angle)
{
  follow_relative_angle = angle;
  return 0;
}

int get_chassis_power(struct chassis_power *chassis_power)
{
	if (HAL_ADC_PollForConversion(&hadc1,10000)== HAL_OK)
	{
		chassis_power->current_debug = HAL_ADC_GetValue(&hadc1);
	}
	if (HAL_ADC_PollForConversion(&hadc2,10000)==HAL_OK)
	{
		chassis_power->voltage_debug = HAL_ADC_GetValue(&hadc2);
	}

	chassis_power->current = smooth_filter(10,((float)chassis_power->current_debug) * MAPPING_INDEX_CRT,weight)/2;
	//chassis_power->voltage = smooth_filter(10,((float)chassis_power->voltage_debug) * MAPPING_INDEX_VTG,weight);
	// chassis_power->power = chassis_power->current * chassis_power->voltage;
  chassis_power->power = ((chassis_power->current-2048.0f)*25.0f/1024.0f)/10.0f; // Assume the sensor is 20 A
	current_js = (int) (chassis_power->current_debug*1000);
	current_js_smooth = (int) (chassis_power->current*1000);
	power_js = (int)(chassis_power->power*1000);
	return chassis_power->power;
}

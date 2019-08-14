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
#include "drv_io.h"
#define RAD_TO_DEG 57.296f // 180/PI
#define MAPPING_INDEX_CRT 1.0f
#define MAPPING_INDEX_VTG 1.0f

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
uint8_t sensor_offline = 0;
#define CURRENT_OFFLINE 0x0Fu
#define VOLTAGE_OFFLINE 0xF0u


/** Edited by Y.H. Liu
  * @Jun 12, 2019: modified the mode switch
  * @Jun 18, 2019: chassis current control
  * @Jun 20, 2019: adaption for hero
  * @Jul 27, 2019: confirm the control signals for supercapacitor
  * @Jul 27, 2019: add the logic to handle the sensor offline
  * @Jul 31, 2019: change the dodging logic for Hero
  *
  * Implement the customized control logic and FSM, details in Control.md
*/
#ifdef CHASSIS_POWER_CTRL
  #include "referee_system.h"
  static uint8_t superCapacitor_Ctrl(chassis_t pchassis, uint8_t low_cap_flag, uint8_t extra_current, uint8_t last_sw);
#endif

#define km_dodge          prc_info->kb.bit.SHIFT == 1

static uint8_t dodging = 0;
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

  pid_struct_init(&pid_follow, MAX_CHASSIS_VW_SPEED*0.85f, 50, 7.673848152f, 0.0f, 2.0f);

  chassis_disable(pchassis);

  chassis_set_offset(pchassis, ROTATE_X_OFFSET, ROTATE_Y_OFFSET);
  #ifdef HERO_ROBOT
  static int8_t   twist_cnt = 1;
  static float    last_relative_angle;
  #endif
  while (1)
  {
    static uint8_t sw, extra_current;
    float vx, vy, wz;
    if (rc_device_get_state(prc_dev, RC_S2_UP) == RM_OK || rc_device_get_state(prc_dev, RC_S2_MID) == RM_OK)
    { //not disabled
      chassis_enable(pchassis);
      int32_t key_x_speed = MAX_CHASSIS_VX_SPEED/2;
      int32_t key_y_speed = MAX_CHASSIS_VY_SPEED/2;
      if(prc_info->kb.bit.V)
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
        wz = MAX_CHASSIS_VW_SPEED;
        #else
        if(!dodging)
        {
          twist_cnt = 1;
        }
        else if(last_relative_angle * follow_relative_angle <= 0 || 
                (follow_relative_angle>=DODGING_TH-2.5f && last_relative_angle<=DODGING_TH-2.5f)|| 
                (follow_relative_angle<=-DODGING_TH+2.5f && last_relative_angle>=-DODGING_TH+2.5f))
        {
          twist_cnt += 1;
        }
        last_relative_angle = follow_relative_angle;
        int8_t delta_wz = (twist_cnt/2)%2 ? DODGING_TH : -DODGING_TH;
        wz  = pid_calculate(&pid_follow, follow_relative_angle, delta_wz);
        #endif
        dodging |= 1;
      }
      else
      {
        wz  = pid_calculate(&pid_follow, follow_relative_angle, 0);
        dodging &= 0;
      }
      if(abs((int)vx)>2*MAX_CHASSIS_VX_SPEED/3 || abs((int)vy)>=2*MAX_CHASSIS_VY_SPEED/3 || abs((int)wz)>=2*MAX_CHASSIS_VW_SPEED/3)
        extra_current = 1;
      else
        extra_current = 0;
      if(dodging&&(vx!=0 || vy!=0))
      {
        vx*=0.6f;
        vy*=0.6f;
        wz*=0.8f;
      }
      chassis_set_speed(pchassis, vx, vy, wz);
    }
    else
    {
      chassis_set_speed(pchassis, 0, 0, 0);
      chassis_disable(pchassis);
      WRITE_LOW_CAPACITOR();
    }
    chassis_set_acc(pchassis, 0, 0, 0);

    #ifdef CHASSIS_POWER_CTRL
      uint8_t current_excess_flag = 0;
      uint8_t low_volatge_flag = 0;
      do
      {
        chassis_imu_update(pchassis);
        chassis_execute(pchassis);
        get_chassis_power(&chassis_power); // Power Value Getter
        if(sensor_offline & CURRENT_OFFLINE)
        {// if offline, then use the returned current from the motors to determine the current
          chassis_power.current =(abs(pchassis->motor[0].data.given_current)+
                                  abs(pchassis->motor[1].data.given_current)+
                                  abs(pchassis->motor[2].data.given_current)+
                                  abs(pchassis->motor[3].data.given_current))/1000*MOTOR_TORQUE_CURRENT_CO;
          if(sensor_offline & VOLTAGE_OFFLINE || chassis_power.voltage < LOW_VOLTAGE)
            chassis_power.current *= 2;
        }
        osDelayUntil(&period, 2);
        /*-------- Then, adjust the power --------*/
      //get the buffer
        ext_power_heat_data_t * referee_power = get_heat_power();
        shooter_data_sent_by_can(referee_power);
        if(referee_power->chassis_power_buffer == 0)
          LED_R_ON();
        else if(chassis_check_enable(pchassis))
          LED_R_OFF(); 
      //set the current & voltage flags
        if(referee_power->chassis_power_buffer > LOW_BUFFER && chassis_power.voltage>LOW_VOLTAGE && 
           chassis_power.current > (CHASSIS_POWER_TH+LOW_BUFFER)/WORKING_VOLTAGE)
        {
          current_excess_flag = 2;
        }
        else if(chassis_power.current > CHASSIS_POWER_TH/WORKING_VOLTAGE)
        {
          current_excess_flag = 1;
        }
        else
        {
          current_excess_flag = 0;
        }
        if(chassis_power.voltage<LOW_VOLTAGE || sensor_offline&VOLTAGE_OFFLINE)
          low_volatge_flag = 1;          
        else
          low_volatge_flag = 0;
      //control the supercap
        sw = superCapacitor_Ctrl(pchassis,low_volatge_flag, extra_current | current_excess_flag, sw);
        if(sw) //either offline or must use the super cap
          WRITE_HIGH_CAPACITOR();
        else
          WRITE_LOW_CAPACITOR();
      //control the speed ref if necessary
        if(current_excess_flag)
        {
          float prop = chassis_power.current / ((CHASSIS_POWER_TH+(current_excess_flag-1)*LOW_BUFFER)/WORKING_VOLTAGE);
          prop = sqrtf(prop);
          chassis_set_vx_vy(pchassis, pchassis->mecanum.speed.vx/prop, pchassis->mecanum.speed.vy/prop);
          chassis_set_vw(pchassis, pchassis->mecanum.speed.vw/prop);
          LED_R_ON();
        }
        else if(chassis_check_enable(pchassis) && referee_power->chassis_power_buffer!=0)
        {
          LED_R_OFF(); 
        }
        //Share the flags and data with the gimbal
        current_excess_flag_js = current_excess_flag;
        power_data_sent_by_can(current_excess_flag, low_volatge_flag, chassis_power.current, chassis_power.voltage, referee_power->chassis_power_buffer);
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
/**Added by Y.H. Liu
 * @Jul 2X, 2019: define the function
 * @Jul 28, 2019: modify the logic
 * 
 * RetVal: 0----write low to the relay, supercapacitor is charging
 *         1----write high to the relay, supercapacitor is discharging
 */
static uint8_t superCapacitor_Ctrl(chassis_t pchassis, uint8_t low_cap_flag, uint8_t extra_current_need, uint8_t last_sw)
{
  static uint32_t last_operation_time=0;
  if(extra_current_need)
    return 1; // as long as the current excess the limitation, then switch on the supercapacitor
  else // no longer excessing
  {
    if(get_time_ms()-last_operation_time<SUPER_CAP_HOLDING_TIME)
      return last_sw; 
    else
    {
      last_operation_time = get_time_ms();
      if(low_cap_flag)
        return 0; // no excess & timeout & (low volatge or offline)
      else 
        return 1;
    }
  }
  
}
#endif

int32_t chassis_set_relative_angle(float angle)
{
  follow_relative_angle = angle;
  return 0;
}

/**Added by Eric Chen
 * @Jun 2019: Define the function & current
 * @Jul 23, 2019: Change the current to be *current* by Y.H. Liu
 * 
 * BRIEF: refresh the power of 2 chassis
 * PARAM: chassis_power ---- struct storing the power data
 *        -  current_debug ---- raw data from current sensor
 *        -  voltage_debug ---- raw data from voltage sensor
 *        -  current ---- real current data after scaling
 *        -  voltage ---- real voltage data after scaling
 *        -  power ---- current * voltage
 *        current_smoothed ---- smoothed current data
 *        voltage_smoothed ---- smoothed voltage data
 *        current_js ---- global variable exposing the current
 *        voltage_js ---- global variable exposing the voltage
 *        power_js ---- global variable exposing the power
 * REVAL: the value of power
 */
int get_chassis_power(struct chassis_power *chassis_power)
{ //Getting raw data
	if (HAL_ADC_PollForConversion(&hadc1,10000)== HAL_OK)
	{
		chassis_power->current_debug = HAL_ADC_GetValue(&hadc1);
	}
	if (HAL_ADC_PollForConversion(&hadc2,10000)==HAL_OK)
	{
		chassis_power->voltage_debug = HAL_ADC_GetValue(&hadc2);
	}
  // Check the offline # TODO: determine whether this is corrent
  if(chassis_power->current_debug < 1300)
    sensor_offline |= CURRENT_OFFLINE;
  if(chassis_power->voltage_debug < 1300)
    sensor_offline |= VOLTAGE_OFFLINE;
  // Smoothed raw data
	float current_smoothed = smooth_filter(10,((float)chassis_power->current_debug) * MAPPING_INDEX_CRT,weight)/2;
	//float voltage_smoothed = smooth_filter(10,((float)chassis_power->voltage_debug) * MAPPING_INDEX_VTG,weight); //TODO: change the coefficient
  // Store the real data
  chassis_power->current = fabs((((chassis_power->current_debug-2048.0f)*25.0f/1024.0f)/10.0f-2.45f)*2.1); // Assume the sensor is 20 A
  chassis_power->voltage = fabs(((5.0f * 5.0f) / 4096.0f)*chassis_power->voltage_debug); // TODO: change the scaling
	//chassis_power->power = chassis_power->current * chassis_power->voltage;
	// Refresh the js variables
  current_js = (int) (chassis_power->current_debug*1000);
	current_js_smooth = (int) (chassis_power->current*1000);
	//power_js = (int)(chassis_power->power*1000);
	return chassis_power->power;
}

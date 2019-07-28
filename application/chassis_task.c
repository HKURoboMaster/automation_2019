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
#include "chassis_calc.h"
#include "timer_task.h"
#include "infantry_cmd.h"
#include "ahrs.h"
#include "drv_imu.h"
#include "smooth_filter.h"
#include <math.h>
#include "drv_io.h"
#define RAD_TO_DEG 57.296f // 180/PI
#define MAPPING_INDEX_CRT 1.0f
#define MAPPING_INDEX_VTG 0.005f

float follow_relative_angle;
struct pid pid_follow = {0}; //angle control
static void chassis_imu_update(void *argc);

// =======uncomment this if using rc, not random movement=======
// #define USING_RC_CTRL_VY
#define RC_CONNECTED

//=======ir & control global var========
int left_blocked = 0, right_blocked = 0;  //IR detects if left or right side is blocked
int left_ir_js = 0, right_ir_js = 0;  // for jscope

//=======chassis movement logic global var========
chassis_state_t state = {IDLE_STATE, IDLE_CONSTANT_SPEED}; //The state of chassis
cv_dynamic_event_t dynamic_eve = ENEMY_STAY_STILL;
cv_static_event_t static_eve = ENEMY_NOT_DETECTED;
power_event_t power_eve = POWER_NORMAL;
armor_event_t armor_eve = NO_HIT_FOR_X_SEC;
int vy_js = 0; // for debugging vy
int chassis_yaw_js = 0;

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
  * @Jun 14, 2019: delete the supercapacitor control for sentry
  *
  * Implement the customized control logic and FSM, details in Control.md
*/
#ifdef CHASSIS_POWER_CTRL
  #include "referee_system.h"
  static uint8_t superCapacitor_Ctrl(chassis_t pchassis, uint8_t low_cap_flag);
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

  pid_struct_init(&pid_follow, MAX_CHASSIS_VW_SPEED*0.85f, 50, 7.673848152f, 0.0f, 2.0f);

  chassis_disable(pchassis);

  generate_movement(); // initialize a movement

  set_state(&state, IDLE_STATE); // default state: idle

  //activate_bounded_movement(50000);

  #ifdef HERO_ROBOT
  static int8_t   twist_sign = 1;
  #endif
      
  while (1)
  {
    check_ir_signal(); // check ir signals
    
    float vx, vy, wz;
    #ifdef RC_CONNECTED
		if (rc_device_get_state(prc_dev, RC_S2_UP) == RM_OK || rc_device_get_state(prc_dev, RC_S2_MID) == RM_OK)
		{ //not disabled
    #endif

			chassis_enable(pchassis);

      if (rc_device_get_state(prc_dev, RC_S2_MID) == RM_OK) {

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

      } else {

        vy = chassis_random_movement(pchassis, get_spd(&state));
        vy_js = vy * 1000;

      }

      chassis_set_offset(pchassis, ROTATE_X_OFFSET, ROTATE_Y_OFFSET);

      float raw_vy = vy;
      vy = direction_control(vy); // direction control

      if (rc_device_get_state(prc_dev, RC_S2_MID) == RM_OK) {
        // if blocked by wall
        if (vy == 0) {
          generate_movement(); // bounce off by another movement
          adjust_accumulated_distance(raw_vy);
        }
      }

      chassis_set_speed(pchassis, 0, vy, 0);

    #ifdef RC_CONNECTED
    }
    else
    {
      chassis_set_speed(pchassis, 0, 0, 0);
      chassis_disable(pchassis);
    }
    #endif
    
    chassis_set_acc(pchassis, 0, 0, 0);

    #ifdef CHASSIS_POWER_CTRL
      uint8_t current_excess_flag = 0;
      uint8_t low_volatge_flag = 0xFF;
      do
      {
        chassis_imu_update(pchassis);
        if(sensor_offline & CURRENT_OFFLINE)
        {
          //TODO: what if current sensor offline
        }
        chassis_execute(pchassis);
        get_chassis_power(&chassis_power); // Power Value Getter
        osDelayUntil(&period, 2);
        /*-------- Then, adjust the power --------*/
      //get the buffer
        ext_power_heat_data_t * referee_power = get_heat_power();
        shooter_data_sent_by_can(referee_power);
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
        uint8_t sw = superCapacitor_Ctrl(pchassis,low_volatge_flag);
        if(sw)
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
        else if(chassis_check_enable(pchassis))
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
  chassis_yaw_js = mahony_atti.yaw;
  chassis_gyro_update(pchassis, -mahony_atti.yaw, -mpu_sensor.wz * RAD_TO_DEG);
}

#ifdef CHASSIS_POWER_CTRL
static uint8_t superCapacitor_Ctrl(chassis_t pchassis, uint8_t low_cap_flag)
{
  if(!low_cap_flag)
  {
    for(int i=0; i<4; i++)
    {
      if(pchassis->motor[i].data.speed_rpm < pchassis->mecanum.wheel_rpm[i]/5)
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

/**Added by Eric Chen
 * @Jun 2019: Define the function & current
 * @Jul 23, 2019: Change the current to be *current* by Y.H. Liu
 * 
 * BRIEF: refresh the power of the chassis
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
  if(chassis_power->current_debug > 1200)
    sensor_offline |= CURRENT_OFFLINE;
  if(chassis_power->voltage_debug > 1200)
    sensor_offline |= VOLTAGE_OFFLINE;
  // Smoothed raw data
	float current_smoothed = smooth_filter(10,((float)chassis_power->current_debug) * MAPPING_INDEX_CRT,weight)/2;
	float voltage_smoothed = smooth_filter(10,((float)chassis_power->voltage_debug) * MAPPING_INDEX_VTG,weight); //TODO: change the coefficient
  // Store the real data
  chassis_power->current = (((current_smoothed-2048.0f)*25.0f/1024.0f)/10.0f-2.45f)*3; // Assume the sensor is 20 A
  chassis_power->voltage = (((voltage_smoothed-2048.0f)*25.0f/1024.0f)/10.0f-2.45f)*3; // TODO: change the scaling
	chassis_power->power = chassis_power->current * chassis_power->voltage;
	// Refresh the js variables
  current_js = (int) (chassis_power->current_debug*1000);
	current_js_smooth = (int) (chassis_power->current*1000);
	power_js = (int)(chassis_power->power*1000);
	return chassis_power->power;
}

/**
 * Jerry 10 Jul
 * @brief Control the direction of v so that sentry won't crash.
 */
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

/**
 * Jerry 10 Jul
 * @brief Update IR Sensor's signal as well as updating jscope
 * variables.
 */
void check_ir_signal(void) {
  left_blocked = (HAL_GPIO_ReadPin(IR_LEFT_Port, IR_LEFT_Pin) == GPIO_PIN_RESET);
  right_blocked = (HAL_GPIO_ReadPin(IR_RIGHT_Port, IR_RIGHT_Pin) == GPIO_PIN_RESET);
  left_ir_js = left_blocked ? 5000 : 0;
  right_ir_js = right_blocked ? -5000 : 0;
}

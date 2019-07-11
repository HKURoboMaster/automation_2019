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
#define MAPPING_INDEX_CRT 0.005f
#define MAPPING_INDEX_VTG 0.005f
static float vx, vy, wz;

float follow_relative_angle;
struct pid pid_follow = {0}; //angle control
static void chassis_imu_update(void *argc);

///=======ir & control global var========
int left_blocked = 0, right_blocked = 0;  //IR detects if left or right side is blocked
int left_ir_js = 0, right_ir_js = 0;  //for jscope
int using_rm = USING_RM;  //if using a remote control, if in use then 1

//=======chassis movement logic global var========
chassis_state_t state = {IDLE_STATE, IDLE_CONSTANT_SPEED}; //The state of chassis
cv_dynamic_event_t dynamic_eve = ENEMY_STAY_STILL;
cv_static_event_t static_eve = ENEMY_NOT_DETECTED;
power_event_t power_eve = POWER_NORMAL;
armor_event_t armor_eve = NO_HIT_FOR_THREE_SEC;

/**Eric Edited get data from ADC
  * @Jul 3, 2019: Add power gettter function: get_chassis_power
*/
extern ADC_HandleTypeDef hadc1,hadc2;
struct chassis_power chassis_power; // Using a struct to store related dara from chassis
float weight[] = {0.05f,0.05f,0.1f,0.1f,0.1f,0.1f,0.1f,0.1f,0.1f,0.2f};
int32_t current_js;
int32_t power_pidout_js;


/** Edited by Y.H. Liu
  * @Jun 12, 2019: modified the mode switch
  * @Jun 18, 2019: chassis current control
  * @Jun 20, 2019: adaption for hero
  *
  * Implement the customized control logic and FSM, details in Control.md
*/
#ifdef CHASSIS_POWER_CTRL
  #include "referee_system.h"
  static uint8_t reset_chassis_speed(chassis_t pchassis, uint8_t flag, float current);
  static uint8_t detect_chassis_power(chassis_t, uint8_t *, uint8_t sprint_cmd, uint16_t sc_v, float current);
  typedef enum {NORMAL=0, SPRINT, BUFF_RECOVER} chassis_power_t;
#endif

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

  while (1)
  {
    check_ir_signal(); // check ir signals
    if (rc_device_get_state(prc_dev, RC_S2_UP) == RM_OK || rc_device_get_state(prc_dev, RC_S2_MID) == RM_OK)
    { //not disabled
      chassis_enable(pchassis);
      int32_t key_x_speed = 2*MAX_CHASSIS_VX_SPEED/3;
      int32_t key_y_speed = 2*MAX_CHASSIS_VY_SPEED/3;
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
      temp_vy += (prc_info->kb.bit.D - prc_info->kb.bit.A)* key_y_speed;
      vx = temp_vx * cos(-follow_relative_angle / RAD_TO_DEG) - temp_vy * sin(-follow_relative_angle / RAD_TO_DEG);
      vy = temp_vx * sin(-follow_relative_angle / RAD_TO_DEG) + temp_vy * cos(-follow_relative_angle / RAD_TO_DEG);

      chassis_set_offset(pchassis, ROTATE_X_OFFSET, ROTATE_Y_OFFSET);

      vy = direction_control(vy); // direction control

      chassis_set_speed(pchassis, 0, vy, 0);
    }
    else
    {
      chassis_set_speed(pchassis, 0, 0, 0);
      chassis_disable(pchassis);
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
    */

    #ifdef CHASSIS_POWER_CTRL
      uint8_t superCapacitor_Ctrl = 0;
      uint8_t power_excess = 0;
      do
      {
        if(power_excess)
          power_excess = reset_chassis_speed(pchassis, power_excess, chassis_power.current);
        chassis_imu_update(pchassis);
        chassis_execute(pchassis);
        get_chassis_power(&chassis_power); // Power Value Getter
        osDelayUntil(&period, 2);
        power_excess = detect_chassis_power(pchassis,
                                            &superCapacitor_Ctrl,
                                            prc_info->kb.bit.SHIFT || prc_info->ch2>600,
                                            chassis_power.voltage,
                                            chassis_power.current);
        //set_cmd_to_sc(superCapacitor_Ctrl)
      }while(power_excess);
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
 * @Jun 18, 2019: implement the function
 *
 * Reset the chassis moving speed ref in case too much power consumed
 */
static uint8_t reset_chassis_speed(chassis_t pchassis, uint8_t flag, float current)
{
  float portion;
  switch (flag)
  {
  case 1:
    portion = sqrtf(current * WORKING_VOLTAGE / CHASSIS_POWER_TH);
    break;
  case 2:
    portion = sqrtf(current * WORKING_VOLTAGE / CHASSIS_POWER_TH);
    break;
  case 4:
    portion = sqrtf(current * WORKING_VOLTAGE / (0.8f * CHASSIS_POWER_TH));
    break;
  default:
    portion = 1;
  }

  pchassis->mecanum.speed.vx /= portion;
  pchassis->mecanum.speed.vy /= portion;

  return 0;
}
/**Added by Y.H. Liu
 * @Jun 18, 2019: implement the function
 *
 * state update and detect whether the chassis is consuming too much power
 */
static uint8_t detect_chassis_power(chassis_t pchassis, uint8_t * supercap_ctrl, uint8_t sprint_cmd, uint16_t sc_v, float current)
{
  static chassis_power_t st = NORMAL;
  static uint32_t no_buffer_time = 0;
  static uint32_t last_click = 0;
  int16_t current_limit = CHASSIS_POWER_TH / WORKING_VOLTAGE;
  extPowerHeatData_t * power = get_heat_power();

  if(power->chassisPowerBuffer<=0)
  {
    no_buffer_time = no_buffer_time==0 ? 1 : no_buffer_time+(HAL_GetTick()-last_click);
    last_click = HAL_GetTick();
  }
  else
  {
    no_buffer_time = 0;
    last_click = 0;
  }

  switch (st)
  {
  case NORMAL:
    *supercap_ctrl = 0;
    if(power->chassisPowerBuffer <= LOW_BUFFER)
      st = BUFF_RECOVER;
    else if(sprint_cmd)
      st = SPRINT;
    else
      st = NORMAL;
    break;
  case SPRINT:
    *supercap_ctrl = 1;
    if(!sprint_cmd || sc_v <= LOW_VOLTAGE)
      st = NORMAL;
    else if(no_buffer_time > NO_BUFFER_TIME_TH)
      st = BUFF_RECOVER;
    else
      st = NORMAL;
    break;
  case BUFF_RECOVER:
    *supercap_ctrl = 0;
    current_limit *= 0.8; // limit the current below the allowed value
    if(power->chassisPowerBuffer > 2*LOW_BUFFER)
      st = NORMAL;
    else
      st = BUFF_RECOVER;
    break;
  default:
    *supercap_ctrl = 0;
    st = NORMAL;
    break;
  }

  return current>current_limit ? 1<<(uint8_t)st : 0;
  //thus, return 0 <=> No excessing
  //             1 <=> NORMAL, exceeded
  //             2 <=> SRPINT, exceeded
  //             4 <=> RECOVERY, exceeded
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

	chassis_power->current = smooth_filter(10,((float)chassis_power->current_debug) * MAPPING_INDEX_CRT,weight);
	chassis_power->voltage = smooth_filter(10,((float)chassis_power->voltage_debug) * MAPPING_INDEX_VTG,weight);
	// chassis_power->power = chassis_power->current * chassis_power->voltage;
  chassis_power->power = chassis_power->current * 24u;
	current_js = (int) chassis_power->power;

	return chassis_power->power;
}

/**
 * Jerry @10 Jul
 * Control the direction of v so that sentry won't crash.
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
 * Jerry @10 Jul
 * Update IR Sensor's signal as well as updating jscope
 * variables.
 */
void check_ir_signal(void) {
  left_blocked = (HAL_GPIO_ReadPin(IR_LEFT_Port, IR_LEFT_Pin) == GPIO_PIN_RESET);
  right_blocked = (HAL_GPIO_ReadPin(IR_RIGHT_Port, IR_RIGHT_Pin) == GPIO_PIN_RESET);
  left_ir_js = left_blocked ? 5000 : 0;
  right_ir_js = right_blocked ? -5000 : 0;
}

/**
 * Jerry @10 Jul
 * Set the chassis state to be one of the three states.
 * Example: set_state(&state, IDLE_STATE);
 */
void set_state(chassis_state_t * state, chassis_state_name_t dest_state) {
  state->state_name = dest_state;
  switch (dest_state) {
    case IDLE_STATE:
      state->constant_spd = IDLE_CONSTANT_SPEED;
      break;
    case NORMAL_STATE:
      state->constant_spd = NORMAL_CONSTANT_SPEED;
      break;
    case BOOST_STATE:
      state->constant_spd = BOOST_CONSTANT_SPEED;
  }
}

/**
 * Jerry @10 Jul
 * Get the chassis state name.
 */
chassis_state_name_t get_state(const chassis_state_t * state) {
  return state->state_name;
}

/**
 * Jerry @10 Jul
 * Get the constant speed under current state.
 */
float get_spd(const chassis_state_t * state) {
  return state->constant_spd;
}

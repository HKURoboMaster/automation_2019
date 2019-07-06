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
#include "dbus.h"
#include "gimbal.h"
#include "timer_task.h"
#include "gimbal_task.h"
#include "infantry_cmd.h"
#include "offline_check.h"
#include "param.h"
#include "ramp.h"

#define DEFAULT_IMU_TEMP 50

/* patrol period time (ms) */
#define GIMBAL_PERIOD 2
/* gimbal back center time (ms) */
#define BACK_CENTER_TIME 3000

float pit_delta, yaw_delta;

static void imu_temp_ctrl_init(void);
static int32_t gimbal_imu_update(void *argc);
static int32_t imu_temp_keep(void *argc);
static void auto_gimbal_adjust(gimbal_t pgimbal);
static void gimbal_state_init(gimbal_t pgimbal);

uint8_t auto_adjust_f;
uint8_t auto_init_f;

int32_t mpu_pit, mpu_yaw, mpu_rol;
int32_t mpu_wx, mpu_wy, mpu_wz;

/* control ramp parameter */
static ramp_t yaw_ramp = RAMP_GEN_DAFAULT;
static ramp_t pitch_ramp = RAMP_GEN_DAFAULT;

int32_t yaw_angle_fdb_js, yaw_angle_ref_js;
int32_t pit_angle_fdb_js, pit_angle_ref_js;
int32_t yaw_spd_fdb_js, yaw_spd_ref_js;
int32_t pit_spd_fdb_js, pit_spd_ref_js;
int32_t yaw_ecd_angle_js, pit_ecd_angle_js;
/** Edited by Y.H. Liu
 *  @Jun 12, 2019: modified the mode switch
 *
 *  Implement the customized control logic and FSM, details in Control.md
 */
extern int32_t auto_aiming_pitch;
extern int32_t auto_aiming_yaw;
void gimbal_task(void const *argument)
{
  uint32_t period = osKernelSysTick();
  rc_device_t prc_dev = NULL;
  rc_info_t prc_info = NULL;
  gimbal_t pgimbal = NULL;
  cali_sys_t *pparam = NULL;

  pgimbal = gimbal_find("gimbal");
  prc_dev = rc_device_find("can_rc");
  pparam = get_cali_param();

  if (pparam->gim_cali_data.calied_done == CALIED_FLAG)
  {
    gimbal_set_offset(pgimbal, pparam->gim_cali_data.yaw_offset, pparam->gim_cali_data.pitch_offset);
  }
  else
  {
    auto_adjust_f = 1;
  }

  gimbal_init_state_reset();

  if (prc_dev != NULL)
  {
    prc_info = rc_device_get_info(prc_dev);
  }
  else
  {
  }

  soft_timer_register(imu_temp_keep, (void *)pgimbal, 5);
  soft_timer_register(gimbal_push_info, (void *)pgimbal, 10);

  imu_temp_ctrl_init();

  while (1)
  {
    /*
    if (rc_device_get_state(prc_dev, RC_S2_UP) == RM_OK)
    {
      gimbal_set_yaw_mode(pgimbal, GYRO_MODE);
      pit_delta = -(float)prc_info->ch4 * 0.0007f;
      yaw_delta = -(float)prc_info->ch3 * 0.0007f;
      gimbal_set_pitch_delta(pgimbal, pit_delta);
      gimbal_set_yaw_delta(pgimbal, yaw_delta);
    }

    if (rc_device_get_state(prc_dev, RC_S2_MID) == RM_OK)
    {
      gimbal_set_yaw_mode(pgimbal, ENCODER_MODE);
      pit_delta = -(float)prc_info->ch4 * 0.0007f;
      gimbal_set_pitch_delta(pgimbal, pit_delta);

      if (rc_device_get_state(prc_dev, RC_S2_UP2MID) == RM_OK)
      {
        gimbal_set_yaw_angle(pgimbal, 0, 0);
      }
    }

    if (rc_device_get_state(prc_dev, RC_S2_DOWN2MID) == RM_OK)
    {
      gimbal_set_yaw_angle(pgimbal, 0, 0);
    }

    if (rc_device_get_state(prc_dev, RC_S2_DOWN) == RM_OK)
    {
      gimbal_set_yaw_mode(pgimbal, ENCODER_MODE);
    }
    */
    if(rc_device_get_state(prc_dev, RC_S2_DOWN2MID) == RM_OK)
    {
      //switched out disabled mode
      gimbal_pitch_enable(pgimbal);
      gimbal_yaw_enable(pgimbal);
    }
    if(rc_device_get_state(prc_dev, RC_S2_MID2DOWN) == RM_OK)
    {
      //switch to the disabled mode
      gimbal_set_yaw_angle(pgimbal, 0, 0);
    }
    if (rc_device_get_state(prc_dev, RC_S2_UP) == RM_OK || rc_device_get_state(prc_dev, RC_S2_MID) == RM_OK
    ||  rc_device_get_state(prc_dev, RC_S2_MID2UP) == RM_OK || rc_device_get_state(prc_dev,RC_S2_UP2MID == RM_OK))
    {
      //manual control mode i.e. chassis follow gimbal
      if(prc_info->kb.bit.X != 1)
      {
        //auto_aimming
        //gimbal_set_pitch_speed(pgimbal, auto_aiming_pitch);
        //gimbal_set_yaw_speed(pgimbal, auto_aiming_yaw);

        float square_ch3 = (float)prc_info->ch3 * abs(prc_info->ch3) / RC_CH_SCALE;

        gimbal_set_yaw_mode(pgimbal, GYRO_MODE);
        pit_delta = -(float)prc_info->ch4 * GIMBAL_RC_PITCH + (float)prc_info->mouse.y * GIMBAL_MOUSE_PITCH;
        yaw_delta =      square_ch3       * GIMBAL_RC_YAW   + (float)prc_info->mouse.x * GIMBAL_MOUSE_YAW;
        yaw_delta += prc_info->kb.bit.E ? YAW_KB_SPEED : 0;
        yaw_delta -= prc_info->kb.bit.Q ? YAW_KB_SPEED : 0;
        gimbal_set_pitch_delta(pgimbal, pit_delta);
        gimbal_set_yaw_delta(pgimbal, yaw_delta);
      }
      else
      {
        gimbal_set_yaw_mode(pgimbal, ENCODER_MODE);
        gimbal_set_yaw_angle(pgimbal, 0, 0);
        //no rotation allowed, gimbal rotate back to the netural position of the encoder
        //reserved for get rid of the uncontrollable dodging when necessary
      }
    }
    if(rc_device_get_state(prc_dev, RC_S2_DOWN) == RM_OK)
    {
      //disbaled
      gimbal_pitch_disable(pgimbal);
      gimbal_yaw_disable(pgimbal);
    }

    if (get_offline_state() == 0)
    {
      gimbal_state_init(pgimbal);
    }

    auto_gimbal_adjust(pgimbal);

    yaw_angle_fdb_js = pgimbal->cascade[0].outer.get * 1000;
    yaw_angle_ref_js = pgimbal->cascade[0].outer.set * 1000;
    pit_angle_fdb_js = pgimbal->cascade[1].outer.get * 1000;
    pit_angle_ref_js = pgimbal->cascade[1].outer.set * 1000;

    yaw_spd_fdb_js = pgimbal->cascade[0].inter.get * 1000;
    yaw_spd_ref_js = pgimbal->cascade[0].inter.set * 1000;
    pit_spd_fdb_js = pgimbal->cascade[1].inter.get * 1000;
    pit_spd_ref_js = pgimbal->cascade[1].inter.set * 1000;
		
		//Eric Edited Debug the ecd of pitch and yaw
		yaw_ecd_angle_js = pgimbal->ecd_angle.yaw;
		pit_ecd_angle_js = pgimbal->ecd_angle.pitch;
		
		
		
    gimbal_imu_update(pgimbal);
    gimbal_execute(pgimbal);
    osDelayUntil(&period, 2);
  }
}

static int32_t gimbal_imu_update(void *argc)
{
  struct ahrs_sensor mpu_sensor;
  struct attitude mahony_atti;
  gimbal_t pgimbal = (gimbal_t)argc;
  mpu_get_data(&mpu_sensor);
  mahony_ahrs_updateIMU(&mpu_sensor, &mahony_atti);
  // TODO: adapt coordinates to our own design
  gimbal_pitch_gyro_update(pgimbal, mahony_atti.roll);
  gimbal_yaw_gyro_update(pgimbal, mahony_atti.yaw);
  gimbal_rate_update(pgimbal, -mpu_sensor.wy * RAD_TO_DEG, -mpu_sensor.wx * RAD_TO_DEG);
  // TODO: adapt coordinates to our own design
  
  mpu_pit = mahony_atti.pitch;
  mpu_yaw = mahony_atti.yaw;
  mpu_rol = mahony_atti.roll;
	mpu_wz = mpu_sensor.wz;
	mpu_wy = mpu_sensor.wy;
	mpu_wx = mpu_sensor.wx;
  
  return 0;
}

struct pid pid_imu_tmp;

static void imu_temp_ctrl_init(void)
{
  pid_struct_init(&pid_imu_tmp, 2000, 500, 1100, 10, 0);
}

static int32_t imu_temp_keep(void *argc)
{
  float temp;
  mpu_get_temp(&temp);
  pid_calculate(&pid_imu_tmp, temp, DEFAULT_IMU_TEMP);
  mpu_heat_output(pid_imu_tmp.out);
  return 0;
}

uint8_t auto_adjust_f;
volatile uint32_t pit_time, yaw_time;
uint32_t pit_cnt;
volatile uint16_t yaw_ecd_r, yaw_ecd_l;
volatile uint16_t pit_ecd_c, yaw_ecd_c;

void send_gimbal_current(int16_t iq1, int16_t iq2, int16_t iq3)
{
  static uint8_t tx_data[8];

  tx_data[0] = iq1 >> 8;
  tx_data[1] = iq1;
  tx_data[2] = iq2 >> 8;
  tx_data[3] = iq2;
  tx_data[4] = iq3 >> 8;
  tx_data[5] = iq3;

  can_msg_bytes_send(&hcan1, tx_data, 6, 0x1FF);
}

struct pid pid_pit = {0};
struct pid pid_pit_spd = {0};

/**Modified by Y.H. Liu
 * @Jun 20, 2019: adaption for hero
 * 
 * Automatically adjust the netural position for gimbal
 */
static void auto_gimbal_adjust(gimbal_t pgimbal)
{
  if (auto_adjust_f)
  {
    pit_ecd_c = pgimbal->motor[PITCH_MOTOR_INDEX].data.ecd;
    yaw_ecd_c = pgimbal->motor[YAW_MOTOR_INDEX].data.ecd;

    gimbal_save_data(yaw_ecd_c, pit_ecd_c);
    gimbal_set_offset(pgimbal, yaw_ecd_c, pit_ecd_c);
    auto_adjust_f = 0;
    __disable_irq();
    NVIC_SystemReset();
    while (1)
      ;
  }
}

void gimbal_auto_adjust_start(void)
{
  auto_adjust_f = 1;
}

uint8_t get_gimbal_init_state(void)
{
  return auto_init_f;
}

void gimbal_init_state_reset(void)
{
  ramp_init(&pitch_ramp, BACK_CENTER_TIME / GIMBAL_PERIOD);
  ramp_init(&yaw_ramp, BACK_CENTER_TIME / GIMBAL_PERIOD);
  auto_init_f = 0;
}

static void gimbal_state_init(gimbal_t pgimbal)
{
  if (auto_init_f == 0)
  {
    gimbal_set_pitch_mode(pgimbal, ENCODER_MODE);
    gimbal_set_yaw_mode(pgimbal, ENCODER_MODE);
    gimbal_yaw_disable(pgimbal);
    gimbal_set_pitch_angle(pgimbal, pgimbal->ecd_angle.pitch * (1 - ramp_calculate(&pitch_ramp)));

    if ((pgimbal->ecd_angle.pitch != 0) && (pgimbal->ecd_angle.yaw != 0))
    {
      if (fabs(pgimbal->ecd_angle.pitch) < 1.5f)
      {
        gimbal_yaw_enable(pgimbal);
				// Rotate from current angle to 0 read from ecd
        gimbal_set_yaw_angle(pgimbal, pgimbal->ecd_angle.yaw * (1 - ramp_calculate(&yaw_ramp)), 0);
        if (fabs(pgimbal->ecd_angle.yaw) < 1.5f)
        {
          auto_init_f = 1;
        }
      }
    }
  }
}

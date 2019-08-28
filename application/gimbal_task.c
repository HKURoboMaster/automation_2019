/***************************************************************************
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
#include "angle_queue.h"

#define DEFAULT_IMU_TEMP 50

/* patrol period time (ms) */
#define GIMBAL_PERIOD 2
/* gimbal back center time (ms) */
#define BACK_CENTER_TIME 3000
#define MAX_TRACK_ANGLE 35
#define LOST_THRESHOLD 200 // 400ms
#define SPEED_RATIO 1.0f


#define INTEGRAL_LIM 5

//Kalman filter related define
//#define ACC_KALMAN
#ifndef ACC_KALMAN
kalman_filter_init_t yaw_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},					// Co-variance Matrix
  .A_data = {1, 0.0028, 0, 1},			// Predict function Transfer parameter 1000Hz?
  .H_data = {1, 0, 0, 1},					// Measurement transfer parameter
  .Q_data = {1, 0, 0, 1},					// Co-variance of progress matrix
  .R_data = {200, 0, 0, 400}		// Co-Variance of Measurement Observe matrix.
};

kalman_filter_init_t pit_kalman_filter_para = 
{
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.0028, 0, 1},
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {400, 0, 0, 800}		// Basic Idea is kalman filter uses co-variance data.
};	// Only consider the variance instead of co-variance.
// Kalman Filter
#else
kalman_filter_3d_init_t yaw_kalman_filter_para = {
	.P_data = {2, 0, 0, 0, 2, 0, 0, 0, 2},
  .A_data = {1, 0.002, 0, 0, 1, 0.002, 0, 0, 1},
  .H_data = {1, 0, 0, 0, 1, 0, 0, 0, 1},
  .Q_data = {1, 0, 0, 0, 1, 0, 0, 0, 1},
  .R_data = {1000, 0, 0, 0, 2000, 0, 0, 0, 4500}		// Basic Idea is kalman filter uses co-variance data.
};

kalman_filter_3d_init_t pit_kalman_filter_para = 
{
  .P_data = {2, 0, 0, 0, 2, 0, 0, 0, 2},
  .A_data = {1, 0.002, 0, 0, 1, 0.002, 0, 0, 1},
  .H_data = {1, 0, 0, 0, 1, 0, 0, 0, 1},
  .Q_data = {1, 0, 0, 0, 1, 0, 0, 0, 1},
  .R_data = {2000, 0, 0, 0, 5000, 0, 0, 0, 11000}		// Basic Idea is kalman filter uses co-variance data.
};

kalman_filter_3d_t yaw_kalman_filter;
kalman_filter_3d_t pit_kalman_filter;


#endif

kalman_filter_t yaw_kalman_filter;
kalman_filter_t pit_kalman_filter;
uint32_t   gim_tim_ms = 0; 
uint32_t   gim_last_tim = 0;

#ifndef ACC_KALMAN
int kalman_yaw_js[2];
int kalman_pit_js[2];
#endif

#ifdef ACC_KALMAN 
int kalman_yaw_js[3];
int kalman_pit_js[3];
#endif


int pc_js1;
int pc_js2;
int lost_target_counter = 0;
float yaw_angle_raw = 0;
float pit_angle_raw = 0;

// Edited By Eric Chen 
// Complement the motion of gimbal itself when using kalman filtered motion prediction of object.

float ref_yaw_speed = 0;
float ref_pit_speed = 0;

float delta_pit_speed = 0;
float delta_yaw_speed = 0;

float yaw_speed = 0.0f;
float pit_speed = 0.0f;

float integral_error_yaw = 0;
float integral_error_pit = 0;

float yaw_i = 0.001;
float pit_i = 0.001;


// Eric Chen's Edition End.

int speed_calc_time=0;
int speed_calc_last_time=0;
typedef struct  // speed_calc_data_t
{
  int delay_cnt;
  int freq;
  int last_time;
  float last_position;
  float speed;
  float last_speed;
  float processed_speed;
} speed_calc_data_t;

// Acceleration typedef
typedef struct
{
	int delay_cnt;
	int freq;
	int last_time;
	float last_speed;
	float acceleration;
	float last_acceleration;
	float processed_acceleration;
}acc_calc_data_t;

speed_calc_data_t yaw_speed_struct;
speed_calc_data_t pit_speed_struct;

acc_calc_data_t yaw_acc_struct;
acc_calc_data_t pit_acc_struct;



static float yaw_speed_raw;
static float pit_speed_raw;

//static float yaw_acc_raw;
//static float pit_acc_raw;

float *yaw_kf_data;
float *pit_kf_data;


int pc_counter = 0;

float target_speed_calc(speed_calc_data_t *S,uint32_t time,float position);
float target_acc_calc(acc_calc_data_t *A,uint32_t time, float speed);
#define KALMAN
// Aimming related
extern float auto_aiming_pitch;
extern float auto_aiming_yaw;
extern uint32_t time_pc;
uint32_t time_last = 0;
// Function declaration
static void imu_temp_ctrl_init(void);
static int32_t gimbal_imu_update(void *argc);
static int32_t imu_temp_keep(void *argc);
static void auto_gimbal_adjust(gimbal_t pgimbal);
static void gimbal_state_init(gimbal_t pgimbal);

// Flags
uint8_t auto_adjust_f;
uint8_t auto_init_f;
/* control ramp parameter */
static ramp_t yaw_ramp = RAMP_GEN_DAFAULT;
static ramp_t pitch_ramp = RAMP_GEN_DAFAULT;
// Orientation debugging
int32_t mpu_pit, mpu_yaw, mpu_rol;
int32_t mpu_wx, mpu_wy, mpu_wz;
// PID debugging
int32_t yaw_angle_fdb_js, yaw_angle_ref_js;
int32_t pit_angle_fdb_js, pit_angle_ref_js;
int32_t yaw_spd_fdb_js, yaw_spd_ref_js;
int32_t pit_spd_fdb_js, pit_spd_ref_js;
int32_t yaw_ecd_angle_js, pit_ecd_angle_js;
/** Edited by Y.H. Liu
 * @Jun 12, 2019: modified the mode switch
 * @Jul 6, 2019: polar coordinate for mouse movement
 *
 *  Implement the customized control logic and FSM, details in Control.md
 */
void gimbal_task(void const *argument)
{
  // Edited By Eric Chen 2019.7.20
  //gim_tim_ms = HAL_GetTick() - gim_last_tim;  // For speed calculation
  //gim_last_tim = HAL_GetTick();
  uint32_t period = osKernelSysTick();
  rc_device_t prc_dev = NULL;
  rc_info_t prc_info = NULL;
  gimbal_t pgimbal = NULL;
  cali_sys_t *pparam = NULL;

  pgimbal = gimbal_find("gimbal");
  prc_dev = rc_device_find("can_rc");
  pparam = get_cali_param();
  
  /**Added by Y.H. Liu
   * @Jul 17, 2019: Define a queue for auto aiming
   * @Jul 19, 2019: Change the global variable to be the local ones
   * 
   * Use a queue to implement the speed mode of auto aiming
   */
  float yaw_autoaim_offset = 0.0f;
  float pitch_autoaim_offset = 0.0f;
  float pit_delta, yaw_delta;
  // Added By Eric Chen : Init Kalman filter params
  //#ifdef KALMAN
	yaw_kalman_filter_para.xhat_data[0] = 0;
	yaw_kalman_filter_para.xhat_data[1] = 0;
	pit_kalman_filter_para.xhat_data[0] = 0;
	pit_kalman_filter_para.xhat_data[1] = 0;
  kalman_filter_init(&yaw_kalman_filter,&yaw_kalman_filter_para);
  kalman_filter_init(&pit_kalman_filter,&pit_kalman_filter_para);
  //#else
  struct angle_queue yawQ;
  struct angle_queue pitQ; 
  queue_init(&yawQ);
  queue_init(&pitQ);
	//#endif
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
		#ifdef KALMAN
    // Added By Eric Chen: Enable Kalman filter function
    gim_tim_ms = HAL_GetTick() - gim_last_tim;  // For speed calculation
    gim_last_tim = HAL_GetTick();
    // Eric Chen Edition End.
		
		#ifdef ACC_KALMAN
		mat_init(*yaw_kalman_filter.Q,3,3,yaw_kalman_filter_para.Q_data);
		mat_init(*pit_kalman_filter.Q,3,3,pit_kalman_filter_para.Q_data);
		
		#else
    mat_init(&yaw_kalman_filter.Q,2,2,yaw_kalman_filter_para.Q_data);
    mat_init(&pit_kalman_filter.Q,2,2,pit_kalman_filter_para.Q_data);
		#endif
		//Each time update
    #endif
		// The reason using a queue is : Make a 10 ms delay when receive the data;
		// Since each 10 ms the data will be updated from PC
		// Encoder angle will be enqueue.each time and the encoder
		//#ifndef KALMAN
    if(yawQ.len>=DELAY)
    {
      do
      {
        yaw_autoaim_offset = pgimbal->ecd_angle.yaw - deQueue(&yawQ);
      }while(yawQ.len>=DELAY);
    }
    if(pitQ.len>=DELAY)
    {
      do
      {
        pitch_autoaim_offset = pgimbal->ecd_angle.pitch - deQueue(&pitQ);
      }while(pitQ.len>=DELAY);
    }
		//#endif
    if(rc_device_get_state(prc_dev, RC_S2_DOWN2MID) == RM_OK)
    {
      //switched out disabled mode
      gimbal_pitch_enable(pgimbal);
      gimbal_yaw_enable(pgimbal);
    }
    if (rc_device_get_state(prc_dev, RC_S2_UP) == RM_OK || rc_device_get_state(prc_dev, RC_S2_MID) == RM_OK
    ||  rc_device_get_state(prc_dev, RC_S2_MID2UP) == RM_OK || rc_device_get_state(prc_dev,RC_S2_UP2MID == RM_OK))
    {
      //manual control mode i.e. chassis follow gimbal
      if(prc_info->kb.bit.X != 1)
      {
        //auto_aimming
				#ifndef KALMAN
        if(prc_info->mouse.r || rc_device_get_state(prc_dev, RC_S2_UP) == RM_OK)
        {
          if(auto_aiming_pitch!=0)
            gimbal_set_yaw_delta(pgimbal, auto_aiming_yaw-yaw_autoaim_offset);
          auto_aiming_pitch = 0;	// Make sure the only data sent by pc will be used
          auto_aiming_yaw = 0;
        }
				#endif
				// Auto aiming kalman testing
				// PC data toggle
				#ifdef KALMAN
				
				pc_js1 = (int)(auto_aiming_yaw*1000);
				pc_js2 = (int)(auto_aiming_pitch*1000);
				
				if(time_pc != time_last)
				{
					// Update the Time sent From PC.
					// For checking wheather PC sent data to me.
					time_last = time_pc;
					// Using STM32 Clock to calculate the time interval between 2 measurement.
					speed_calc_time = HAL_GetTick() - time_last;
					speed_calc_last_time = HAL_GetTick();
					
					yaw_angle_raw = auto_aiming_yaw;
					pit_angle_raw = auto_aiming_pitch;
					// Unit Degree per second
					ref_yaw_speed = pgimbal->sensor.rate.yaw_rate;
					ref_pit_speed = pgimbal->sensor.rate.pitch_rate;
					// By default the time input is time Gap in ms
					// Unit Degree Per second
          yaw_speed_raw = target_speed_calc(&yaw_speed_struct,speed_calc_time,yaw_angle_raw);
				  pit_speed_raw = target_speed_calc(&pit_speed_struct,speed_calc_time,pit_angle_raw); 
				}
				delta_pit_speed = pgimbal->sensor.rate.pitch_rate - ref_pit_speed;
				delta_yaw_speed = pgimbal->sensor.rate.yaw_rate - ref_yaw_speed;
				// If they share same ratio
				// For CV the angular ratio is uncertain.
				// BUG Fixed
				pit_speed = pit_speed_raw ;//-  delta_pit_speed;
				yaw_speed = yaw_speed_raw ;//-  delta_yaw_speed;
				// Since speed data is irrelavant from historical data.
				// Each time speed need to delta the speed of gimbal compared with
				// The time when reference was settled
				#ifdef ACC_KALMAN
				yaw_acc_raw d= target_acc_calc(&yaw_acc_struct,speed_calc_time/1000,yaw_speed_raw);
				pit_acc_raw = target_acc_calc(&pit_acc_struct,speed_calc_time/1000,pit_speed_raw);
				yaw_kf_data = kalman_filter_3d_calc(&yaw_kalman_filter,yaw_angle_raw,yaw_speed_raw,yaw_acc_raw);
				pit_kf_data = kalman_filter_3d_calc(&pit_kalman_filter,pit_angle_raw,pit_speed_raw,pit_acc_raw);
				kalman_pit_js[0] = pit_kf_data[0];
				kalman_pit_js[1] = pit_kf_data[1];
				kalman_pit_js[2] = pit_kf_data[2];
				
				kalman_yaw_js[0] = yaw_kf_data[0];
				kalman_yaw_js[1] = yaw_kf_data[1];
				kalman_yaw_js[2] = yaw_kf_data[2];
				#else
				
				//yaw_angle_raw += yaw_speed*gim_tim_ms/1000;
        //pit_angle_raw += pit_speed*gim_tim_ms/1000;
				yaw_kf_data = kalman_filter_calc(&yaw_kalman_filter,yaw_angle_raw,yaw_speed);
				pit_kf_data = kalman_filter_calc(&pit_kalman_filter,pit_angle_raw,pit_speed);
				kalman_yaw_js[0] = (int)((yaw_kf_data[0] + yaw_kf_data[1]*0.123f)*1000);
				kalman_yaw_js[1] = (int)(yaw_kf_data[1]*1000);
				kalman_pit_js[0] = (int)((pit_kf_data[0]+pit_kf_data[1]*0.1f)*1000);
				kalman_pit_js[1] = (int)(pit_kf_data[1]*1000);
				#endif

				if(prc_info->mouse.r || rc_device_get_state(prc_dev,RC_S2_UP)==RM_OK)
				{
					
					// The reason to implement PC counter is that kalman filter need time to converge
					// The PC_counter make it possible to converge.
					if(pc_counter==200)
					{
					
					// Equavalent to P only control. Need a I term.
					// Set angle speed is no matter what set the difference of angle
					if((yaw_kf_data[0] - yaw_autoaim_offset + yaw_kf_data[1]*0.273f)>3.0f)
						gimbal_set_yaw_speed(pgimbal,0.12f*(yaw_kf_data[0] + yaw_kf_data[1]*0.123f));
					else
						gimbal_set_yaw_speed(pgimbal,0.07f*(yaw_kf_data[0] + yaw_kf_data[1]*0.273f));
				  //gimbal_set_yaw_speed(pgimbal,0.1*yaw_kf_data[0]);
					if((pit_kf_data[0] - pitch_autoaim_offset + pit_kf_data[1]*0.273f)>3.0f)
						gimbal_set_pitch_speed(pgimbal,0.12f*(pit_kf_data[0] + pit_kf_data[1]*0.15f));
					else
						gimbal_set_pitch_speed(pgimbal,0.07f*(pit_kf_data[0] + pit_kf_data[1]*0.273f));
					}
					else
						pc_counter++;
				}
				else
				{
					pc_counter = 0;
				}
				#endif
				//kalman_over:lost_target_counter = lost_target_counter;
				float square_ch1 = (float)prc_info->ch1 * abs(prc_info->ch1) / RC_CH_SCALE;
        /*-------- Map mouse coordinates into polar coordiantes --------*/
        int16_t yaw_mouse,pit_mouse;
        int16_t radius = (int16_t)sqrt(prc_info->mouse.y * prc_info->mouse.y + prc_info->mouse.x * prc_info->mouse.x);
        int16_t tanTheta = prc_info->mouse.y / prc_info->mouse.x;
        if(tanTheta<0.8 && tanTheta>-0.8)
        {
          yaw_mouse = radius * prc_info->mouse.x / abs(prc_info->mouse.x);
          pit_mouse = 0;
        }
        else if(tanTheta>1.2 || tanTheta<-1.2)
        {
          yaw_mouse = 0;
          pit_mouse = radius * prc_info->mouse.y / abs(prc_info->mouse.y);
        }
        else
        {
          yaw_mouse = prc_info->mouse.x;
          pit_mouse = prc_info->mouse.y;
        }
        

        gimbal_set_yaw_mode(pgimbal, GYRO_MODE);
        pit_delta = (float)prc_info->ch2 * GIMBAL_RC_PITCH + 1 *  (float)pit_mouse * GIMBAL_MOUSE_PITCH;
        yaw_delta =     square_ch1       * GIMBAL_RC_YAW   +       (float)yaw_mouse * GIMBAL_MOUSE_YAW;
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
        if(prc_info->kb.bit.Z)
        {
          if(!prc_info->kb.bit.CTRL)
          {
            pgimbal->param.yaw_ecd_center += ((float)prc_info->wheel/RC_CH_SCALE);
            gimbal_set_offset(pgimbal, pgimbal->param.yaw_ecd_center, pgimbal->param.pitch_ecd_center);
          }
          else
          {
            mpu_manual_cali(0, 2*prc_info->wheel/RC_CH_SCALE, 0);
          }
        }
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
		
		
		
    enQueue(&yawQ, pgimbal->ecd_angle.yaw);
    enQueue(&pitQ, pgimbal->ecd_angle.pitch);
    gimbal_imu_update(pgimbal);
    gimbal_execute(pgimbal);
    // The period is calculated from very beginning.
    // If time excess then do this task. Fake real time.
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

  gimbal_pitch_gyro_update(pgimbal, -mahony_atti.roll);
  gimbal_yaw_gyro_update(pgimbal, -mahony_atti.yaw);
  gimbal_rate_update(pgimbal, mpu_sensor.wy * RAD_TO_DEG, -mpu_sensor.wx * RAD_TO_DEG);
  
  mpu_pit = mahony_atti.pitch * 1000;
  mpu_yaw = mahony_atti.yaw   * 1000;
  mpu_rol = mahony_atti.roll  * 1000;
	mpu_wz = mpu_sensor.wz * 1000;
	mpu_wy = mpu_sensor.wy * 1000;
	mpu_wx = mpu_sensor.wx * 1000;
  
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
uint32_t pit_timeout_cnt=0;
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
 * @Jul 8, 2019: use the original methods to calculate the pitch centre
 * @Jul 17, 2019: use a queue for auto-aiming adaption
 * 
 * Automatically adjust the netural position for gimbal
 */
static void auto_gimbal_adjust(gimbal_t pgimbal)
{
  if (auto_adjust_f)
  {
    pid_struct_init(&pid_pit, 2000, 0, 30, 0.001, 0);
    pid_struct_init(&pid_pit_spd, 30000, 8000, 200, 0, 0);
    while (1)
    {
      gimbal_imu_update(pgimbal);
      pid_calculate(&pid_pit, -pgimbal->sensor.gyro_angle.pitch, -90);
      pid_calculate(&pid_pit_spd, -pgimbal->sensor.rate.pitch_rate, pid_pit.out);

      send_gimbal_current(0, pid_pit_spd.out, 0);

      HAL_Delay(2);

      if ((fabs(pgimbal->sensor.gyro_angle.pitch-85) < 0.1))
      {
        pit_cnt++;
        pit_timeout_cnt = 0;
      }
      else
      {
        pit_cnt = 0;
        pit_timeout_cnt++;
      }
      if (pit_cnt > 1000 || pit_timeout_cnt>2000)
      {
        pit_ecd_c = pgimbal->motor[PITCH_MOTOR_INDEX].data.ecd;
        break;
      }
    }
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

/* Modified By Eric Chen.
 * Changed real time into time gap to for adaptation
 */
float speed_threshold = 10.0f;
float target_speed_calc(speed_calc_data_t *S, uint32_t time, float position)
{
  //S->delay_cnt++;
	// This time speed calculate in MS per second.
	S->speed = ((position - S->last_position) / time) * 1000; 
  //S->speed = (position - S->last_position) / (time - S->last_time) * 1000;
#if 1
  if ((S->speed - S->processed_speed) < -speed_threshold)
  {
      S->processed_speed = S->processed_speed - speed_threshold;
  }
  else if ((S->speed - S->processed_speed) > speed_threshold)
  {
      S->processed_speed = S->processed_speed + speed_threshold;
  }
  else 
#endif
  S->processed_speed = S->speed;
    
  S->last_time = time;
  S->last_position = position;
  S->last_speed = S->speed;
  S->delay_cnt = 0;
	// Since we have implemented lost connection checking, delay mechinism is improper.
	
  //if(S->delay_cnt > 200) // delay 200ms speed = 0
  //{
  //  S->processed_speed = 0;
  //}
  return S->processed_speed;
}

float acc_threshold = 10.0f;

float target_acc_calc(acc_calc_data_t *A,uint32_t time, float speed)
{
  A->delay_cnt++;

  if (time != A->last_time)
  {
    A->acceleration = (speed - A->last_speed) / (time - A->last_time) * 1000;
#if 1
    if ((A->acceleration - A->processed_acceleration) < -acc_threshold)
    {
        A->processed_acceleration = A->processed_acceleration - acc_threshold;
    }
    else if ((A->acceleration - A->processed_acceleration) > acc_threshold)
    {
        A->processed_acceleration = A->processed_acceleration + acc_threshold;
    }
    else 
#endif
      A->processed_acceleration = A->acceleration;
    
    A->last_time = time;
    A->last_acceleration = A->acceleration;
    A->last_acceleration = A->acceleration;
    A->delay_cnt = 0;
  }
  
  if(A->delay_cnt > 200) // delay 200ms speed = 0
  {
    A->processed_acceleration = 0;
  }

  return A->processed_acceleration;
}


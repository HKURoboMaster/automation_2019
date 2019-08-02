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

#include "shoot.h"
#include "drv_io.h"

static uint8_t trigger_motor_status(struct shoot * shoot);
static int32_t shoot_pid_input_convert(struct controller *ctrl, void *input);
static int32_t shoot_fric_ctrl(struct shoot *shoot);
static int32_t shoot_cmd_ctrl(struct shoot *shoot);
static int32_t shoot_block_check(struct shoot *shoot);

//Leo starts
int32_t shoot_pid_register2(struct shoot *shoot, const char *name, enum device_can can)
{
  char motor_name[OBJECT_NAME_MAX_LEN] = {0};
  uint8_t name_len;

  int32_t err;

  if (shoot == NULL)
    return -RM_INVAL;
  if (shoot_find(name) != NULL)
    return -RM_EXISTED;

  object_init(&(shoot->parent), Object_Class_Shoot, name);

  name_len = strlen(name);

  if (name_len > OBJECT_NAME_MAX_LEN / 2)
  {
    name_len = OBJECT_NAME_MAX_LEN / 2;
  }

  memcpy(&motor_name, name, name_len);
  shoot->motor.can_periph = can;
  shoot->motor.can_id = 0x208; 
  shoot->motor.init_offset_f = 1; 

  shoot->ctrl.convert_feedback = shoot_pid_input_convert;

  pid_struct_init(&(shoot->motor_pid), 15000, 7500, 10, 0.3, 0); 

  shoot->param.block_current = BLOCK_CURRENT_DEFAULT;
  shoot->param.block_speed = BLOCK_SPEED_DEFAULT;
  shoot->param.block_timeout = BLOCK_TIMEOUT_DEFAULT;
  shoot->param.turn_speed = TURN_SPEED_DEFAULT;
  shoot->param.check_timeout = BLOCK_CHECK_TIMEOUT_DEFAULT;
	shoot->fric_spd[0] = FRIC_MIN_SPEED;
  shoot->fric_spd[1] = FRIC_MIN_SPEED;

  memcpy(&motor_name[name_len], "_TURN\0", 6);

  err = motor_device_register(&(shoot->motor), motor_name, 0);
  if (err != RM_OK)
    goto end;

  memcpy(&motor_name[name_len], "_CTL\0", 7);

  err = pid_controller_register(&(shoot->ctrl), motor_name, &(shoot->motor_pid), &(shoot->motor_feedback), 1);
  if (err != RM_OK)
    goto end;

  shoot_state_update(shoot);

  return RM_OK;
end:
  object_detach(&(shoot->parent));

  return err;
}
//Leo ends
int32_t shoot_pid_register(struct shoot *shoot, const char *name, enum device_can can)
{
  char motor_name[OBJECT_NAME_MAX_LEN] = {0};
  uint8_t name_len;

  int32_t err;

  if (shoot == NULL)
    return -RM_INVAL;
  if (shoot_find(name) != NULL)
    return -RM_EXISTED;

  object_init(&(shoot->parent), Object_Class_Shoot, name);

  name_len = strlen(name);

  if (name_len > OBJECT_NAME_MAX_LEN / 2)
  {
    name_len = OBJECT_NAME_MAX_LEN / 2;
  }

  memcpy(&motor_name, name, name_len);
  shoot->motor.can_periph = can;
  shoot->motor.can_id = 0x207;
  shoot->motor.init_offset_f = 1;

  shoot->ctrl.convert_feedback = shoot_pid_input_convert;

  pid_struct_init(&(shoot->motor_pid), 15000, 7500, 10, 0.3, 0);

  shoot->param.block_current = BLOCK_CURRENT_DEFAULT;
  shoot->param.block_speed = BLOCK_SPEED_DEFAULT;
  shoot->param.block_timeout = BLOCK_TIMEOUT_DEFAULT;
  shoot->param.turn_speed = TURN_SPEED_DEFAULT;
  shoot->param.check_timeout = BLOCK_CHECK_TIMEOUT_DEFAULT;

  shoot->fric_spd[0] = FRIC_MIN_SPEED;
  shoot->fric_spd[1] = FRIC_MIN_SPEED;

  memcpy(&motor_name[name_len], "_TURN\0", 6);

  err = motor_device_register(&(shoot->motor), motor_name, 0);
  if (err != RM_OK)
    goto end;

  memcpy(&motor_name[name_len], "_CTL\0", 7);

  err = pid_controller_register(&(shoot->ctrl), motor_name, &(shoot->motor_pid), &(shoot->motor_feedback), 1);
  if (err != RM_OK)
    goto end;

  shoot_state_update(shoot);
  controller_disable(&(shoot->ctrl));

  return RM_OK;
end:
  object_detach(&(shoot->parent));

  return err;
}
/**Edited by Y.H. Liu
 * @Jul 8, 2019: Change the slope to the function: shoot_fric_ctrl
 * 
 * Set the target speed of the trigger motor
 */
int32_t shoot_set_fric_speed(struct shoot *shoot, uint16_t fric_spd1, uint16_t fric_spd2)
{
  if (shoot == NULL)
    return -RM_INVAL;
  shoot->target.fric_spd[0] = fric_spd1;
  shoot->target.fric_spd[1] = fric_spd2;

	return RM_OK;
}

int32_t shoot_get_fric_speed(struct shoot *shoot, float *fric_spd1, float *fric_spd2)
{
  if (shoot == NULL)
    return -RM_INVAL;
  uint16_t fric_pwm_1, fric_pwm_2;
  fric_get_speed(&fric_pwm_1, &fric_pwm_2);
  *fric_spd1 = *fric_spd1 - (int16_t)*fric_spd1;
  *fric_spd2 = *fric_spd2 - (int16_t)*fric_spd2;
  *fric_spd1 += fric_pwm_1;
  *fric_spd2 += fric_pwm_2;
  return RM_OK;
}

/**Edited by Y.H. Liu
 * @Jun 13, 2019: Count the bullets
 * @Jul 4, 2019: Not count the bullets
 * 
 * Receive the command from shoot_task.c
 */
int32_t shoot_set_cmd(struct shoot *shoot, uint8_t cmd, uint32_t shoot_num)
{
  if (shoot == NULL)
    return -RM_INVAL;

  shoot->cmd = cmd;
  /*------ No matter what command it is, always count the bullets ------*/
  // if (cmd == SHOOT_ONCE_CMD)
  // {
  shoot->target.shoot_num = shoot->shoot_num + shoot_num;
  // }

  return RM_OK;
}

int32_t shoot_execute(struct shoot *shoot)
{
  float motor_out;
  struct motor_data *pdata;

  if (shoot == NULL)
    return -RM_INVAL;

  shoot_fric_ctrl(shoot);
  // shoot_block_check(shoot); Inside the shoot_cmd_ctrl and inside state_update
  shoot_cmd_ctrl(shoot);

  pdata = motor_device_get_data(&(shoot->motor));
	
  controller_set_input(&(shoot->ctrl), shoot->target.motor_speed);
  controller_execute(&(shoot->ctrl), (void *)pdata);
  controller_get_output(&(shoot->ctrl), &motor_out);

  motor_device_set_current(&shoot->motor, (int16_t)motor_out);

  return RM_OK;
}

/**Edited by Y.H. Liu
 * @Jun 13, 2019: Count the bullet
 * @Jun 13, 2019: Bypass the trigger switch
 * @Jul 4, 2019: Change the FSM
 * 
 * Detect whether a bullet has been shot and then update the state
 */
int32_t shoot_state_update(struct shoot *shoot)
{
  if (shoot == NULL)
    return -RM_INVAL;

  // shoot->trigger_key = get_trig_status();
  shoot->trigger_key = trigger_motor_status(shoot);
  /*------ Use the encoder of the trigger motor to bypass the switch ------*/
  switch(shoot->state)
  {
    case SHOOT_READY: 
      if(shoot->cmd != SHOOT_STOP_CMD) // One bullet to be shot
        shoot->state = SHOOT_INIT;
      //else, remain in ready
      break;
    case SHOOT_INIT:
      if(shoot->cmd == SHOOT_ONCE_CMD)
        shoot->cmd = SHOOT_STOP_CMD;
      if(shoot->trigger_key == TRIG_BOUNCE_UP) //One bullet in chamber
        shoot->state = SHOOT_RUNNING;
      //else, remain in INIT
      break;
    case SHOOT_RUNNING:
      if(shoot->trigger_key == TRIG_PRESS_DOWN) // One bullet away
      {
        shoot->state = SHOOT_READY;
        shoot->shoot_num += 1;
      }
      //else, remain in RUNNING
      break;
    default: 
      shoot->state = SHOOT_READY;
  }
  return RM_OK;
}

int32_t shoot_set_turn_speed(struct shoot *shoot, uint16_t speed)
{
  if (shoot == NULL)
    return -RM_INVAL;
  VAL_LIMIT(speed, 1000, 2500);
  shoot->param.turn_speed = speed;

  return RM_OK;
}

shoot_t shoot_find(const char *name)
{
  struct object *object;

  object = object_find(name, Object_Class_Shoot);

  return (shoot_t)object;
}

int32_t shoot_enable(struct shoot *shoot)
{
  if (shoot == NULL)
    return -RM_INVAL;

  controller_enable(&(shoot->ctrl));

  return RM_OK;
}

int32_t shoot_disable(struct shoot *shoot)
{
  if (shoot == NULL)
    return -RM_INVAL;
  shoot_set_fric_speed(shoot, 0, 0);
  controller_disable(&(shoot->ctrl));

  return RM_OK;
}

static int32_t shoot_block_check(struct shoot *shoot)
{
  static uint8_t first_block_f = 0;
  static uint32_t check_time;
  
  if (shoot == NULL)
    return -RM_INVAL;

  if (shoot->motor.current > shoot->param.block_current)
  {
    if (first_block_f == 0)
    {
      first_block_f = 1;
      check_time = get_time_ms();
    }
    else if(get_time_ms() - check_time > shoot->param.check_timeout)
    {
      first_block_f = 0;
      shoot->block_time = get_time_ms();
      shoot->state = SHOOT_BLOCK;
    }
  }
  else
  {
    first_block_f = 0;
  }
  
  return RM_OK;
}

/**Edited by Y.H. Liu
 * @Jun 13, 2019: Count the bullets and if too many are shot, stop shooting
 * @Jul 4, 2019: Simpify the FSM output
 * @Jul 24, 2019: change the state-updating sequence
 * 
 * Set the controlling signals for the trigger motor
 */
static int32_t shoot_cmd_ctrl(struct shoot *shoot)
{
  if (shoot == NULL)
    return -RM_INVAL;
  
  shoot_block_check(shoot);
  if(shoot->state == SHOOT_BLOCK)
  {
    shoot->target.motor_speed = shoot->param.block_speed;
    if (get_time_ms() - shoot->block_time > shoot->param.block_timeout)
    {
      shoot_state_update(shoot);
    }
  }
  else
  {
    shoot_state_update(shoot); //update according to cmd and trigger status
    if (shoot->state == SHOOT_INIT || shoot->state == SHOOT_RUNNING) //start to shoot, trigger motor running
    {
      shoot->target.motor_speed = shoot->param.turn_speed;
    }
    else if (shoot->state == SHOOT_READY) //ready for the next bullet, tirgger motor stopped
    {
      shoot_state_update(shoot); //update according to cmd and trigger status
      shoot->target.motor_speed = 0;
    }
    else
      return RM_INVAL;
  }
 
	if(shoot->fric_spd[0] < (FRIC_MAX_SPEED+FRIC_MIN_SPEED)/2 || shoot->fric_spd[1] < (FRIC_MAX_SPEED+FRIC_MIN_SPEED)/2)
	{
		controller_disable(&(shoot->ctrl));
	}
	
  return RM_OK;
}

/**Edited by Y.H. Liu
 * @Jul 8, 2019: Slow down the speeding up
 * @Jul 30, 2019: Use strlen to determine whether the given shooter is shoot2
 * 
 * Send the friction wheel motor signals by PWM
 */
static int32_t shoot_fric_ctrl(struct shoot *shoot)
{
  if (shoot == NULL)
    return -RM_INVAL;

  shoot_get_fric_speed(shoot, &shoot->fric_spd[0], &shoot->fric_spd[1]);

  if (shoot->target.fric_spd[0] != shoot->fric_spd[0])
  {
    if (shoot->target.fric_spd[0] < shoot->fric_spd[0])
    {
      if(shoot->fric_spd[0]>shoot->target.fric_spd[0]-15)
        shoot->fric_spd[0] -= 0.03125f;
      else
        shoot->fric_spd[0] -= 1;
    }
    else
    {
      shoot->fric_spd[0] += 0.25f;
    }
  }
  if (shoot->target.fric_spd[1] != shoot->fric_spd[1])
  {
    if (shoot->target.fric_spd[1] < shoot->fric_spd[1])
    {
      if(shoot->fric_spd[1]>shoot->target.fric_spd[1]-15)
        shoot->fric_spd[1] -= 0.03125f;
      else
        shoot->fric_spd[1] -= 1;
    }
    else
    {
      shoot->fric_spd[1] += 0.25f;
    }
  }

  VAL_LIMIT(shoot->fric_spd[0], FRIC_STOP_SPEED, FRIC_MAX_SPEED);
  VAL_LIMIT(shoot->fric_spd[1], FRIC_STOP_SPEED, FRIC_MAX_SPEED);

  if(5==strlen(shoot->parent.name))
	  fric_set_output((uint16_t)shoot->fric_spd[0], (uint16_t)shoot->fric_spd[1]);
	return RM_OK;
}

static int32_t shoot_pid_input_convert(struct controller *ctrl, void *input)
{
  pid_feedback_t pid_fdb = (pid_feedback_t)(ctrl->feedback);
  motor_data_t data = (motor_data_t)input;
  pid_fdb->feedback = data->speed_rpm;

  return RM_OK;
}
/**Added by Y.H. Liu
 * @Jun 13, 2019: Define the function
 * @Jul 4, 2019: Change the threashold and using abs value
 * @Jul 24, 2019: Simplify the control logic
 * 
 * Replace the trigger switch by the total angle of the trigger motor
 */
static uint8_t trigger_motor_status(struct shoot * shoot)
{
  #ifndef HERO_ROBOT
  int32_t  bullet_passing_offset = ((shoot->motor.data.total_angle/36)%360)%45;
  bullet_passing_offset = abs(bullet_passing_offset);
  if(bullet_passing_offset>=5 && bullet_passing_offset<40 && shoot->motor.data.ecd!=shoot->motor.data.last_ecd)
    return TRIG_BOUNCE_UP;
  else 
    return TRIG_PRESS_DOWN;
  #else
  int32_t  bullet_passing_offset =((shoot->motor.data.total_angle/36)%360)%72;
  bullet_passing_offset = abs(bullet_passing_offset);
  if(bullet_passing_offset>=5 && bullet_passing_offset<67 && shoot->motor.data.ecd!=shoot->motor.data.last_ecd)
    return TRIG_BOUNCE_UP;
  else 
    return TRIG_PRESS_DOWN;
  #endif
}

/**Edited by Y.H Liu
 * @Jun 15, 2019: declare the functions
 * 
 * Control the laser. 
 * @param cmd 0----laser off
 *            1----laser on
 */
int32_t laser_cmd(uint8_t cmd)
{
  if(cmd)
    WRITE_HIGH_LASER();
  else
    WRITE_LOW_LASER();
  return 0;
}

/**Edited by Y.H Liu
 * @Jun 15, 2019: declare the functions
 * 
 * Control the laser. 
 * @param cmd 0----laser off
 *            1----laser on
 */
int32_t magazine_lid_cmd(uint8_t cmd)
{
  if(cmd)
    MAGA_SERVO = 45;
  else
    MAGA_SERVO = 170;
  return 0;
}

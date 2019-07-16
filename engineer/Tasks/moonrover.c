#include "moonrover.h"
#include "motor.h"
#include "mecanum.h"
#include "pid_controller.h"
#include "engineer.h"
#include "chassis.h"
#include "dbus.h"

/* VARIABLES: MOONROVER - related */
extern Engineer engg;
/* END of VARIABLES: MOONROVER - related */

/* FUNCTIONS: MOONROVER - related */
static int32_t motor_pid_input_convert(struct controller *ctrl, void *input);

int32_t moonrover_pid_register(Engineer* engineer, const char *name, enum device_can can)
{
  char motor_name[2][OBJECT_NAME_MAX_LEN] = {0};
  uint8_t name_len;
  int32_t err;
  if (engineer == NULL)
    return -RM_INVAL;
  name_len = strlen(name);
  if (name_len > OBJECT_NAME_MAX_LEN / 2)
  {
    name_len = OBJECT_NAME_MAX_LEN / 2;
  }
  for (int i = MOONROVER_OFFSET + 0; i < MOONROVER_OFFSET + 2; i++)
  {
    memcpy(&motor_name[i], name, name_len);
    engineer->motor[i].can_periph = can;
    engineer->motor[i].can_id = 0x205 + i;
    engineer->motor[i].init_offset_f = 1;

    engineer->ctrl[i].convert_feedback = motor_pid_input_convert;
    pid_struct_init(&engineer->motor_pid[i], 15000, 500, 6.5f, 0.1, 0);
  }

  engineer->mecanum.param.wheel_perimeter = PERIMETER;
  engineer->mecanum.param.wheeltrack = WHEELTRACK;
  engineer->mecanum.param.wheelbase = WHEELBASE;
  engineer->mecanum.param.rotate_x_offset = ROTATE_X_OFFSET;
  engineer->mecanum.param.rotate_y_offset = ROTATE_Y_OFFSET;

  memcpy(&motor_name[0][name_len], "_MR\0", 4);
  memcpy(&motor_name[1][name_len], "_ML\0", 4);

  for (int i = MOONROVER_OFFSET + 0; i < MOONROVER_OFFSET + 2; i++)
  {
    err = motor_device_register(&(engineer->motor[i]), motor_name[i], 0);
    if (err != RM_OK) {
      // error handler
		}
  }

  memcpy(&motor_name[0][name_len], "_CTLMR\0", 7);
  memcpy(&motor_name[1][name_len], "_CTLML\0", 7);

  for (int i = MOONROVER_OFFSET + 0; i < MOONROVER_OFFSET + 2; i++)
  {
    err = pid_controller_register(&(engineer->ctrl[i]), motor_name[i],
																  &(engineer->motor_pid[i - MOONROVER_OFFSET]),
																	&(engineer->motor_feedback[i - MOONROVER_OFFSET]), 1);
    if (err != RM_OK) {
      // error handler
		}
  }

  return RM_OK;
}

int32_t moonrover_execute(Engineer* engineer, chassis_t pchassis, rc_device_t prc_dev, rc_info_t prc_info) {
  if (engineer == NULL)
    return -RM_INVAL;
  	
	if (engineer->ENGINEER_BIG_STATE == LOWERPART && engineer->ENGINEER_SMALL_STATE == CHASSIS &&
			(rc_device_get_state(prc_dev, RC_WHEEL_DOWN) == RM_OK && prc_dev->last_rc_info.wheel > -300)) {
		engineer->mecanum.speed.vx = pchassis->mecanum.speed.vx;
    engineer->mecanum.speed.vy = pchassis->mecanum.speed.vy;
    engineer->mecanum.speed.vw = pchassis->mecanum.speed.vw;
		
		float motor_out;
		struct motor_data *pdata;
		struct mecanum_motor_fdb wheel_fdb[2];
		mecanum_calculate(&(engineer->mecanum));
		motor_out = ROTATION_SPEED;
		for (int i = MOONROVER_OFFSET + 0; i < MOONROVER_OFFSET + 2; i++)
		{
			pdata = motor_device_get_data(&(engineer->motor[i]));

			wheel_fdb[i - MOONROVER_OFFSET].total_ecd = pdata->total_ecd;
			wheel_fdb[i - MOONROVER_OFFSET].speed_rpm = pdata->speed_rpm;
		
			controller_set_input(&engineer->ctrl[i], engineer->mecanum.wheel_rpm[i - MOONROVER_OFFSET]);
			controller_execute(&engineer->ctrl[i], (void *)pdata);
			controller_get_output(&engineer->ctrl[i], &motor_out);

			motor_device_set_current(&engineer->motor[i], (int16_t)motor_out);
		}
		mecanum_position_measure(&(engineer->mecanum), wheel_fdb);
	}
	
  return RM_OK;
}

static int32_t motor_pid_input_convert(struct controller *ctrl, void *input)
{
  pid_feedback_t pid_fdb = (pid_feedback_t)(ctrl->feedback);
  motor_data_t data = (motor_data_t)input;
  pid_fdb->feedback = data->speed_rpm;

  return RM_OK;
}

int32_t moonrover_enable(Engineer* engineer)
{
  if (engineer == NULL)
    return -RM_INVAL;

  for (int i = MOONROVER_OFFSET + 0; i < MOONROVER_OFFSET + 2; i++)
  {
    controller_enable(&(engineer->ctrl[i])); 
  }

  return RM_OK;
}

int32_t moonrover_disable(Engineer* engineer)
{
  if (engineer == NULL)
    return -RM_INVAL;

  for (int i = MOONROVER_OFFSET + 0; i < MOONROVER_OFFSET + 2; i++)
  {
    controller_disable(&(engineer->ctrl[i])); 
  }

  return RM_OK;
}
/* END of FUNCTIONS: MOONROVER - related */

/* RTOS: MOONROVER - related */
void moonrover_task(void const *argument)
{
  uint32_t period = osKernelSysTick();
	
	chassis_t pchassis = NULL;
	pchassis = chassis_find("chassis");
	
	rc_device_t prc_dev = NULL;
	rc_info_t prc_info = NULL;
	
	prc_dev = rc_device_find("uart_rc");
  if (prc_dev != NULL)
  {
    prc_info = rc_device_get_info(prc_dev);
  }
	
	for(;;) {
		moonrover_execute(&engg, pchassis, prc_dev, prc_info);
    osDelayUntil(&period, 2);
	}
}
/* END of RTOS: MOONROVER - related */

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
  for (int i = 0; i < 2; i++)
  {
    memcpy(&motor_name[i], name, name_len);
    engineer->motor[i].can_periph = can;
    engineer->motor[i].can_id = 0x205 + MOONROVER_OFFSET + i;
    engineer->motor[i].init_offset_f = 1;

    engineer->ctrl[i].convert_feedback = motor_pid_input_convert;
    pid_struct_init(&engineer->motor_pid[i], 1500000, 500000, 0.0f, 0, 0);
  }

  memcpy(&motor_name[0][name_len], "_MR\0", 4);
  memcpy(&motor_name[1][name_len], "_ML\0", 4);

  for (int i = 0; i < 2; i++)
  {
    err = motor_device_register(&(engineer->motor[i]), motor_name[i], 0);
    if (err != RM_OK) {
      // error handler
		}
  }

  memcpy(&motor_name[0][name_len], "_CTLMR\0", 7);
  memcpy(&motor_name[1][name_len], "_CTLML\0", 7);

  for (int i = 0; i < 2; i++)
  {
    err = pid_controller_register(&(engineer->ctrl[i]), motor_name[i],
																  &(engineer->motor_pid[i]),
																	&(engineer->motor_feedback[i]), 1);
    if (err != RM_OK) {
      // error handler
		}
  }

  return RM_OK;
}

int32_t moonrover_execute(Engineer* engineer, chassis_t pchassis, rc_device_t prc_dev, rc_info_t prc_info) {
  if (engineer == NULL)
    return -RM_INVAL;
 
	float lmotor_out, rmotor_out;
	struct motor_data *pdata_lm, *pdata_rm;
	
	float motor_speed;
	
	if (engineer->ENGINEER_BIG_STATE == LOWERPART && engineer->ENGINEER_SMALL_STATE == CHASSIS &&
			rc_device_get_state(prc_dev, RC_WHEEL_DOWN) == RM_OK) {
		motor_speed = ROTATION_SPEED;
	}
	else if (engineer->ENGINEER_BIG_STATE == LOWERPART && engineer->ENGINEER_SMALL_STATE == CHASSIS &&
			rc_device_get_state(prc_dev, RC_WHEEL_UP) == RM_OK) {
		motor_speed = -ROTATION_SPEED;
	}
	else {
		motor_device_set_current(&engineer->motor[LEFT_MOONROVER_INDEX], (int16_t)0);
		motor_device_set_current(&engineer->motor[RIGHT_MOONROVER_INDEX], (int16_t)0);
		return RM_OK;
	}
	
	pdata_lm = motor_device_get_data(&(engineer->motor[LEFT_MOONROVER_INDEX]));
	pdata_rm = motor_device_get_data(&(engineer->motor[RIGHT_MOONROVER_INDEX]));
				
	controller_set_input(&engineer->ctrl[LEFT_MOONROVER_INDEX], motor_speed);
	controller_set_input(&engineer->ctrl[RIGHT_MOONROVER_INDEX], -motor_speed);
				
	controller_execute(&engineer->ctrl[LEFT_MOONROVER_INDEX], (void *)pdata_lm);
	controller_execute(&engineer->ctrl[RIGHT_MOONROVER_INDEX], (void *)pdata_rm);
				
	controller_get_output(&engineer->ctrl[LEFT_MOONROVER_INDEX], &lmotor_out);
	controller_get_output(&engineer->ctrl[RIGHT_MOONROVER_INDEX], &rmotor_out);	
			
	motor_device_set_current(&engineer->motor[LEFT_MOONROVER_INDEX], (int16_t)16384);
	motor_device_set_current(&engineer->motor[RIGHT_MOONROVER_INDEX], (int16_t)-16384);
	
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

  for (int i = 0; i < 2; i++)
  {
    controller_enable(&(engineer->ctrl[i])); 
  }

  return RM_OK;
}

int32_t moonrover_disable(Engineer* engineer)
{
  if (engineer == NULL)
    return -RM_INVAL;

  for (int i = 0; i < 2; i++)
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

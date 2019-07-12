#include "dualmotor.h"
#include "engineer.h"
#include "pid.h"
#include "math.h"

/* VARIABLES: DUALMOTOR - related */
extern Engineer engg;
/* END of VARIABLES: DUALMOTOR - related */

/* FUNCTIONS: DUALMOTOR - related */
static float dualmotor_get_ecd_angle(int16_t raw_ecd);

int32_t dualmotor_cascade_register(Engineer *engineer, const char *name, enum device_can can) {
	char motor_name[2][OBJECT_NAME_MAX_LEN] = {0};
  uint8_t name_len;
  int32_t err;
	name_len = strlen(name);
	if (name_len > OBJECT_NAME_MAX_LEN / 2)
  {
    name_len = OBJECT_NAME_MAX_LEN / 2;
  }
	for (int i = DUALMOTOR_OFFSET + 0; i < DUALMOTOR_OFFSET + 2; i++)
  {
    memcpy(&motor_name[i], name, name_len);
    engineer->motor[i].can_periph = can;	
    engineer->motor[i].can_id = 0x205 + i;
  }
	memcpy(&motor_name[LEFT_DUALMOTOR_INDEX][name_len], "_LEFT\0", 6);
  memcpy(&motor_name[RIGHT_DUALMOTOR_INDEX][name_len], "_RIGHT\0", 7);
	for (int i = DUALMOTOR_OFFSET + 0; i < DUALMOTOR_OFFSET + 2; i++)
  {
    err = motor_device_register(&(engineer->motor[i]), motor_name[i], 0);
    if (err != RM_OK) {
			// error handle
		}
  }
	pid_struct_init(&(engineer->cascade[LEFT_DUALMOTOR_INDEX].outer), 2000, 0, 50, 0, 0);
  pid_struct_init(&(engineer->cascade[LEFT_DUALMOTOR_INDEX].inter), 30000, 3000, 70, 0.2, 0);
  pid_struct_init(&(engineer->cascade[RIGHT_DUALMOTOR_INDEX].outer), 2000, 0, 50, 0, 0);
  pid_struct_init(&(engineer->cascade[RIGHT_DUALMOTOR_INDEX].inter), 30000, 3000, 70, 0.2, 0);
	for (int i = DUALMOTOR_OFFSET + 0; i < DUALMOTOR_OFFSET + 2; i++)
  {
    err = cascade_controller_register(&(engineer->ctrl[i]), motor_name[i],
                                      &(engineer->cascade[i - DUALMOTOR_OFFSET]),
                                      &(engineer->cascade_fdb[i - DUALMOTOR_OFFSET]), 1);
    if (err != RM_OK) {
			// error handle
		}
  }
	return RM_OK;
}

int32_t dualmotor_execute(Engineer* engineer) {
	if (engineer == NULL)
		return -RM_INVAL;
	
	struct controller *ctrl_lm, *ctrl_rm;
	float ecd_target_angle;
	
	if (engineer->ENGINEER_STATE == CLAW_ROTATE) {
		ecd_target_angle = engineer->dualMotor.rise_angle;
	}
	else {
		ecd_target_angle = engineer->dualMotor.rest_angle;
	}
	
	ctrl_lm = &(engineer->ctrl[LEFT_DUALMOTOR_INDEX]);
	ctrl_rm = &(engineer->ctrl[RIGHT_DUALMOTOR_INDEX]);
	
	controller_set_input(ctrl_lm, ecd_target_angle);
	controller_set_input(ctrl_rm, ecd_target_angle);
	
	float motor_out_lm, motor_out_rm;
	struct motor_data *pdata_lm, *pdata_rm;
	
	pdata_lm = motor_device_get_data(&(engineer->motor[LEFT_DUALMOTOR_INDEX]));
	pdata_rm = motor_device_get_data(&(engineer->motor[RIGHT_DUALMOTOR_INDEX]));
	
	engineer->dualMotor.left_ecd_angle = dualmotor_get_ecd_angle(pdata_lm->ecd);
	engineer->dualMotor.right_ecd_angle = dualmotor_get_ecd_angle(pdata_rm->ecd);
	
	controller_execute(&(engineer->ctrl[LEFT_DUALMOTOR_INDEX]), (void *)engineer);
	controller_execute(&(engineer->ctrl[RIGHT_DUALMOTOR_INDEX]), (void *)engineer);
	
  controller_get_output(&(engineer->ctrl[LEFT_DUALMOTOR_INDEX]), &motor_out_lm);
	controller_get_output(&(engineer->ctrl[RIGHT_DUALMOTOR_INDEX]), &motor_out_rm);
	
  motor_device_set_current(&(engineer->motor[LEFT_DUALMOTOR_INDEX]), (int16_t)motor_out_lm);
	motor_device_set_current(&(engineer->motor[RIGHT_DUALMOTOR_INDEX]), (int16_t)motor_out_rm);
	
	return RM_OK;
}

static float dualmotor_get_ecd_angle(int16_t raw_ecd)
{
  return fabs(raw_ecd / 8191.0) * 360.0;
}

int32_t dualmotor_enable(Engineer* engineer)
{
  if (engineer == NULL)
    return -RM_INVAL;

  for (int i = DUALMOTOR_OFFSET + 0; i < DUALMOTOR_OFFSET + 2; i++)
  {
    controller_enable(&(engineer->ctrl[i])); 
  }

  return RM_OK;
}

int32_t dualmotor_disable(Engineer* engineer)
{
  if (engineer == NULL)
    return -RM_INVAL;

  for (int i = DUALMOTOR_OFFSET + 0; i < DUALMOTOR_OFFSET + 2; i++)
  {
    controller_disable(&(engineer->ctrl[i])); 
  }

  return RM_OK;
}
/* END of FUNCTIONS: DUALMOTOR - related */

/* RTOS: DUALMOTOR - related */
void dualmotor_task(void const *argument)
{
  uint32_t period = osKernelSysTick();
	for(;;) {
		dualmotor_execute(&engg);
    osDelayUntil(&period, 2);
	}
}
/* END of RTOS: DUALMOTOR - related */

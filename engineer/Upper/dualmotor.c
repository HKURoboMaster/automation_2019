#include "dualmotor.h"
#include "engineer.h"
#include "pid.h"
#include "math.h"
#include "dbus.h"
#include "upper.h"

/* VARIABLES: DUALMOTOR - related */
extern upper_ctrl upper_controller;
/* END of VARIABLES: DUALMOTOR - related */

/* FUNCTIONS: DUALMOTOR - related */
static float dualmotor_get_ecd_angle(int16_t raw_ecd);
static int32_t dualmotor_left_input_convert(struct controller *ctrl, void *input);
static int32_t dualmotor_right_input_convert(struct controller *ctrl, void *input);

int32_t dualmotor_cascade_register(upper_ctrl *upper, const char *name, enum device_can can) {
	char motor_name[2][OBJECT_NAME_MAX_LEN] = {0};
  uint8_t name_len;
  int32_t err;
	
	object_init(&(upper->parent), Object_Class_DualMotor, name);
	
	name_len = strlen(name);
	if (name_len > OBJECT_NAME_MAX_LEN / 2)
  {
    name_len = OBJECT_NAME_MAX_LEN / 2;
  }
	for (int i = 0; i < 2; i++)
  {
    memcpy(&motor_name[i], name, name_len);
    upper->motor[i].can_periph = can;	
    upper->motor[i].can_id = 0x201 + DUALMOTOR_OFFSET + i;
  }
	memcpy(&motor_name[LEFT_DUALMOTOR_INDEX][name_len], "_LFT\0", 5);
  memcpy(&motor_name[RIGHT_DUALMOTOR_INDEX][name_len], "_RHT\0", 5);
	
	for (int i = 0; i < 2; i++)
  {
    err = motor_device_register(&(upper->motor[i]), motor_name[i], 0);
    if (err != RM_OK) {
			goto end;
		}
  }
	
	memcpy(&motor_name[LEFT_DUALMOTOR_INDEX][name_len], "_CTL_L\0", 7);
  memcpy(&motor_name[RIGHT_DUALMOTOR_INDEX][name_len], "_CTL_R\0", 7);
	
	upper->ctrl[LEFT_DUALMOTOR_INDEX].convert_feedback = dualmotor_left_input_convert;
	pid_struct_init(&(upper->cascade[LEFT_DUALMOTOR_INDEX].outer), 360, 0, 1.0, 0, 0);
  pid_struct_init(&(upper->cascade[LEFT_DUALMOTOR_INDEX].inter), 30000, 0, 100.0, 0, 0.0001);
	
	upper->ctrl[RIGHT_DUALMOTOR_INDEX].convert_feedback = dualmotor_right_input_convert;
  pid_struct_init(&(upper->cascade[RIGHT_DUALMOTOR_INDEX].outer), 360, 0, 1.0, 0, 0);
  pid_struct_init(&(upper->cascade[RIGHT_DUALMOTOR_INDEX].inter), 30000, 0, 100.0, 0, 0.0001);
	for (int i = 0; i < 2; i++)
  {
    err = cascade_controller_register(&(upper->ctrl[i]), motor_name[i],
                                      &(upper->cascade[i]),
                                      &(upper->cascade_fdb[i]), 1);
    if (err != RM_OK) {
			goto end;
		}
  }
	return RM_OK;
end:
  object_detach(&(upper->parent));

  return err;
}

int32_t dualmotor_execute(upper_ctrl* upper) {
	if (upper == NULL)
		return -RM_INVAL;
	
	struct controller *ctrl_lm, *ctrl_rm;
	float ecd_target_angle;
	float motor_out_lm, motor_out_rm;
	struct motor_data *pdata_lm, *pdata_rm;

	pdata_lm = motor_device_get_data(&(upper->motor[LEFT_DUALMOTOR_INDEX]));
	pdata_rm = motor_device_get_data(&(upper->motor[RIGHT_DUALMOTOR_INDEX]));
	
	upper->dualMotor.left_ecd_angle = dualmotor_get_ecd_angle(pdata_lm->total_angle);
	upper->dualMotor.right_ecd_angle = dualmotor_get_ecd_angle(pdata_rm->total_angle);
	
	if (upper->dualMotor.DUALMOTOR_STATE == CLAW_RISE) {
		ecd_target_angle = upper->dualMotor.current_target_angle;
		if (upper->dualMotor.current_target_angle < upper->dualMotor.rise_angle &&
				upper->dualMotor.left_ecd_angle < upper->dualMotor.rise_angle &&
				upper->dualMotor.right_ecd_angle < upper->dualMotor.rise_angle)
				upper->dualMotor.current_target_angle += 1;
		ecd_target_angle = upper->dualMotor.rise_angle;
	}
	else if (upper->dualMotor.DUALMOTOR_STATE == CLAW_FALL) {
		ecd_target_angle = upper->dualMotor.current_target_angle;
		float eps = 0.1;
		if (upper->dualMotor.current_target_angle > 0 + eps)
				upper->dualMotor.current_target_angle -= 0.3;
	}
	else {
		ecd_target_angle = 0;
	}
	
	ctrl_lm = &(upper->ctrl[LEFT_DUALMOTOR_INDEX]);
	ctrl_rm = &(upper->ctrl[RIGHT_DUALMOTOR_INDEX]);
	
	controller_set_input(ctrl_lm, ecd_target_angle);
	controller_set_input(ctrl_rm, ecd_target_angle);
	
	upper->dualMotor.left_ecd_velocity = dualmotor_get_ecd_angle(pdata_lm->speed_rpm * (3.14159265359/30.0) / 19.0);
	upper->dualMotor.right_ecd_velocity = dualmotor_get_ecd_angle(pdata_rm->speed_rpm * (3.14159265359/30.0) / 19.0);
	
	controller_execute(&(upper->ctrl[LEFT_DUALMOTOR_INDEX]), (void *)upper);
	controller_execute(&(upper->ctrl[RIGHT_DUALMOTOR_INDEX]), (void *)upper);
	
  controller_get_output(&(upper->ctrl[LEFT_DUALMOTOR_INDEX]), &motor_out_lm);
	controller_get_output(&(upper->ctrl[RIGHT_DUALMOTOR_INDEX]), &motor_out_rm);
	
  motor_device_set_current(&(upper->motor[LEFT_DUALMOTOR_INDEX]), (int16_t)motor_out_lm);
	motor_device_set_current(&(upper->motor[RIGHT_DUALMOTOR_INDEX]), (int16_t)-motor_out_rm);
	
	return RM_OK;
}

static float dualmotor_get_ecd_angle(int16_t raw_ecd)
{
  return fabs(raw_ecd / 8191.0) * 360.0;
}

static int32_t dualmotor_left_input_convert(struct controller *ctrl, void *input)
{
	cascade_feedback_t cascade_fdb = (cascade_feedback_t)(ctrl->feedback);
  upper_ctrl* data = (upper_ctrl*)input;
  cascade_fdb->outer_fdb = data->dualMotor.left_ecd_angle;
  cascade_fdb->inter_fdb = data->dualMotor.left_ecd_velocity;
  return RM_OK;
}

static int32_t dualmotor_right_input_convert(struct controller *ctrl, void *input)
{
	cascade_feedback_t cascade_fdb = (cascade_feedback_t)(ctrl->feedback);
  upper_ctrl* data = (upper_ctrl*)input;
  cascade_fdb->outer_fdb = data->dualMotor.right_ecd_angle;
  cascade_fdb->inter_fdb = data->dualMotor.right_ecd_velocity;
  return RM_OK;
}

int32_t dualmotor_enable(upper_ctrl* upper)
{
  if (upper == NULL)
    return -RM_INVAL;

  for (int i = 0; i < 2; i++)
  {
    controller_enable(&(upper->ctrl[i])); 
  }

  return RM_OK;
}

int32_t dualmotor_disable(upper_ctrl* upper)
{
  if (upper == NULL)
    return -RM_INVAL;

  for (int i = 0; i < 2; i++)
  {
    controller_disable(&(upper->ctrl[i])); 
  }

  return RM_OK;
}
/* END of FUNCTIONS: DUALMOTOR - related */

/* RTOS: DUALMOTOR - related */
void dualmotor_task(void const *argument)
{
  uint32_t period = osKernelSysTick();
	for(;;) {
		dualmotor_execute(&upper_controller);
    osDelayUntil(&period, 2);
	}
}
/* END of RTOS: DUALMOTOR - related */

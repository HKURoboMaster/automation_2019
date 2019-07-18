#include "dualmotor.h"
#include "engineer.h"
#include "pid.h"
#include "math.h"
#include "dbus.h"
#include "arduino.h"

#define MINI_TOLERANCE 2.0f

/* VARIABLES: DUALMOTOR - related */
extern Engineer engg;
extern Ard_Tx tran_data;
/* END of VARIABLES: DUALMOTOR - related */

/* FUNCTIONS: DUALMOTOR - related */
static float dualmotor_get_ecd_angle(int16_t raw_ecd);
static int32_t dualmotor_left_input_convert(struct controller *ctrl, void *input);
static int32_t dualmotor_right_input_convert(struct controller *ctrl, void *input);

int32_t dualmotor_cascade_register(Engineer *engineer, const char *name, enum device_can can) {
	char motor_name[2][OBJECT_NAME_MAX_LEN] = {0};
  uint8_t name_len;
  int32_t err;
	
	object_init(&(engineer->parent), Object_Class_DualMotor, name);
	
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
	memcpy(&motor_name[LEFT_DUALMOTOR_INDEX][name_len], "_LFT\0", 5);
  memcpy(&motor_name[RIGHT_DUALMOTOR_INDEX][name_len], "_RHT\0", 5);
	
	for (int i = DUALMOTOR_OFFSET + 0; i < DUALMOTOR_OFFSET + 2; i++)
  {
    err = motor_device_register(&(engineer->motor[i]), motor_name[i], 0);
    if (err != RM_OK) {
			goto end;
		}
  }
	
	memcpy(&motor_name[LEFT_DUALMOTOR_INDEX][name_len], "_CTL_L\0", 7);
  memcpy(&motor_name[RIGHT_DUALMOTOR_INDEX][name_len], "_CTL_R\0", 7);
	
	engineer->ctrl[LEFT_DUALMOTOR_INDEX].convert_feedback = dualmotor_left_input_convert;
	pid_struct_init(&(engineer->cascade[LEFT_DUALMOTOR_INDEX].outer), 360, 0, 1.0, 0, 0);
  pid_struct_init(&(engineer->cascade[LEFT_DUALMOTOR_INDEX].inter), 30000, 0, 100.0, 0, 0.0001);
	
	engineer->ctrl[RIGHT_DUALMOTOR_INDEX].convert_feedback = dualmotor_right_input_convert;
  pid_struct_init(&(engineer->cascade[RIGHT_DUALMOTOR_INDEX].outer), 360, 0, 1.0, 0, 0);
  pid_struct_init(&(engineer->cascade[RIGHT_DUALMOTOR_INDEX].inter), 30000, 0, 100.0, 0, 0.0001);
	for (int i = DUALMOTOR_OFFSET + 0; i < DUALMOTOR_OFFSET + 2; i++)
  {
    err = cascade_controller_register(&(engineer->ctrl[i]), motor_name[i],
                                      &(engineer->cascade[i - DUALMOTOR_OFFSET]),
                                      &(engineer->cascade_fdb[i - DUALMOTOR_OFFSET]), 1);
    if (err != RM_OK) {
			goto end;
		}
  }
	return RM_OK;
end:
  object_detach(&(engineer->parent));

  return err;
}

int32_t dualmotor_execute(Engineer* engineer) {
	if (engineer == NULL)
		return -RM_INVAL;
	
	struct controller *ctrl_lm, *ctrl_rm;
	float ecd_target_angle;
	float motor_out_lm, motor_out_rm;
	struct motor_data *pdata_lm, *pdata_rm;

	pdata_lm = motor_device_get_data(&(engineer->motor[LEFT_DUALMOTOR_INDEX]));
	pdata_rm = motor_device_get_data(&(engineer->motor[RIGHT_DUALMOTOR_INDEX]));
	
	engineer->dualMotor.left_ecd_angle = dualmotor_get_ecd_angle(pdata_lm->total_angle);
	engineer->dualMotor.right_ecd_angle = dualmotor_get_ecd_angle(pdata_rm->total_angle);
	
	if (engineer->dualMotor.DUALMOTOR_STATE == CLAW_RISE) {
		ecd_target_angle = engineer->dualMotor.current_target_angle;
		if (engineer->dualMotor.current_target_angle < engineer->dualMotor.rise_angle &&
				engineer->dualMotor.left_ecd_angle < engineer->dualMotor.rise_angle &&
				engineer->dualMotor.right_ecd_angle < engineer->dualMotor.rise_angle)
				engineer->dualMotor.current_target_angle += 1;
		ecd_target_angle = engineer->dualMotor.rise_angle;
	}
	else if (engineer->dualMotor.DUALMOTOR_STATE == CLAW_FALL) {
		ecd_target_angle = engineer->dualMotor.current_target_angle;
		float eps = 0.1;
		if (engineer->dualMotor.current_target_angle > 0 + eps)
				engineer->dualMotor.current_target_angle -= 0.3f;
	}
	else {
		ecd_target_angle = 0;
	}
	
	ctrl_lm = &(engineer->ctrl[LEFT_DUALMOTOR_INDEX]);
	ctrl_rm = &(engineer->ctrl[RIGHT_DUALMOTOR_INDEX]);
	
	controller_set_input(ctrl_lm, ecd_target_angle);
	controller_set_input(ctrl_rm, ecd_target_angle);
	
	engineer->dualMotor.left_ecd_velocity = dualmotor_get_ecd_angle(pdata_lm->speed_rpm * (3.14159265359/30.0) / 19.0);
	engineer->dualMotor.right_ecd_velocity = dualmotor_get_ecd_angle(pdata_rm->speed_rpm * (3.14159265359/30.0) / 19.0);
	
	controller_execute(&(engineer->ctrl[LEFT_DUALMOTOR_INDEX]), (void *)engineer);
	controller_execute(&(engineer->ctrl[RIGHT_DUALMOTOR_INDEX]), (void *)engineer);
	
  controller_get_output(&(engineer->ctrl[LEFT_DUALMOTOR_INDEX]), &motor_out_lm);
	controller_get_output(&(engineer->ctrl[RIGHT_DUALMOTOR_INDEX]), &motor_out_rm);
	
  motor_device_set_current(&(engineer->motor[LEFT_DUALMOTOR_INDEX]), (int16_t)motor_out_lm);
	motor_device_set_current(&(engineer->motor[RIGHT_DUALMOTOR_INDEX]), (int16_t)-motor_out_rm);
	//Implement an error range checking mechanism
	//If the error smaller than a certain number Report the task is Done
	if(fabs(engineer->cascade_fdb->outer_fdb - ecd_target_angle) < MINI_TOLERANCE)
	{
		if(engineer->dualMotor.DUALMOTOR_STATE == CLAW_FALL)
		{
			tran_data.rotation_flag[0] = ROTATION_DONE;
			tran_data.rotation_flag[1] = ROTATION_BUSY;
		}
		else if(engineer->dualMotor.DUALMOTOR_STATE == CLAW_RISE)
		{	
			tran_data.rotation_flag[1] = ROTATION_DONE;
			tran_data.rotation_flag[0] = ROTATION_BUSY;
		}
	}
	else
	{
		tran_data.rotation_flag[0] = ROTATION_BUSY;
		tran_data.rotation_flag[1] = ROTATION_BUSY;
	}
		
		
	return RM_OK;
}

static float dualmotor_get_ecd_angle(int16_t raw_ecd)
{
  return fabs(raw_ecd / 8191.0) * 360.0;
}

static int32_t dualmotor_left_input_convert(struct controller *ctrl, void *input)
{
	cascade_feedback_t cascade_fdb = (cascade_feedback_t)(ctrl->feedback);
  Engineer* data = (Engineer*)input;
  cascade_fdb->outer_fdb = data->dualMotor.left_ecd_angle;
  cascade_fdb->inter_fdb = data->dualMotor.left_ecd_velocity;
  return RM_OK;
}

static int32_t dualmotor_right_input_convert(struct controller *ctrl, void *input)
{
	cascade_feedback_t cascade_fdb = (cascade_feedback_t)(ctrl->feedback);
  Engineer* data = (Engineer*)input;
  cascade_fdb->outer_fdb = data->dualMotor.right_ecd_angle;
  cascade_fdb->inter_fdb = data->dualMotor.right_ecd_velocity;
  return RM_OK;
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

#include "raiser.h"
#include "engineer.h"
#include "pid.h"
#include "math.h"
#include "dbus.h"
#include "upper.h"

/* VARIABLES: RAISER - related */
extern Engineer engg;
/* END of VARIABLES: RAISER - related */

/* FUNCTIONS: RAISER - related */
static float raiser_get_ecd_angle(int16_t raw_ecd);
static int32_t raiser_left_input_convert(struct controller *ctrl, void *input);
static int32_t raiser_right_input_convert(struct controller *ctrl, void *input);

int32_t raiser_cascade_register(Engineer* engineer, const char *name, enum device_can can) {
	char motor_name[2][OBJECT_NAME_MAX_LEN] = {0};
  uint8_t name_len;
  int32_t err;
	
	object_init(&(engineer->parent), Object_Class_Raiser, name);
	
	name_len = strlen(name);
	if (name_len > OBJECT_NAME_MAX_LEN / 2)
  {
    name_len = OBJECT_NAME_MAX_LEN / 2;
  }
	for (int i = 0; i < 2; i++)
  {
    memcpy(&motor_name[i], name, name_len);
    engineer->raiser_motor[i].can_periph = can;	
    engineer->raiser_motor[i].can_id = 0x205 + RAISER_OFFSET + i;
  }
	memcpy(&motor_name[FRONT_RAISER_INDEX][name_len], "_FRT\0", 5);
  memcpy(&motor_name[REAR_RAISER_INDEX][name_len], "_RER\0", 5);
	
	for (int i = 0; i < 2; i++)
  {
    err = motor_device_register(&(engineer->raiser_motor[i]), motor_name[i], 0);
    if (err != RM_OK) {
			goto end;
		}
  }
	
	memcpy(&motor_name[FRONT_RAISER_INDEX][name_len], "_CTL_F\0", 7);
  memcpy(&motor_name[REAR_RAISER_INDEX][name_len], "_CTL_R\0", 7);
	
	engineer->raiser_ctrl[FRONT_RAISER_INDEX].convert_feedback = raiser_left_input_convert;
	pid_struct_init(&(engineer->raiser_cascade[FRONT_RAISER_INDEX].outer), 360, 0, 1.0, 0, 0);
  pid_struct_init(&(engineer->raiser_cascade[FRONT_RAISER_INDEX].inter), 30000, 0, 100.0, 0, 0.0001);
	
	engineer->raiser_ctrl[REAR_RAISER_INDEX].convert_feedback = raiser_right_input_convert;
  pid_struct_init(&(engineer->raiser_cascade[REAR_RAISER_INDEX].outer), 360, 0, 1.0, 0, 0);
  pid_struct_init(&(engineer->raiser_cascade[REAR_RAISER_INDEX].inter), 30000, 0, 100.0, 0, 0.0001);
	for (int i = 0; i < 2; i++)
  {
    err = cascade_controller_register(&(engineer->raiser_ctrl[i]), motor_name[i],
                                      &(engineer->raiser_cascade[i]),
                                      &(engineer->raiser_cascade_fdb[i]), 1);
    if (err != RM_OK) {
			goto end;
		}
  }
	return RM_OK;
end:
  object_detach(&(engineer->parent));

  return err;
}

int32_t raiser_execute(Engineer* engineer) {
	if (engineer == NULL)
		return -RM_INVAL;
	
	struct controller *ctrl_fm, *ctrl_rm;
	float ecd_target_angle;
	float motor_out_fm, motor_out_rm;
	struct motor_data *pdata_fm, *pdata_rm;

	pdata_fm = motor_device_get_data(&(engineer->raiser_motor[FRONT_RAISER_INDEX]));
	pdata_rm = motor_device_get_data(&(engineer->raiser_motor[REAR_RAISER_INDEX]));
	
	engineer->raiser.left_ecd_angle = raiser_get_ecd_angle(pdata_fm->total_angle);
	engineer->raiser.right_ecd_angle = raiser_get_ecd_angle(pdata_rm->total_angle);
	
	if (engineer->raiser.RAISER_STATE == RAISE) {
		ecd_target_angle = engineer->raiser.current_target_angle;
		if (engineer->raiser.current_target_angle < engineer->raiser.rise_angle &&
				engineer->raiser.left_ecd_angle < engineer->raiser.rise_angle &&
				engineer->raiser.right_ecd_angle < engineer->raiser.rise_angle)
				engineer->raiser.current_target_angle += 1;
		ecd_target_angle = engineer->raiser.rise_angle;
	}
	else if (engineer->raiser.RAISER_STATE == LOWER) {
		ecd_target_angle = engineer->raiser.current_target_angle;
		float eps = 0.1;
		if (engineer->raiser.current_target_angle > 0 + eps)
				engineer->raiser.current_target_angle -= 0.3;
	}
	else {
		ecd_target_angle = 0;
	}
	
	ctrl_fm = &(engineer->raiser_ctrl[FRONT_RAISER_INDEX]);
	ctrl_rm = &(engineer->raiser_ctrl[REAR_RAISER_INDEX]);
	
	controller_set_input(ctrl_fm, ecd_target_angle);
	controller_set_input(ctrl_rm, ecd_target_angle);
	
	engineer->raiser.left_ecd_velocity = raiser_get_ecd_angle(pdata_fm->speed_rpm * (3.14159265359/30.0) / 19.0);
	engineer->raiser.right_ecd_velocity = raiser_get_ecd_angle(pdata_rm->speed_rpm * (3.14159265359/30.0) / 19.0);
	
	controller_execute(&(engineer->raiser_ctrl[FRONT_RAISER_INDEX]), (void *)engineer);
	controller_execute(&(engineer->raiser_ctrl[REAR_RAISER_INDEX]), (void *)engineer);
	
  controller_get_output(&(engineer->raiser_ctrl[FRONT_RAISER_INDEX]), &motor_out_fm);
	controller_get_output(&(engineer->raiser_ctrl[REAR_RAISER_INDEX]), &motor_out_rm);
	
  motor_device_set_current(&(engineer->raiser_motor[FRONT_RAISER_INDEX]), (int16_t)motor_out_fm);
	motor_device_set_current(&(engineer->raiser_motor[REAR_RAISER_INDEX]), (int16_t)-motor_out_rm);
	
	return RM_OK;
}

static float raiser_get_ecd_angle(int16_t raw_ecd)
{
  return fabs(raw_ecd / 8191.0) * 360.0;
}

static int32_t raiser_left_input_convert(struct controller *ctrl, void *input)
{
	cascade_feedback_t cascade_fdb = (cascade_feedback_t)(ctrl->feedback);
  Engineer* data = (Engineer*)input;
  cascade_fdb->outer_fdb = data->raiser.left_ecd_angle;
  cascade_fdb->inter_fdb = data->raiser.left_ecd_velocity;
  return RM_OK;
}

static int32_t raiser_right_input_convert(struct controller *ctrl, void *input)
{
	cascade_feedback_t cascade_fdb = (cascade_feedback_t)(ctrl->feedback);
  Engineer* data = (Engineer*)input;
  cascade_fdb->outer_fdb = data->raiser.right_ecd_angle;
  cascade_fdb->inter_fdb = data->raiser.right_ecd_velocity;
  return RM_OK;
}

int32_t raiser_enable(Engineer* engineer)
{
  if (engineer == NULL)
    return -RM_INVAL;

  for (int i = 0; i < 2; i++)
  {
    controller_enable(&(engineer->ctrl[i])); 
  }

  return RM_OK;
}

int32_t raiser_disable(Engineer* engineer)
{
  if (engineer == NULL)
    return -RM_INVAL;

  for (int i = 0; i < 2; i++)
  {
    controller_disable(&(engineer->ctrl[i])); 
  }

  return RM_OK;
}
/* END of FUNCTIONS: RAISER - related */

/* RTOS: RAISER - related */
void raiser_task(void const *argument)
{
  uint32_t period = osKernelSysTick();
	for(;;) {
		raiser_execute(&engg);
    osDelayUntil(&period, 2);
	}
}
/* END of RTOS: RAISER - related */

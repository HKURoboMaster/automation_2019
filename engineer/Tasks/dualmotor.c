#include "dualmotor.h"
#include "motor.h"
#include "pid.h"
#include "math.h"
#include "engineer.h"

/* VARIABLES: DUALMOTOR - related */
/* END of VARIABLES: DUALMOTOR - related */

/* FUNCTIONS: DUALMOTOR - related */
int32_t DUALMOTOR_cascade_register(Engineer *engineer, const char *name, enum device_can can) {
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
                                      &(engineer->cascade[i]),
                                      &(engineer->cascade_fdb[i]), 1);
    if (err != RM_OK) {
			// error handle
		}
  }
	return RM_OK;
}
int32_t DUALMOTOR_execute(Engineer engineer) {
	float motor_out;
  struct motor_data *pdata;
}
/* END of FUNCTIONS: DUALMOTOR - related */

/* VARIABLES: RTOS - related */
osThreadId DUALMOTOR_Handle;
/* END of VARIABLES: RTOS - related */

/* FUNCTIONS: DUALMOTOR - related */
float rms_error_2vel(float TARGET_left_vel, float TARGET_right_vel) {
	float left_motor_vel = left.data.speed_rpm * (3.14159265359/30.0) / 19.0;
	float right_motor_vel = right.data.speed_rpm * (3.14159265359/30.0) / 19.0;
	return sqrt((TARGET_left_vel - left_motor_vel)*(TARGET_left_vel - left_motor_vel) + (TARGET_right_vel - right_motor_vel)*(TARGET_right_vel - right_motor_vel));
}
void DUALMOTOR_Task(void const *argument)
{
	uint32_t DUALMOTOR_wake_time = osKernelSysTick();
	float TARGET_left_pos, TARGET_right_pos, TARGET_left_vel, TARGET_right_vel;
	float eps = 0.1;
	for(;;) {	// POSITION
		left_motor.outer.current_value = left.data.total_angle / left.data.round_cnt;
		right_motor.outer.current_value = right.data.total_angle / right.data.round_cnt;
		TARGET_left_vel = CASCADE_OUTER_CALC(TARGET_left_pos, &left_motor);
		TARGET_right_vel = CASCADE_OUTER_CALC(TARGET_right_pos, &right_motor);
		while(rms_error_2vel(TARGET_left_vel, TARGET_right_vel) > eps) {	// SPEED
			left_motor.inner.current_value = left.data.speed_rpm * (3.14159265359/30.0) / 19.0;
			right_motor.inner.current_value = right.data.speed_rpm * (3.14159265359/30.0) / 19.0;
			TRANSMIT_Can(CASCADE_INNER_CALC(TARGET_left_vel, &left_motor), 0, 0, CASCADE_INNER_CALC(TARGET_right_vel, &right_motor));
			osDelayUntil(&DUALMOTOR_wake_time, 5);
		}
    osDelayUntil(&DUALMOTOR_wake_time, 5);
  }
}
/* END of FUNCTIONS: DUALMOTOR - related */

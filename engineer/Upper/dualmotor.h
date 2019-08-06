#ifndef DUAL_MOTOR_H
#define DUAL_MOTOR_H

#include "motor.h"

#define AT_TARGET_rise 0
#define NOT_AT_TARGET_rise 1
#define AT_TARGET_fall 3
#define NOT_AT_TARGET_fall 4

struct dualMotor {
	float rest_angle;
	float rise_angle;
	float grab_angle;
	float left_ecd_angle;
	float right_ecd_angle;
	float left_ecd_velocity;
	float right_ecd_velocity;
	int init;
	float dual_left_offset, dual_right_offset;
	float current_target_angle;
	
	int dualMotor_init;
	
	float DUALMOTOR_STATE;
	int DUALMOTOR_ACTUAL_STATE;
};

typedef struct upper_ctrl upper_ctrl;

int32_t dualmotor_cascade_register(upper_ctrl *upper, const char *name, enum device_can can);
int32_t dualmotor_disable(upper_ctrl* upper);
int32_t dualmotor_enable(upper_ctrl* upper);
void dualmotor_task(void const *argument);

void rotate_to_grab(upper_ctrl* upper);
void rotate_to_zero(upper_ctrl* upper);
void rotate_to_grabbing(upper_ctrl* upper);

#endif

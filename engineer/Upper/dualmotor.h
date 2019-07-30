#ifndef DUAL_MOTOR_H
#define DUAL_MOTOR_H

#include "motor.h"

struct dualMotor {
	float rest_angle;
	float rise_angle;
	float left_ecd_angle;
	float right_ecd_angle;
	float left_ecd_velocity;
	float right_ecd_velocity;
	int init;
	float current_target_angle;
	float DUALMOTOR_STATE;
};

typedef struct upper_ctrl upper_ctrl;

int32_t dualmotor_cascade_register(upper_ctrl *upper, const char *name, enum device_can can);
int32_t dualmotor_disable(upper_ctrl* upper);
int32_t dualmotor_enable(upper_ctrl* upper);
void dualmotor_task(void const *argument);

#endif

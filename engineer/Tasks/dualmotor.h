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

typedef struct Engineer Engineer;

int32_t dualmotor_cascade_register(Engineer *engineer, const char *name, enum device_can can);
int32_t dualmotor_disable(Engineer* engineer);
int32_t dualmotor_enable(Engineer* engineer);
void dualmotor_task(void const *argument);

#endif

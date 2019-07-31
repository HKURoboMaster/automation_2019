#ifndef RAISER_H
#define RAISER_H

#include "motor.h"

struct raiser {
	float rest_angle;
	float rise_angle;
	float left_ecd_angle;
	float right_ecd_angle;
	float left_ecd_velocity;
	float right_ecd_velocity;
	int init;
	float current_target_angle;
	float RAISER_STATE;
};

typedef struct Engineer Engineer;

int32_t raiser_cascade_register(Engineer* engineer, const char *name, enum device_can can);
int32_t raiser_enable(Engineer* engineer);
int32_t raiser_disable(Engineer* engineer);
void raiser_task(void const *argument);

#endif

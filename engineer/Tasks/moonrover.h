#ifndef MOON_ROVER_H
#define MOON_ROVER_H

#include "motor.h"
typedef struct Engineer Engineer;

int32_t moonrover_pid_register(Engineer* engineer, const char *name, enum device_can can);
void moonrover_task(void const *argument);
int32_t moonrover_enable(Engineer* engineer);
int32_t moonrover_disable(Engineer* engineer);

#endif

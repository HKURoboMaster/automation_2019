#ifndef UPPER_H
#define UPPER_H

#include "dualmotor.h"
#include "motor.h"
#include "pid_controller.h"

struct upper_info
{
  int mode;
};

typedef struct upper_ctrl
{
	struct object parent;
	struct motor_device motor[2];
  struct cascade cascade[2];
	struct dualMotor dualMotor;
  struct cascade_feedback cascade_fdb[2];
  struct controller ctrl[2];
	
} upper_ctrl;

void set_upper_info(int mode);
void upper_task(void const *argument);

#endif

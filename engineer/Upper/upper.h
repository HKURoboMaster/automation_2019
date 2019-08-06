#ifndef UPPER_H
#define UPPER_H

#include "dualmotor.h"
#include "motor.h"
#include "pid_controller.h"
#include "slider.h"

/* VARIABLES: DUALMOTOR - related */
#define LEFT_DUALMOTOR_INDEX 0
#define RIGHT_DUALMOTOR_INDEX 1
#define DUALMOTOR_OFFSET 0
#define REST_ANGLE 10
#define RISE_ANGLE 110
#define GRAB_ANGLE 150
/* END of VARIABLES: DUALMOTOR - related */

/* VARIABLES: UPPER - related */
#define NO_GRAB 0
#define START_GRAB 1

#define GRAB_READY 2
#define DUNKING 3
#define THROWING 4
/* END of VARIABLES: UPPER - related */

/* VARIABLES: REQUEST (except for slider) - related */
#define NULL_REQ 0
#define PNEU_REQ 1
/* END of VARIABLES: REQUEST (except for slider) - related */

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
		
  int START_GRABBING;
	int CURRENT_STATE;
	
} upper_ctrl;

void set_upper_info(int mode);
void upper_task(void const *argument);

#endif

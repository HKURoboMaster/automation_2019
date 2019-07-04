#ifndef ENGINEER_H
#define ENGINEER_H

#include "motor.h"
#include "pid_controller.h"

/* VARIABLES: DUALMOTOR - related */
#define LEFT_DUALMOTOR_INDEX 0
#define RIGHT_DUALMOTOR_INDEX 1
#define DUALMOTOR_OFFSET 0
/* END of VARIABLES: DUALMOTOR - related */

/* STATES: Engineer - related */
#define CHASSIS 0
#define CLIMBING 1
#define LOCATING 2
/* END of STATES: Engineer - related */

typedef struct Engineer {
	float roll, yaw, pitch;
	int ENGINEER_STATE;
	struct motor_device motor[10];
  struct cascade cascade[2];
  struct cascade_feedback cascade_fdb[2];
  struct controller ctrl[10];
} Engineer;

#endif

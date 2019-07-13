#ifndef ENGINEER_H
#define ENGINEER_H

#include "motor.h"
#include "pid_controller.h"
#include "dualmotor.h"
#include "mecanum.h"

/* VARIABLES: DUALMOTOR - related */
#define LEFT_DUALMOTOR_INDEX 0
#define RIGHT_DUALMOTOR_INDEX 1
#define DUALMOTOR_OFFSET 0
#define REST_ANGLE 0
#define RISE_ANGLE 120
/* END of VARIABLES: DUALMOTOR - related */

/* VARIABLES: MOONROVER - related */
#define MOONROVER_OFFSET 2 + DUALMOTOR_OFFSET
#define ROTATION_SPEED 5
/* END of VARIABLES: MOONROVER - related */

/* STATES: Engineer - related */
#define CHASSIS 0
#define CLIMBING 1
#define LOCATING 2
#define CLAW_ROTATE 3
/* END of STATES: Engineer - related */

typedef struct Engineer {
	struct object parent;
	float roll, yaw, pitch;
	int ENGINEER_STATE;
	int HALT_CHASSIS;
	struct motor_device motor[4];
	struct pid motor_pid[2];
	struct pid_feedback motor_feedback[2];
	struct mecanum mecanum;
  struct cascade cascade[2];
	struct dualMotor dualMotor;
  struct cascade_feedback cascade_fdb[2];
  struct controller ctrl[4];
} Engineer;

void engineer_task(void const *argument);

#endif

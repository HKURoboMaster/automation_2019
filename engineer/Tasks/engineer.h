#ifndef ENGINEER_H
#define ENGINEER_H

#include "motor.h"
#include "pid_controller.h"
#include "raiser.h"
#include "mecanum.h"
#include "locomotion.h"

/* VARIABLES: MOONROVER - related */
#define MOONROVER_OFFSET 0
#define LEFT_MOONROVER_INDEX 0
#define RIGHT_MOONROVER_INDEX 1
#define ROTATION_SPEED 	10000
/* END of VARIABLES: MOONROVER - related */

/* VARIABLES: RAISER - related */
#define FRONT_RAISER_INDEX 0
#define REAR_RAISER_INDEX 1
#define RAISER_OFFSET 2
#define LOWER 0
#define RAISE 1
/* END of VARIABLES: RAISER - related */

/* VARIABLES: CLIMBING - related */
#define PITCH_2_ASSIST 1
/* VARIABLES: CLIMBING - related */

/* BIG STATES: Engineer - related */
#define UPPERPART 0
#define LOWERPART 1
#define MANAGE 2
/* END of BIG STATES: Engineer - related */

/* SMALL STATES: Engineer - related */
#define CHASSIS 0
#define REVERSE_CHASSIS 1
#define UNLOAD 2
#define SINGLE_LOCATE 3
#define TRIO_LOCATE 4
#define PENTA_LOCATE 5
#define RESET 6
#define CHASSIS_ONLY 7
#define OFF 8
/* END of SMALL STATES: Engineer - related */

/* GRAB STATES: GRAB - related */
#define IDLE 0
#define LOCATING 1
#define LOCATED 2
/* END of GRAB STATES: GRAB - related */

/* DUALMOTOR STATES: DUALMOTOR - related */
#define CLAW_FALL 0
#define CLAW_RISE 1
#define CLAW_GRAB 2
/* END of DUALMOTOR STATES: DUALMOTOR - related */

/* RELOADER STATES: RELOADER - related */
#define CLOSE 0
#define OPEN 1
/* END of RELOADER STATES: RELOADER - related */

typedef struct Engineer {
	struct object parent;
	float roll, yaw, pitch;
	
	int ENGINEER_BIG_STATE;
	int ENGINEER_SMALL_STATE;
	int HALT_CHASSIS;
	
	struct motor_device motor[2];
	struct pid motor_pid[2];
	struct pid_feedback motor_feedback[2];
	struct controller ctrl[2];
	
	struct motor_device raiser_motor[2];
  struct cascade raiser_cascade[2];
  struct cascade_feedback raiser_cascade_fdb[2];
  struct controller raiser_ctrl[2];
	
	struct raiser raiser;
	int reloader;
} Engineer;

void engineer_task(void const *argument);

#endif

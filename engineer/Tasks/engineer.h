#ifndef ENGINEER_H
#define ENGINEER_H

#include "motor.h"
#include "pid_controller.h"
#include "dualmotor.h"
#include "mecanum.h"
#include "grab.h"
#include "locomotion.h"

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
// SINGLE_LOCATE MEANS MANUAL STATE
#define FIVE_LOCATE 3
#define SINGLE_LOCATE 4
#define THREE_LOCATE 5
// THREE_LOCATE means on the island Get 3 continous boxes.
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
#define CLAW_IDLE 2
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
	struct motor_device motor[4];
	
	struct pid motor_pid[2];
	struct pid_feedback motor_feedback[2];
	struct mecanum mecanum;
  struct cascade cascade[2];
	
	struct dualMotor dualMotor;
	struct Grabber grabber;
	
  struct cascade_feedback cascade_fdb[2];
	
  struct controller ctrl[4];
	
	int reloader;
	float PITCH_TILL_ASSIST;
} Engineer;

void engineer_task(void const *argument);
void update_engg_imu(float yaw, float pitch, float roll);

#endif

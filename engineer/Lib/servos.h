#ifndef SERVO_H
#define SERVO_H

#include "cmsis_os.h"

#define SLAVE 0x80

/* VARIABLES: SERVOS - related */
#define SERVO_NOTDONE 0
#define SERVO_DONE 1
/* END of VARIABLES: SERVOS - related */

typedef struct Servo {
	int servoNo;
	int minAngle;
	int maxAngle;
	int minPulse;
	int maxPulse;
	int current_index;
	int target_index;
	int step_size;
} Servo;

void PCA9685_init(void);
void PCA9685_transmit(uint8_t no, uint16_t on, uint16_t off);
int SERVO_raiseTo(Servo* servo, int to);
int SERVO_fallTo(Servo* servo, int to);
void SERVO_pwm(Servo* servo, int startAngle, int endAngle);

#endif

#ifndef SERVO_H
#define SERVO_H

#include "cmsis_os.h"

#define SLAVE 0x80

typedef struct Servo {
	int servoNo;
	int minAngle;
	int maxAngle;
	int minPulse;
	int maxPulse;
} Servo;

void PCA9685_init(void);
void SERVO_pwm(Servo* servo, int startAngle, int endAngle);

#endif

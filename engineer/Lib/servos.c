#include "servos.h"
#include "i2c.h"

/* VARIABLES: SERVO - related */
/* END of VARIABLES: SERVO - related */

/* FUNCTIONS: SERVO - related */
void PCA9685_init() {
	// START
	HAL_I2C_Master_Transmit(&hi2c2, SLAVE, 0x00, 1, 1);
	// INIT
	uint8_t init[2];
	init[0] = 0x00;
	init[1] =  ((0 & 0x7F) | 0x10);
	HAL_I2C_Master_Transmit(&hi2c2, SLAVE, init, 2, 1);
	init[1] = 3;
	HAL_I2C_Master_Transmit(&hi2c2, SLAVE, init, 2, 1);
	init[1] = 0;
	HAL_I2C_Master_Transmit(&hi2c2, SLAVE, init, 2, 1);
	osDelay(10);
	init[1] = (0 | 0xA1);
	HAL_I2C_Master_Transmit(&hi2c2, SLAVE, init, 2, 1);
}
void PCA9685_transmit(uint8_t no, uint16_t on, uint16_t off) {
	uint8_t msg[5] = {0x06 + 4*no, on, (on >> 8), off, (off >> 8)};
	HAL_I2C_Master_Transmit(&hi2c2, SLAVE, msg, 5, 1);
}
void SERVO_pwm(Servo* servo, int startAngle, int endAngle) {
	/*
		startAngle -> current servo angle. Note that this is relative, as it is the DELAY till servo rotates by (end_angle - current_angle)
		endAngle -> desired servo angle
	*/
	float scalePulse = (float)(servo->maxPulse - servo->minPulse) / (float)(servo->maxAngle - servo->minAngle);
	int onPulse = scalePulse * startAngle, offPulse = scalePulse * endAngle;
	PCA9685_transmit(servo->servoNo, onPulse, offPulse);
}
/* END of FUNCTIONS: SERVO - related */

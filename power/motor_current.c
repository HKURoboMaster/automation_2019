#include "motor_current.h"
#include "chassis.h"

void return_chassis_power(chassis_t pchassis, float* current0, float* current1,
												  float* current2, float* current3) {
	// power = current * voltage
	*current0 = pchassis->motor[0].current * 24.0;
	*current1 = pchassis->motor[1].current * 24.0;
	*current2 = pchassis->motor[2].current * 24.0;
	*current3 = pchassis->motor[3].current * 24.0;
}

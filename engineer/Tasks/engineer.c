#include "engineer.h"
#include "cmsis_os.h"
#include "param.h"
#include "ramp.h"
#include "can.h"
#include "board.h"
#include "dbus.h"
#include "dualmotor.h"
#include "engg_gpio.h"

/* VARIABLES: ENGINEER - related */
float PITCH_TILL_ASSIST  = 0.5;
/* VARIABLES: END of ENGINEER - related */

/* CONSTANTS: ENGINEER - related */
Engineer engg;
/* CONSTANTS: END of ENGINEER - related */

/* FUNCTIONS: ENGINEER - related */
void ENGINEER_update_imu(Engineer* engineer, float yaw, float pitch, float roll) {
	engineer->yaw = yaw;
	engineer->pitch = pitch;
	engineer->roll = roll;
}
int32_t imu_update(Engineer engineer) {
	struct ahrs_sensor mpu_sensor;
  struct attitude mahony_atti;
	mpu_get_data(&mpu_sensor);
  mahony_ahrs_updateIMU(&mpu_sensor, &mahony_atti);
	ENGINEER_update_imu(&engineer, mahony_atti.yaw, mahony_atti.pitch, mahony_atti.roll);
	return 0;
}
/* END of FUNCTIONS: ENGINEER - related */

/* RTOS: ENGINEER - related */
void engineer_task(void const *argument)
{
	uint32_t period = osKernelSysTick();
	engg.dualMotor.rest_angle = REST_ANGLE;
	engg.dualMotor.rise_angle = RISE_ANGLE;
	
	for(;;) {
		imu_update(engg);
		if (engg.ENGINEER_STATE == LOCATING) {
			if (read_sonicL() && read_sonicR()) {
					engg.HALT_CHASSIS = 1;
			}
			else {
				engg.HALT_CHASSIS = 0;
			}
		}
		osDelayUntil(&period, 2);
	}
}
/* END of RTOS: ENGINEER - related */

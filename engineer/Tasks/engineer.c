#include "engineer.h"
#include "cmsis_os.h"
#include "param.h"
#include "ramp.h"
#include "can.h"
#include "board.h"
#include "dbus.h"

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

/* VARIABLES: RTOS - related */
osThreadId ENGINEER_Handle;
/* END of VARIABLES: RTOS - related */

/* FUNCTIONS: RTOS - related */
void ENGINEER_Task(void const *argument)
{
	uint32_t ENGINEER_wake_time = osKernelSysTick();
	for(;;) {
		if (engg.ENGINEER_STATE == CHASSIS) {
			// use base 2019 code to control chassis
		}
		else if (engg.ENGINEER_STATE == CLIMBING) {
			imu_update(engg);
			if (engg.pitch > PITCH_TILL_ASSIST) {
				// spin third wheel, send CAN cmd etc
			}
		}
		else if (engg.ENGINEER_STATE == LOCATING) {
			// poll GPIO
		}
		osDelayUntil(&ENGINEER_wake_time, 5);
	}
}
void CREATE_ENGINEER_Task(void)
{
	osThreadDef(ENGINEERTask, ENGINEER_Task, osPriorityRealtime, 0, 256);
	ENGINEER_Handle = osThreadCreate(osThread(ENGINEERTask), NULL);
}
/* END of FUNCTIONS: RTOS - related */

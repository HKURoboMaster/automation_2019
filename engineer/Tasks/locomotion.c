#include "locomotion.h"
#include "engineer.h"
#include "dbus.h"
#include "servos.h"
#include "engg_gpio.h"

/* VARIABLES: LOCOMOTION - related */
extern Engineer engg;
Servo reload_0 = {
	0, 0, 180, 0, 4096
};
	
Servo reload_1 = {
	1, 0, 180, 0, 4096
};
/* END of VARIABLES: LOCOMOTION - related */

/* FUNCTIONS: LOCOMOTION - related */
int32_t locomotion_execute(Engineer* engineer, rc_device_t prc_dev, rc_info_t prc_info) {
	if (engineer == NULL)
		return -RM_INVAL;
	
	if (engineer->ENGINEER_BIG_STATE != LOWERPART) {
		// look forward and close servos
		use_front_cam();
		if (engineer->reloader == OPEN) {
			SERVO_pwm(&reload_0, 180, 0);
			SERVO_pwm(&reload_1, 180, 0);
			engineer->reloader = CLOSE;
		}
		return RM_OK;
	}
	else {
		if (engineer->HALT_CHASSIS != 0)
			engineer->HALT_CHASSIS = 0;
	}
	
	if (engineer->ENGINEER_SMALL_STATE != UNLOAD) {
		if (engineer->reloader == OPEN) {
			SERVO_pwm(&reload_0, 180, 0);
			SERVO_pwm(&reload_1, 180, 0);
			engineer->reloader = CLOSE;
		}
	}
	
	if (engineer->ENGINEER_SMALL_STATE == CHASSIS) {
		use_front_cam();
	}
	else if (engineer->ENGINEER_SMALL_STATE == REVERSE_CHASSIS) {
		use_rear_cam();
	}
	else if (engineer->ENGINEER_SMALL_STATE == UNLOAD) {
		if (engineer->reloader == CLOSE) {
			SERVO_pwm(&reload_0, 0, 180);
			SERVO_pwm(&reload_1, 0, 180);
			engineer->reloader = OPEN;
		}
	}

	return RM_OK;
}
/* END of FUNCTIONS: LOCOMOTION - related */

/* RTOS: LOCOMOTION - related */
void locomotion_task(void const *argument)
{
  uint32_t period = osKernelSysTick();
	
	rc_device_t prc_dev = NULL;
	rc_info_t prc_info = NULL;
	
	prc_dev = rc_device_find("uart_rc");
  if (prc_dev != NULL)
  {
    prc_info = rc_device_get_info(prc_dev);
  }
	
	for(;;) {
		locomotion_execute(&engg, prc_dev, prc_info);
    osDelayUntil(&period, 2);
	}
}
/* END of RTOS: LOCOMOTION - related */

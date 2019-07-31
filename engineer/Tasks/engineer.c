#include "engineer.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "param.h"
#include "ramp.h"
#include "can.h"
#include "board.h"
#include "dbus.h"
#include "dualmotor.h"
#include "engg_gpio.h"
#include "upper.h"

/* VARIABLES: ENGINEER - related */
extern struct upper_info upper;
/* VARIABLES: END of ENGINEER - related */

/* CONSTANTS: ENGINEER - related */
Engineer engg;
/* CONSTANTS: END of ENGINEER - related */

/* FUNCTIONS: ENGINEER - related */
static void engineer_state_handler(rc_device_t prc_dev, rc_info_t prc_info) {
	if (rc_device_get_state(prc_dev, RC_S1_UP) == RM_OK) {
		engg.ENGINEER_BIG_STATE = UPPERPART;
		if (rc_device_get_state(prc_dev, RC_S2_UP) == RM_OK)
			engg.ENGINEER_SMALL_STATE = SINGLE_LOCATE;
		if (rc_device_get_state(prc_dev, RC_S2_MID) == RM_OK)
			engg.ENGINEER_SMALL_STATE = TRIO_LOCATE;
		if (rc_device_get_state(prc_dev, RC_S2_DOWN) == RM_OK)
			engg.ENGINEER_SMALL_STATE = PENTA_LOCATE;
	}
	if (rc_device_get_state(prc_dev, RC_S1_MID) == RM_OK) {
		engg.ENGINEER_BIG_STATE = LOWERPART;
		if (rc_device_get_state(prc_dev, RC_S2_UP) == RM_OK)
			engg.ENGINEER_SMALL_STATE = REVERSE_CHASSIS;
		if (rc_device_get_state(prc_dev, RC_S2_MID) == RM_OK)
			engg.ENGINEER_SMALL_STATE = CHASSIS;
		if (rc_device_get_state(prc_dev, RC_S2_DOWN) == RM_OK)
			engg.ENGINEER_SMALL_STATE = UNLOAD;
	}
	if (rc_device_get_state(prc_dev, RC_S1_DOWN) == RM_OK) {
		engg.ENGINEER_BIG_STATE = MANAGE;
		if (rc_device_get_state(prc_dev, RC_S2_UP) == RM_OK)
			engg.ENGINEER_SMALL_STATE = RESET;
		if (rc_device_get_state(prc_dev, RC_S2_MID) == RM_OK)
			engg.ENGINEER_SMALL_STATE = CHASSIS_ONLY;
		if (rc_device_get_state(prc_dev, RC_S2_DOWN) == RM_OK)
			engg.ENGINEER_SMALL_STATE = OFF;
	}
}
/* END of FUNCTIONS: ENGINEER - related */

/* RTOS: ENGINEER - related */
void engineer_task(void const *argument)
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
		engineer_state_handler(prc_dev, prc_info);
		
		osDelayUntil(&period, 2);
	}
}
/* END of RTOS: ENGINEER - related */

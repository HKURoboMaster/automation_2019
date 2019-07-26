#include "sub_engineer.h"
#include "dbus.h"
#include "engineer.h"

/* VARIABLES: SUB_ENGINEER - related */
/* VARIABLES: END of SUB_ENGINEER - related */

/* CONSTANTS: SUB_ENGINEER - related */
Sub_Engineer sub_engg;
/* CONSTANTS: END of SUB_ENGINEER - related */

/* FUNCTIONS: SUB_ENGINEER - related */
static void sub_engineer_state_handler(rc_device_t prc_dev, rc_info_t prc_info) {
	if (rc_device_get_state(prc_dev, RC_S1_UP)) {
		sub_engg.ENGINEER_BIG_STATE = UPPERPART;
		if (rc_device_get_state(prc_dev, RC_S2_UP) == RM_OK)
			sub_engg.ENGINEER_SMALL_STATE = SINGLE_LOCATE;
		if (rc_device_get_state(prc_dev, RC_S2_MID) == RM_OK)
			sub_engg.ENGINEER_SMALL_STATE = TRIO_LOCATE;
		if (rc_device_get_state(prc_dev, RC_S2_DOWN) == RM_OK)
			sub_engg.ENGINEER_SMALL_STATE = PENTA_LOCATE;
	}
	if (rc_device_get_state(prc_dev, RC_S1_MID) == RM_OK) {
		sub_engg.ENGINEER_BIG_STATE = LOWERPART;
		if (rc_device_get_state(prc_dev, RC_S2_UP) == RM_OK)
			sub_engg.ENGINEER_SMALL_STATE = REVERSE_CHASSIS;
		if (rc_device_get_state(prc_dev, RC_S2_MID) == RM_OK)
			sub_engg.ENGINEER_SMALL_STATE = CHASSIS;
		if (rc_device_get_state(prc_dev, RC_S2_DOWN) == RM_OK)
			sub_engg.ENGINEER_SMALL_STATE = UNLOAD;
	}
	if (rc_device_get_state(prc_dev, RC_S1_DOWN)) {
		sub_engg.ENGINEER_BIG_STATE = MANAGE;
		if (rc_device_get_state(prc_dev, RC_S2_UP) == RM_OK)
			sub_engg.ENGINEER_SMALL_STATE = RESET;
		if (rc_device_get_state(prc_dev, RC_S2_MID) == RM_OK)
			sub_engg.ENGINEER_SMALL_STATE = CHASSIS_ONLY;
		if (rc_device_get_state(prc_dev, RC_S2_DOWN) == RM_OK)
			sub_engg.ENGINEER_SMALL_STATE = OFF;
	}
}
/* END of FUNCTIONS: SUB_ENGINEER - related */

/* RTOS: SUB_ENGINEER - related */
void sub_engineer_task(void const *argument)
{
	uint32_t period = osKernelSysTick();	
	
	rc_device_t prc_dev = NULL;
	rc_info_t prc_info = NULL;
	
	prc_dev = rc_device_find("can_rc");
  if (prc_dev != NULL)
  {
    prc_info = rc_device_get_info(prc_dev);
  }

	for(;;) {
		sub_engineer_state_handler(prc_dev, prc_info);
		
		osDelayUntil(&period, 2);
	}
}
/* END of RTOS: SUB_ENGINEER - related */

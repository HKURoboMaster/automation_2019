#include "upper.h"
#include "dbus.h"
#include "engineer.h"
#include "sub_engineer.h"

/* VARIABLES: UPPER - related */
struct upper_info upper;
upper_ctrl upper_controller;
extern Sub_Engineer sub_engg;
/* END of VARIABLES: UPPER - related */

/* FUNCTIONS: UPPER - related */
int32_t upper_execute(Sub_Engineer* sub_engineer, struct upper_info* upperinf, upper_ctrl* upperctrl, rc_device_t prc_dev, rc_info_t prc_info) {
	if (upperinf == NULL || upperctrl == NULL)
		return -RM_INVAL;
	
	if (sub_engineer->ENGINEER_BIG_STATE != UPPERPART) {
		// reset
		return RM_OK;
	}
	
	if (sub_engineer->ENGINEER_SMALL_STATE == SINGLE_LOCATE || sub_engineer->ENGINEER_SMALL_STATE == TRIO_LOCATE
			|| sub_engineer->ENGINEER_SMALL_STATE == PENTA_LOCATE) {
				
	
		if (upperinf->mode == LOCATED) {
			if (rc_device_get_state(prc_dev, RC_WHEEL_DOWN) == RM_OK && prc_dev->last_rc_info.wheel > -300) {
				if (sub_engineer->ENGINEER_SMALL_STATE == SINGLE_LOCATE) {
					
				}
				else if (sub_engineer->ENGINEER_SMALL_STATE == TRIO_LOCATE) {
					
				}
				else if (sub_engineer->ENGINEER_SMALL_STATE == PENTA_LOCATE) {
					
				}
			}
		}
	}
	
	return RM_OK;
}

void set_upper_info(int mode) {
	upper.mode = mode;
}
/* END of FUNCTIONS: UPPER - related */

/* RTOS: UPPER - related */
void upper_task(void const *argument)
{
  uint32_t period = osKernelSysTick();
	
	upper_controller.dualMotor.rest_angle = REST_ANGLE;
	upper_controller.dualMotor.rise_angle = RISE_ANGLE;
	
	rc_device_t prc_dev = NULL;
	rc_info_t prc_info = NULL;
	
	prc_dev = rc_device_find("can_rc");
  if (prc_dev != NULL)
  {
    prc_info = rc_device_get_info(prc_dev);
  }
		
	for(;;) {
		upper_execute(&sub_engg, &upper, &upper_controller, prc_dev, prc_info);
    osDelayUntil(&period, 2);
	}
}
/* END of RTOS: UPPER - related */

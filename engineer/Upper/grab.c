#include "grab.h"
#include "sub_engineer.h"
#include "engineer.h"
#include "dbus.h"
#include "engg_gpio.h"
#include "timer_task.h"
#include "infantry_cmd.h"
#include "upper.h"

/* VARIABLES: GRAB - related */
extern Sub_Engineer sub_engg;
extern struct upper_info upper;
/* END of VARIABLES: GRAB - related */

/* FUNCTIONS: GRAB - related */
int32_t grab_execute(Sub_Engineer* sub_engineer, struct upper_info* upperinf, rc_device_t prc_dev, rc_info_t prc_info) {
	if (sub_engineer == NULL)
		return -RM_INVAL;
	
	if (sub_engineer->ENGINEER_BIG_STATE != UPPERPART) {
		// reset
		if (sub_engineer->grabber.GRABBER_STATE != IDLE) {
			sub_engineer->grabber.GRABBER_STATE = IDLE;
			upperinf->mode = IDLE;
		}
		return RM_OK;
	}
	
	if (sub_engineer->ENGINEER_SMALL_STATE == SINGLE_LOCATE || sub_engineer->ENGINEER_SMALL_STATE ==TRIO_LOCATE
			|| sub_engineer->ENGINEER_SMALL_STATE == PENTA_LOCATE) {
		if (sub_engineer->grabber.GRABBER_STATE == IDLE) {
			sub_engineer->grabber.GRABBER_STATE = LOCATING;
			upperinf->mode = LOCATING;
		}
		
		if (sub_engineer->grabber.GRABBER_STATE == LOCATING) {
			if (read_sonicL() && read_sonicR()) {
				sub_engineer->grabber.GRABBER_STATE = LOCATED;
				upperinf->mode = LOCATED;
			}
		}
	}
	
	return RM_OK;
}
/* END of FUNCTIONS: GRAB - related */

/* RTOS: GRAB - related */
void grab_task(void const *argument)
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
		grab_execute(&sub_engg, &upper, prc_dev, prc_info);
    osDelayUntil(&period, 2);
	}
}
/* END of RTOS: GRAB - related */

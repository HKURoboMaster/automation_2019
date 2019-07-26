#include "grab.h"
#include "engineer.h"
#include "dbus.h"
#include "engg_gpio.h"
#include "timer_task.h"
#include "infantry_cmd.h"
#include "upper.h"

/* VARIABLES: GRAB - related */
extern Engineer engg;
extern struct upper_info upper;
/* END of VARIABLES: GRAB - related */

/* FUNCTIONS: GRAB - related */
int32_t grab_execute(Engineer* engineer, struct upper_info* upperinf, rc_device_t prc_dev, rc_info_t prc_info) {
	if (engineer == NULL)
		return -RM_INVAL;
	
	if (engineer->ENGINEER_BIG_STATE != UPPERPART) {
		// reset
		if (engineer->grabber.GRABBER_STATE != IDLE) {
			engineer->grabber.GRABBER_STATE = IDLE;
			upperinf->mode = IDLE;
		}
		if (engineer->HALT_CHASSIS != 0)
			engineer->HALT_CHASSIS = 0;
		return RM_OK;
	}
	
	if (engineer->ENGINEER_SMALL_STATE == SINGLE_LOCATE || engineer->ENGINEER_SMALL_STATE ==TRIO_LOCATE
			|| engineer->ENGINEER_SMALL_STATE == PENTA_LOCATE) {
		if (engineer->grabber.GRABBER_STATE == IDLE) {
			engineer->grabber.GRABBER_STATE = LOCATING;
			upperinf->mode = LOCATING;
		}
		
		if (engineer->grabber.GRABBER_STATE == LOCATING) {
			if (read_sonicL() && read_sonicR()) {
				engg.HALT_CHASSIS = 1;
				engineer->grabber.GRABBER_STATE = LOCATED;
				upperinf->mode = LOCATED;
			}
		}
		
		if (engineer->grabber.GRABBER_STATE == LOCATED) {
			if (rc_device_get_state(prc_dev, RC_WHEEL_DOWN) == RM_OK && prc_dev->last_rc_info.wheel > -300) {
				if (engineer->ENGINEER_SMALL_STATE == SINGLE_LOCATE) {
					
				}
				else if (engineer->ENGINEER_SMALL_STATE == TRIO_LOCATE) {
					
				}
				else if (engineer->ENGINEER_SMALL_STATE == PENTA_LOCATE) {
					
				}
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
	
	prc_dev = rc_device_find("uart_rc");
  if (prc_dev != NULL)
  {
    prc_info = rc_device_get_info(prc_dev);
  }
	
	soft_timer_register(upper_push_info, (void *)&upper, 10);
	
	for(;;) {
		grab_execute(&engg, &upper, prc_dev, prc_info);
    osDelayUntil(&period, 2);
	}
}
/* END of RTOS: GRAB - related */

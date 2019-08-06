#include "upper.h"
#include "dbus.h"
#include "engineer.h"
#include "sub_engineer.h"
#include "pneumatic.h"
#include "engg_gpio.h"
#include "upper_pneumatics.h"
#include "infantry_cmd.h"
#include "timer_task.h"

#include "flipper.h"
#include "extender.h"
#include "slider.h"
#include "seizer.h"

/* VARIABLES: UPPER - related */
struct upper_info upper;
upper_ctrl upper_controller;
extern Sub_Engineer sub_engg;

extern Extender extender;
extern Slider slider;
/* END of VARIABLES: UPPER - related */

/* FUNCTIONS: UPPER - related */
void reset_to_grab_state(upper_ctrl* upperctrl);
void grab_n_dunk(upper_ctrl* upperctrl);
void grab_n_throw(upper_ctrl* upperctrl);
void throw_box(upper_ctrl* upperctrl);

int ok_to_grab();

int32_t upper_execute(Sub_Engineer* sub_engineer, struct upper_info* upperinf, upper_ctrl* upperctrl, rc_device_t prc_dev, rc_info_t prc_info) {
	if (upperinf == NULL || upperctrl == NULL)
		return -RM_INVAL;
	
	if (sub_engineer->ENGINEER_BIG_STATE != UPPERPART) {
		// reset
		upperctrl->START_GRABBING = NO_GRAB;
		flipper_state_change(REST);
		seizer_state_change(RELEASE);
		rotate_to_zero(upperctrl);
		return RM_OK;
	}
	
	if (sub_engineer->ENGINEER_SMALL_STATE == PENTA_LOCATE) {
		extender_state_change(EXTEND);
	}
	else {
		extender_state_change(RETRACT);
	}
	
	if (sub_engineer->ENGINEER_SMALL_STATE == SINGLE_LOCATE || sub_engineer->ENGINEER_SMALL_STATE == TRIO_LOCATE ||
			sub_engineer->ENGINEER_SMALL_STATE == PENTA_LOCATE) {
		if (upperctrl->CURRENT_STATE == GRAB_READY) {
			reset_to_grab_state(upperctrl);
		}
		else if (upperctrl->CURRENT_STATE == DUNKING) {
			//grab_n_dunk(upperctrl);
			grab_n_throw(upperctrl);
			
		}
		
		if (rc_device_get_state(prc_dev, RC_WHEEL_DOWN) == RM_OK && upperctrl->CURRENT_STATE == GRAB_READY) {
			upperctrl->CURRENT_STATE = DUNKING;
		}
		else if (rc_device_get_state(prc_dev, RC_WHEEL_UP) == RM_OK && upperctrl->CURRENT_STATE == DUNKING) {
			//upperctrl->CURRENT_STATE = THROWING;
		}
		
		//extender_state_change(EXTEND);
		//flipper_state_change(FLIP);
		/*
		rotate_to_grab(upperctrl);
		osDelay(2000);
		seizer_state_change(SEIZE);
		osDelay(2000);
		rotate_to_zero(upperctrl);
		osDelay(2000);
		seizer_state_change(RELEASE);
		*/
	}
	
	/*
	if (sub_engineer->ENGINEER_SMALL_STATE == SINGLE_LOCATE || sub_engineer->ENGINEER_SMALL_STATE == TRIO_LOCATE
			|| sub_engineer->ENGINEER_SMALL_STATE == PENTA_LOCATE) {
			
		if (upperinf->mode == LOCATED) {
			if (rc_device_get_state(prc_dev, RC_WHEEL_DOWN) == RM_OK) {
				upperctrl->START_GRABBING = START_GRAB;
			}	

			if (upperctrl->START_GRABBING == START_GRAB) {
				reset_to_grab_state();
				
				if (ok_to_grab()) {
					if (sub_engineer->ENGINEER_SMALL_STATE == SINGLE_LOCATE) {
						
					}
					else if (sub_engineer->ENGINEER_SMALL_STATE == TRIO_LOCATE) {
					
					}
					else if (sub_engineer->ENGINEER_SMALL_STATE == PENTA_LOCATE) {
					
					}
				}
			}
		}
	}*/
	
	return RM_OK;
}

void set_upper_info(int mode) {
	upper.mode = mode;
}

void reset_to_grab_state(upper_ctrl* upperctrl) {
	rotate_to_grab(upperctrl);
	seizer_state_change(RELEASE);
}

void grab_n_dunk(upper_ctrl* upperctrl) {
	seizer_state_change(SEIZE);
	osDelay(100);
	rotate_to_zero(upperctrl);
}

void grab_n_throw(upper_ctrl* upperctrl) {
	rotate_to_grabbing(upperctrl);
	osDelay(2000);
	seizer_state_change(SEIZE);
	osDelay(2000);
	rotate_to_zero(upperctrl);
	osDelay(2000);
	seizer_state_change(RELEASE);
	osDelay(2000);
	flipper_state_change(FLIP);
	osDelay(2000);
	flipper_state_change(REST);
	upperctrl->CURRENT_STATE = GRAB_READY;
}

void throw_box(upper_ctrl* upperctrl) {
	rotate_to_grab(upperctrl);
	osDelay(50);
	seizer_state_change(RELEASE);
}

int ok_to_grab() {
	if (slider.state == SLIDER_toCenter  &&
			return_seizer_state() == SEIZER_RELEASED &&
			extender.state == EXTENDER_RETRACTED)
		return 1;
	else
		return 0;
}
/* END of FUNCTIONS: UPPER - related */

/* RTOS: UPPER - related */
void upper_task(void const *argument)
{
  uint32_t period = osKernelSysTick();
	
	upper_controller.dualMotor.rest_angle = REST_ANGLE;
	upper_controller.dualMotor.rise_angle = RISE_ANGLE;
	upper_controller.dualMotor.grab_angle = GRAB_ANGLE;
	
	rc_device_t prc_dev = NULL;
	rc_info_t prc_info = NULL;
	
	prc_dev = rc_device_find("can_rc");
  if (prc_dev != NULL)
  {
    prc_info = rc_device_get_info(prc_dev);
  }
	
	soft_timer_register(upper_push_info, (void *)&upper, 10);
	
	upper_controller.CURRENT_STATE = GRAB_READY;
	
	for(;;) {
		upper_execute(&sub_engg, &upper, &upper_controller, prc_dev, prc_info);

    osDelayUntil(&period, 2);
	}
}
/* END of RTOS: UPPER - related */

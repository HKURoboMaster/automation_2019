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

extern Flipper flipper;
extern Extender extender;
extern Slider slider;
extern Seizer seizer;
/* END of VARIABLES: UPPER - related */

/* FUNCTIONS: UPPER - related */
void reset_to_grab_state(void);
int ok_to_grab();

int32_t upper_execute(Sub_Engineer* sub_engineer, struct upper_info* upperinf, upper_ctrl* upperctrl, UpperPneum* upperpneum, rc_device_t prc_dev, rc_info_t prc_info) {
	if (upperinf == NULL || upperctrl == NULL)
		return -RM_INVAL;
	
	if (sub_engineer->ENGINEER_BIG_STATE != UPPERPART) {
		// reset
		upperctrl->START_GRABBING = NO_GRAB;
		return RM_OK;
	}
	
	if (sub_engineer->ENGINEER_SMALL_STATE == SINGLE_LOCATE) {
		flipper.request = FLIP;
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

void reset_to_grab_state(void) {
	if (flipper.state != FLIPPER_RESTED)
		flipper.request = FLIP;
	if (slider.state != SLIDER_toCenter)
		slider.request = SLIDER_toCenter;
	if (seizer.state != SEIZER_RELEASED)
		seizer.request = RELEASE;
	if (extender.state != EXTENDER_RETRACTED)
		extender.request = RETRACT;
}

int ok_to_grab() {
	if (slider.state == SLIDER_toCenter  &&
			seizer.state == SEIZER_RELEASED &&
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
	
	rc_device_t prc_dev = NULL;
	rc_info_t prc_info = NULL;
	
	prc_dev = rc_device_find("can_rc");
  if (prc_dev != NULL)
  {
    prc_info = rc_device_get_info(prc_dev);
  }
	
	soft_timer_register(upper_push_info, (void *)&upper, 10);
	
	UpperPneum upperpneum;
	upperpneum.flipper = init_pneum_1head(FLIP_GPIO_Port, FLIP_Pin);
	upperpneum.flipper_state = EXTENDED;
	upperpneum.extender = init_pneum_1head(EXTEND_GPIO_Port, EXTEND_Pin);
	upperpneum.extender_state = RETRACTED;
	
	for(;;) {
		upper_execute(&sub_engg, &upper, &upper_controller, &upperpneum, prc_dev, prc_info);

    osDelayUntil(&period, 2);
	}
}
/* END of RTOS: UPPER - related */

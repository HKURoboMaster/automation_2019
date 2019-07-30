#include "slider.h"
#include "pneumatic.h"
#include "upper.h"
#include "engg_gpio.h"
#include "engg_adc.h"

/* VARIABLES: SLIDER - related */
Slider slider;
extern struct upper_info upper;
extern upper_ctrl upper_controller;
/* END of VARIABLES: SLIDER - related */

/* FUNCTIONS: SLIDER - related */
void slider_toLeft();
void slider_toCenterLeft();
void slider_toCenter();
void slider_toCenterRight();
void slider_toRight();

int32_t slider_execute(struct upper_info* upperinf, upper_ctrl* upperctrl) {
	if (upperinf == NULL || upperctrl == NULL)
		return -RM_INVAL;
	
	if (slider.request == SLIDER_toLeft)
		slider_toLeft();
	else if (slider.request == SLIDER_toCenterLeft)
		slider_toCenterLeft();
	else if (slider.request == SLIDER_toCenter)
		slider_toCenter();	
	else if (slider.request == SLIDER_toCenterRight)
		slider_toCenterRight();
	else if (slider.request == SLIDER_toRight)
		slider_toRight();
	
	return RM_OK;
}

void slider_toLeft(void) {
	if (slider.state > SLIDER_toLeft) {
		pneum_2head_toLeft(&(slider.cylinderL), &(slider.cylinderR));
		osDelay(SLIDER_GAP_MILLISEC * (slider.state - SLIDER_toLeft));
		slider.state = SLIDER_toLeft;
	}
}

void slider_toCenterLeft(void) {
	if (slider.state > SLIDER_toCenterLeft) {
		if (poll_magnet_sensor_CL() > MS_tol) {
			pneum_2head_toLeft(&(slider.cylinderL), &(slider.cylinderR));
		}
		else {
			pneum_2head_HOLD(&(slider.cylinderL), &(slider.cylinderR));
			slider.state = SLIDER_toCenterLeft;
		}
	}
	else if (slider.state < SLIDER_toCenterLeft) {
		if (poll_magnet_sensor_CL() > MS_tol) {
			pneum_2head_toRight(&(slider.cylinderL), &(slider.cylinderR));
		}
		else {
			pneum_2head_HOLD(&(slider.cylinderL), &(slider.cylinderR));
			slider.state = SLIDER_toCenterLeft;
		}
	}
}

void slider_toCenter(void) {
	if (slider.state > SLIDER_toCenter) {
		if (poll_magnet_sensor_C() > MS_tol) {
			pneum_2head_toLeft(&(slider.cylinderL), &(slider.cylinderR));
		}
		else {
			pneum_2head_HOLD(&(slider.cylinderL), &(slider.cylinderR));
			slider.state = SLIDER_toCenter;
		}
	}
	else if (slider.state < SLIDER_toCenter) {
		if (poll_magnet_sensor_C() > MS_tol) {
			pneum_2head_toRight(&(slider.cylinderL), &(slider.cylinderR));
		}
		else {
			pneum_2head_HOLD(&(slider.cylinderL), &(slider.cylinderR));
			slider.state = SLIDER_toCenter;
		}
	}
}

void slider_toCenterRight(void) {
	if (slider.state > SLIDER_toCenterRight) {
		if (poll_magnet_sensor_CR() > MS_tol) {
			pneum_2head_toLeft(&(slider.cylinderL), &(slider.cylinderR));
		}
		else {
			pneum_2head_HOLD(&(slider.cylinderL), &(slider.cylinderR));
			slider.state = SLIDER_toCenterRight;
		}
	}
	else if (slider.state < SLIDER_toCenterRight) {
		if (poll_magnet_sensor_CR() > MS_tol) {
			pneum_2head_toRight(&(slider.cylinderL), &(slider.cylinderR));
		}
		else {
			pneum_2head_HOLD(&(slider.cylinderL), &(slider.cylinderR));
			slider.state = SLIDER_toCenterRight;
		}
	}
}

void slider_toRight(void) {
	if (slider.state < SLIDER_toRight) {
		pneum_2head_toRight(&(slider.cylinderL), &(slider.cylinderR));
		osDelay(SLIDER_GAP_MILLISEC * (SLIDER_toRight - slider.state));
		slider.state = SLIDER_toRight;
	}
}
/* END of FUNCTIONS: SLIDER - related */

/* RTOS: SLIDER - related */
void slider_task(void const *argument)
{
  uint32_t period = osKernelSysTick();
	
	slider.cylinderL = init_pneum_1head(SLIDEL_GPIO_Port, SLIDEL_Pin);
	slider.cylinderR = init_pneum_1head(SLIDER_GPIO_Port, SLIDER_Pin);
	
	for(;;) {
		slider_execute(&upper, &upper_controller);

    osDelayUntil(&period, 2);
	}
}
/* END of RTOS: SLIDER - related */

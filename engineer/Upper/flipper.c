#include "flipper.h"
#include "pneumatic.h"
#include "upper.h"
#include "engg_gpio.h"

/* VARIABLES: FLIPPER - related */
Flipper flipper;
extern struct upper_info upper;
extern upper_ctrl upper_controller;
/* END of VARIABLES: FLIPPER - related */

/* FUNCTIONS: FLIPPER - related */
void flipper_flip(void);
void flipper_rest(void);
void flipper_rest_cd(void);
void flipper_flip_cd(void);

int32_t flipper_execute(struct upper_info* upperinf, upper_ctrl* upperctrl) {
	if (upperinf == NULL || upperctrl == NULL)
		return -RM_INVAL;
	
	if (flipper.request == REST) {
		flipper_rest();
	}
	else if (flipper.request == FLIP) {
		flipper_flip();
	}
	
	return RM_OK;
}

void flipper_flip_cd(void) {
	if (flipper.state != FLIPPER_FLIPPED) {
		osDelay(FLIPPER_FLIPPED_MS);
		flipper.state = FLIPPER_FLIPPED;
	}
}
void flipper_flip(void) {
	pneum_1head_LOW(&(flipper.cylinderX2));
}

void flipper_rest_cd(void) {
	if (flipper.state != FLIPPER_RESTED) {
		osDelay(FLIPPER_RESTED_MS);
		flipper.state = FLIPPER_RESTED;
	}
}

void flipper_rest(void) {
	pneum_1head_HIGH(&(flipper.cylinderX2));
}

void flipper_state_change(int state) {
	flipper.request = state;
}

int return_flipper_state(void) {
	return flipper.state;
}
/* END of FUNCTIONS: FLIPPER - related */

/* RTOS: FLIPPER - related */
void flipper_task(void const *argument)
{
  uint32_t period = osKernelSysTick();
	
	flipper.cylinderX2 = init_pneum_1head(FLIP_GPIO_Port, FLIP_Pin);
	flipper.state = FLIPPER_RESTED;
	flipper.request = REST;
	
	for(;;) {
		flipper_execute(&upper, &upper_controller);

    osDelayUntil(&period, 2);
	}
}
/* END of RTOS: FLIPPER - related */

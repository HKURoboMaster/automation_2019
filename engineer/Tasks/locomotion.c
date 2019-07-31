#include "locomotion.h"
#include "engineer.h"
#include "dbus.h"
#include "servos.h"
#include "engg_gpio.h"
#include "upper.h"

/* VARIABLES: LOCOMOTION - related */
extern Engineer engg;
extern struct upper_info upper;
extern int camservo_state;
Servo reload_0 = {
	0, 0, 270, 0, 4096, 0, 180, 16
};
	
Servo reload_1 = {
	1, 0, 270, 0, 4096, 0, 180, 16
};

extern Servo RMcam;
/* END of VARIABLES: LOCOMOTION - related */

/* FUNCTIONS: LOCOMOTION - related */
int unlock_ammobox();
int lock_ammobox();
int ungate_ammobox();
int gate_ammobox();
int permlock_ammobox(Ammobox* ammobox);
int permunlock_ammobox(Ammobox* ammobox);

int32_t locomotion_execute(Engineer* engineer, Ammobox* ammobox, rc_device_t prc_dev, rc_info_t prc_info) {
	if (engineer == NULL)
		return -RM_INVAL;
	
	if (engineer->ENGINEER_BIG_STATE != LOWERPART) {
		// look forward and close servos
		permlock_ammobox(ammobox);
	}
	else {
		if (engineer->HALT_CHASSIS != 0)
			engineer->HALT_CHASSIS = 0;
	}
	
	if (engineer->ENGINEER_SMALL_STATE != UNLOAD) {
		permlock_ammobox(ammobox);
	}
	
	if (engineer->ENGINEER_SMALL_STATE == CHASSIS) {
		camservo_state = use_RMcam();
	}
	else if (engineer->ENGINEER_SMALL_STATE == REVERSE_CHASSIS) {
		use_rear_cam();
	}
	else if (engineer->ENGINEER_SMALL_STATE == UNLOAD) {
		permunlock_ammobox(ammobox);
	}
	
	if (engineer->ENGINEER_BIG_STATE == UPPERPART) {
		use_front_cam();
		if (upper.mode == LOCATED) {
			engineer->HALT_CHASSIS = 1;
		}
		else {
			engineer->HALT_CHASSIS = 0;
		}
	}
	
	if (engineer->ENGINEER_BIG_STATE == MANAGE) {
		camservo_state = use_RMcam();
	}

	return RM_OK;
}

int unlock_ammobox() {
	if (SERVO_fallTo(&reload_1, 160) == SERVO_DONE) {
		reload_1.current_index = 0;
		return UNLOCKED;
	}
	return LOCKED;
}

int lock_ammobox() {
	if (SERVO_raiseTo(&reload_1, 16) == SERVO_DONE) {
		reload_1.current_index = 0;
		return LOCKED;
	}
	return UNLOCKED;
	
}

int ungate_ammobox() {
	if (SERVO_fallTo(&reload_0, 190) == SERVO_DONE) {
		reload_0.current_index = 0;
		return UNGATED;
	}
	return GATED;
}

int gate_ammobox() {
	if (SERVO_raiseTo(&reload_0, 190) == SERVO_DONE) {
		reload_0.current_index = 0;
		return GATED;
	}
	return UNGATED;
}

int permlock_ammobox(Ammobox* ammobox) {
	if (ammobox->boxgate_state == UNGATED) {
		ammobox->boxgate_state = gate_ammobox();
		osDelay(5);
		return PERM_UNLOCKED;
	}
	if (ammobox->boxlock_state == UNLOCKED) {
		ammobox->boxlock_state = lock_ammobox();
		osDelay(5);
		return PERM_UNLOCKED;
	}
	return PERM_LOCKED;
}

int permunlock_ammobox(Ammobox* ammobox) {
	if (ammobox->boxlock_state == LOCKED) {
		ammobox->boxlock_state = unlock_ammobox();
		osDelay(5);
		return PERM_LOCKED;
	}
	if (ammobox->boxgate_state == GATED) {
		ammobox->boxgate_state = ungate_ammobox();
		osDelay(5);
		return PERM_LOCKED;
	}
	return PERM_UNLOCKED;
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
	
	Ammobox ammobox;
	
	for (int i = 0; i < 255; ++i) {
		PCA9685_transmit(reload_0.servoNo, 0, reload_0.maxPulse - reload_0.step_size * i);
		PCA9685_transmit(reload_1.servoNo, 0, reload_1.step_size * i);
	}
	osDelay(100);
	for (int i = 0; i < 255; ++i) {
		PCA9685_transmit(reload_0.servoNo, 0, reload_0.step_size * i);
	}
	
	for (int i = 0; i < 51; ++i) {
		PCA9685_transmit(RMcam.servoNo, 0, RMcam.step_size * i);
	}
	
	ammobox.boxlock_state = UNLOCKED;
	ammobox.boxgate_state = GATED;
	camservo_state = FRONT;	
	
	for(;;) {
		locomotion_execute(&engg, &ammobox, prc_dev, prc_info);
    osDelayUntil(&period, 2);
	}
}
/* END of RTOS: LOCOMOTION - related */

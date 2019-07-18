#include "grab.h"
#include "engineer.h"
#include "dbus.h"
#include "engg_gpio.h"
#include "arduino.h"

/* VARIABLES: GRAB - related */
extern Engineer engg;
int comm_flag = 0;	// By default the communication is not needed.
Ard_Rx rec_data  = {64,0,0};
Ard_Tx tran_data = {64,0,0};
int debug_js = 0;
/* END of VARIABLES: GRAB - related */

/* FUNCTIONS: GRAB - related */
void get_slave_Info(int comm_flag)
{
	if(comm_flag == 1)
		ard_send_msg(tran_data);	// send data from data from Arduino.
	ard_receive_msg(&rec_data);	// Acquire data from Arduino.
	
}

int32_t grab_execute(Engineer* engineer, rc_device_t prc_dev, rc_info_t prc_info) 
{
	if (engineer == NULL)
		return -RM_INVAL;
	
	if (engineer->ENGINEER_BIG_STATE != UPPERPART) 
	{
		// reset
		if (engineer->grabber.GRABBER_STATE != IDLE)
			engineer->grabber.GRABBER_STATE = IDLE;
		if (engineer->HALT_CHASSIS != 0)
			engineer->HALT_CHASSIS = 0;
		tran_data.command_flag = NULL_TASK;
		tran_data.rotation_flag[0] = 0;
		tran_data.rotation_flag[1] = 0;
		get_slave_Info(1);
		return RM_OK;
	}
	if(engineer->ENGINEER_BIG_STATE == UPPERPART)
	{
		if(read_sonicL() && read_sonicR())
		{
			engineer->grabber.GRABBER_STATE =LOCATED;
		}
		else
		{
			engineer->grabber.GRABBER_STATE = LOCATING;
		}
	}
	if (engineer->grabber.GRABBER_STATE == LOCATED) 
	{
		// Do something to indicate the task is done
		
	}
	if (rc_device_get_state(prc_dev, RC_WHEEL_DOWN) == RM_OK && prc_dev->last_rc_info.wheel > -300) 
	{
		if (engineer->ENGINEER_SMALL_STATE == SINGLE_LOCATE) 
				{
					// Send Information to the Arduino TASK2
					tran_data.command_flag = TASK2;
					if (rec_data.current_task == NULL_TASK)
						comm_flag = 1;
					else
						comm_flag = 0;
					// Enable Information Enable flag
				}
		else if (engineer->ENGINEER_SMALL_STATE == THREE_LOCATE) 
			{
					// Send Information to the Arduino TASK3
					// Enable Infomation Acquire flag
					tran_data.command_flag = TASK3;
					if (rec_data.current_task == NULL_TASK)
						comm_flag = 1;
					else
						comm_flag = 0;
			}
	else if (engineer->ENGINEER_SMALL_STATE == FIVE_LOCATE) 
	{
		// Send Information to the Arduino TASK1.
		// Enable Infomation Acquire Flag
		// 
		tran_data.command_flag = TASK1;
		if (rec_data.current_task == NULL_TASK)
			comm_flag = 1;
		else
			comm_flag = 0;
	}
  }
	if(rc_device_get_state(prc_dev,RC_WHEEL_UP)==RM_OK && prc_dev-> last_rc_info.wheel < 300)
	{
		// Send Information to Arduino RESET.
		tran_data.command_flag = RESET_TASK;
		comm_flag = 1;
	}
	else if(prc_dev->rc_info.kb.bit.E == 1)
	{
		// Send Information to Arduino EJECT
		// EJECT should have pre-request that claw is rotate front / Claw is open.
		// Claw Eject rotate will be checked here.
		// Claw open will be checked on Arduino.
		// Eject task shouldn't be blocking
		tran_data.command_flag = EJECT_TASK;
		if(tran_data.rotation_flag[1]==ROTATION_DONE)
			comm_flag = 1;
		else
			comm_flag = 0;
	}
	else
	{
		//check if task is done.
		// Once arduino is done no extra task should be ever assigned.
		if(rec_data.current_task == NULL_TASK)
		{
			tran_data.command_flag = NULL_TASK;
			comm_flag = 1;
		}
	}
	get_slave_Info(comm_flag);
	//Operation on the dual Motor.
	if(rec_data.current_task==TASK1 || rec_data.current_task == TASK2 || rec_data.current_task==TASK3)
	{
		if(rec_data.rotation_ok[0]==ROTATION_OK)
			engineer->dualMotor.DUALMOTOR_STATE = CLAW_RISE;
		else if(rec_data.rotation_ok[0]==ROTATION_IDLE)
			engineer->dualMotor.DUALMOTOR_STATE =	CLAW_IDLE;
		else if(rec_data.rotation_ok[1]==ROTATION_OK)
			engineer->dualMotor.DUALMOTOR_STATE = CLAW_RISE;
		else if(rec_data.rotation_ok[0]==ROTATION_IDLE)
			engineer->dualMotor.DUALMOTOR_STATE = CLAW_IDLE;
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
	
	for(;;) {
		grab_execute(&engg, prc_dev, prc_info);
    osDelayUntil(&period, 10);	// Change Gap time to prevent mess up arduino
	}
}
/* END of RTOS: GRAB - related */

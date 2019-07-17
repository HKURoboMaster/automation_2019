#ifndef __ARDUINO_H_
#define __ARDUINO_H_
#include "cmsis_os.h"

#define ARDUINO_SLAVE 0x04u

//Interactive Information define
#define TASK1           0
#define TASK2           1
#define TASK3           2
#define RESET_TASK      3
#define EJECT_TASK      4
#define NULL_TASK       1024

#define ROTATION_OK			1
#define ROTATION_IDLE		0
#define ROTATION_DONE		1
#define ROTATION_BUSY		0

typedef struct stm2Ard
{
	int command_flag;
	int rotation_flag[2];
} Ard_Tx;

typedef struct Ard2stm
{
	int current_task;
	int rotation_ok[2];
} Ard_Rx;

int ard_Init(void);
int ard_send_msg(Ard_Tx msg);
int ard_receive_msg(Ard_Rx msg);

#endif
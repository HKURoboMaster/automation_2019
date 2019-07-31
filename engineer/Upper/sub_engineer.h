#ifndef SUB_ENGINEER_H
#define SUB_ENGINEER_H

#include "grab.h"

typedef struct Sub_Engineer {
	int ENGINEER_BIG_STATE;
	int ENGINEER_SMALL_STATE;
	
	struct Grabber grabber;
} Sub_Engineer;

void sub_engineer_task(void const *argument);

#endif

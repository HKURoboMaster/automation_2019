#ifndef GRAB_H
#define GRAB_H

struct Grabber {
	int GRABBER_STATE;
};

void grab_task(void const *argument);

#endif

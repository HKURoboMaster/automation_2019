#ifndef SLIDER_H
#define SLIDER_H

#include "pneumatic.h"

/* VARIABLES: SLIDER - related */
#define SLIDER_toLeft 0
#define SLIDER_toCenterLeft 1
#define SLIDER_toCenter 2
#define SLIDER_toCenterRight 3
#define SLIDER_toRight 4

#define SLIDER_GAP_MILLISEC 100
/* END of VARIABLES: SLIDER - related */

typedef struct Slider {
	Pneumatic cylinderL;
	Pneumatic cylinderR;
	int request;
	int state;
} Slider;

#endif

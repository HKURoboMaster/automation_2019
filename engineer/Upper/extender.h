#ifndef EXTENDER_H
#define EXTENDER_H

#include "pneumatic.h"

/* VARIABLES: EXTENDER - related */
#define EXTENDER_EXTENDED 0
#define EXTENDER_RETRACTED 1
#define EXTENDER_EXTEND_MS 250
#define EXTENDER_RETRACT_MS 250
/* END of VARIABLES: EXTENDER - related */

/* VARIABLES: REQUEST - related */
#define RETRACT 0
#define EXTEND 1
/* END of VARIABLES: REQUEST - related */

typedef struct Extender {
	Pneumatic cylinderX1;
	int request;
	int state;
} Extender;

#endif

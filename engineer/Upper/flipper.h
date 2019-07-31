#ifndef FLIPPER_H
#define FLIPPER_H

#include "pneumatic.h"

/* VARIABLES: FLIPPER - related */
#define FLIPPER_RESTED 0
#define FLIPPER_FLIPPED 1
#define FLIPPER_RESTED_MS 250
#define FLIPPER_FLIPPED_MS 250
/* END of VARIABLES: FLIPPER - related */

/* VARIABLES: REQUEST - related */
#define REST 0
#define FLIP 1
/* END of VARIABLES: REQUEST - related */

typedef struct Flipper {
	Pneumatic cylinderX2;
	int request;
	int state;
} Flipper;

void flipper_task(void const *argument);

#endif

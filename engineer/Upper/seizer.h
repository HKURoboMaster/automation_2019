#ifndef SEIZER_H
#define SEIZER_H

#include "pneumatic.h"

/* VARIABLES: SEIZER - related */
#define SEIZER_SEIZED 0
#define SEIZER_RELEASED 1
#define SEIZER_SEIZE_MS 100
#define SEIZER_RELEASE_MS 100
/* END of VARIABLES: SEIZER - related */

/* VARIABLES: REQUEST - related */
#define RELEASE 0
#define SEIZE 1
/* END of VARIABLES: REQUEST - related */

typedef struct Seizer {
	Pneumatic cylinderX1;
	int request;
	int state;
} Seizer;

void seizer_state_change(int state);
int return_seizer_state(void);
void seizer_task(void const *argument);

#endif

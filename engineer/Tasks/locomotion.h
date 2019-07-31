#ifndef LOCOMOTION_H
#define LOCOMOTION_H

/* VARIABLES: AMMOBOX - related */
#define UNLOCKED 0
#define LOCKED 1
#define UNGATED 2
#define GATED 3
#define PERM_UNLOCKED 4
#define PERM_LOCKED 5
/* END of VARIABLES: AMMOBOX - related */

typedef struct Ammobox {
	int boxlock_state;
	int boxgate_state;
} Ammobox;

void locomotion_task(void const *argument);

#endif

#ifndef UPPER_PNEUMATICS_H
#define UPPER_PNEUMATICS_H

#include "pneumatic.h"

/* VARIABLES: UPPERPNEUM - related */
#define EXTENDED 0
#define RETRACTED 1
/* END of VARIABLES: UPPERPNEUM - related */

typedef struct UpperPneum {
    Pneumatic flipper, extender, seizer;
    int flipper_state, extender_state, seizer_state;
} UpperPneum;

#endif

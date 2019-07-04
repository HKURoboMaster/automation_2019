#include "relay.h"

/* VARIABLES: RELAY - related */
/* END of VARIABLES: RELAY - related */

/* FUNCTIONS: RELAY - related */
void RELAY_high(Relay* relay) {
	HAL_GPIO_WritePin(relay->GPIOx, relay->GPIO_Pin, GPIO_PIN_SET);
}
void RELAY_low(Relay* relay) {
	HAL_GPIO_WritePin(relay->GPIOx, relay->GPIO_Pin, GPIO_PIN_RESET);
}
/* END of FUNCTIONS: RELAY - related */

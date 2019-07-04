#ifndef RELAY_H
#define RELAY_H

#include "stm32f4xx_hal.h"

typedef struct Relay {
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
} Relay;

#endif

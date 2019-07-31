#ifndef PNEUMATIC_H
#define PNEUMATIC_H

#include "gpio.h"

typedef struct Pneumatic {
	GPIO_TypeDef* GPIOx_1;
	uint16_t GPIO_Pin_1;
} Pneumatic;

Pneumatic init_pneum_1head(GPIO_TypeDef* GPIOx_1, uint16_t GPIO_Pin_1);
Pneumatic init_pneum_2head(GPIO_TypeDef* GPIOx_extend, uint16_t GPIO_Pin_extend,
													 GPIO_TypeDef* GPIOx_retract, uint16_t GPIO_Pin_retract);

void pneum_1head_HIGH(Pneumatic* p);
void pneum_1head_LOW(Pneumatic* p);

void pneum_2head_toLeft(Pneumatic* left, Pneumatic* right);
void pneum_2head_toRight(Pneumatic* left, Pneumatic* right);
void pneum_2head_HOLD(Pneumatic* left, Pneumatic* right);

#endif

#ifndef PNEUMATIC_H
#define PNEUMATIC_H

#include "gpio.h"

typedef struct Pneumatic {
	GPIO_TypeDef* GPIOx_1;
	uint16_t GPIO_Pin_1;
	
	GPIO_TypeDef* GPIOx_2;
	uint16_t GPIO_Pin_2;
} Pneumatic;

Pneumatic init_pneum_1head(GPIO_TypeDef* GPIOx_1, uint16_t GPIO_Pin_1);
Pneumatic init_pneum_2head(GPIO_TypeDef* GPIOx_extend, uint16_t GPIO_Pin_extend,
													 GPIO_TypeDef* GPIOx_retract, uint16_t GPIO_Pin_retract);

Pneumatic pneum_1head_EXTEND(Pneumatic* p);
Pneumatic pneum_1head_RETRACT(Pneumatic* p);

Pneumatic pneum_2head_EXTEND(Pneumatic* p);
Pneumatic pneum_2head_RETRACT(Pneumatic* p);
Pneumatic pneum_2head_HOLD(Pneumatic* p);

#endif

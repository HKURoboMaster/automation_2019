#include "pneumatic.h"
#include "gpio.h"

Pneumatic init_pneum_1head(GPIO_TypeDef* GPIOx_1, uint16_t GPIO_Pin_1) {
	Pneumatic p = {GPIOx_1, GPIO_Pin_1};
	return p;
}

Pneumatic init_pneum_2head(GPIO_TypeDef* GPIOx_extend, uint16_t GPIO_Pin_extend,
													 GPIO_TypeDef* GPIOx_retract, uint16_t GPIO_Pin_retract) {
	Pneumatic p = {GPIOx_extend, GPIO_Pin_extend, GPIOx_retract, GPIO_Pin_retract};
	return p;
}

Pneumatic pneum_1head_EXTEND(Pneumatic* p) {
	HAL_GPIO_WritePin(p->GPIOx_1, p->GPIO_Pin_1, GPIO_PIN_SET);
}

Pneumatic pneum_1head_RETRACT(Pneumatic* p) {
	HAL_GPIO_WritePin(p->GPIOx_1, p->GPIO_Pin_1, GPIO_PIN_RESET);
}

Pneumatic pneum_2head_EXTEND(Pneumatic* p) {
	HAL_GPIO_WritePin(p->GPIOx_1, p->GPIO_Pin_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(p->GPIOx_2, p->GPIO_Pin_2, GPIO_PIN_SET);
}

Pneumatic pneum_2head_RETRACT(Pneumatic* p) {
	HAL_GPIO_WritePin(p->GPIOx_1, p->GPIO_Pin_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(p->GPIOx_2, p->GPIO_Pin_2, GPIO_PIN_RESET);
}

Pneumatic pneum_2head_HOLD(Pneumatic* p) {
	HAL_GPIO_WritePin(p->GPIOx_1, p->GPIO_Pin_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(p->GPIOx_2, p->GPIO_Pin_2, GPIO_PIN_RESET);
}

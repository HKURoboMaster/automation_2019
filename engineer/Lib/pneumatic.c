#include "pneumatic.h"
#include "gpio.h"

Pneumatic init_pneum_1head(GPIO_TypeDef* GPIOx_1, uint16_t GPIO_Pin_1) {
	Pneumatic p = {GPIOx_1, GPIO_Pin_1};
	return p;
}

void pneum_1head_HIGH(Pneumatic* p) {
	HAL_GPIO_WritePin(p->GPIOx_1, p->GPIO_Pin_1, GPIO_PIN_SET);
}

void pneum_1head_LOW(Pneumatic* p) {
	HAL_GPIO_WritePin(p->GPIOx_1, p->GPIO_Pin_1, GPIO_PIN_RESET);
}

void pneum_2head_toLeft(Pneumatic* left, Pneumatic* right) {
	HAL_GPIO_WritePin(left->GPIOx_1, left->GPIO_Pin_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(right->GPIOx_1, right->GPIO_Pin_1, GPIO_PIN_SET);
}

void pneum_2head_toRight(Pneumatic* left, Pneumatic* right) {
	HAL_GPIO_WritePin(right->GPIOx_1, right->GPIO_Pin_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(left->GPIOx_1, left->GPIO_Pin_1, GPIO_PIN_RESET);
}

void pneum_2head_HOLD(Pneumatic* left, Pneumatic* right) {
	HAL_GPIO_WritePin(left->GPIOx_1, left->GPIO_Pin_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(right->GPIOx_1, right->GPIO_Pin_1, GPIO_PIN_RESET);
}

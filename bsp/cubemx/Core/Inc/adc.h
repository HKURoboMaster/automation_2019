#ifndef _ADC_H
#define _ADC_H
#ifdef __cplusplus
  extern "C"{
#endif

#include "stm32f4xx_hal.h"
#include "main.h"

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
uint32_t get_ADC_data(ADC_HandleTypeDef *hadc); 

#ifdef __cplusplus
}
#endif
#endif

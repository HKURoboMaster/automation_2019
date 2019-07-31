#ifndef __RESCUE_H_
#define __RESCUE_H_

#include "stm32f4xx_hal.h"
#include "main.h"

#define GPIO_LEFT_MAG  GPIO_PIN_0// Map a number.
#define GPIO_RIGHT_MAG GPIO_PIN_1// Map a number.

//gpio port F and pin 0 and 1 mapped to two I1 and I2 pins

#define GPIO_MAG_GROUP GPIOF

#define TURNON_MAG 1
#define TURNOFF_MAG 0

// For each sides of robot define a magnetic group
typedef struct magnet_group
{
	uint8_t left_status;
	uint8_t right_status;
	uint8_t on_off;
} magnet_group;

void rescue_task(void const* args);
void ctrl_magnet(uint8_t direction);
void gpio_init(void);

#endif
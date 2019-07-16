#include "engg_gpio.h"
#include "gpio.h"

/* FUNCTIONS: ENGINEER_GPIO - related */
void ENGG_GPIO_Init(void) {
		
  GPIO_InitTypeDef GPIO_InitStruct;

  /*Configure GPIO pins : SONICL*/
  GPIO_InitStruct.Pin = SONICL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SONICL_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pins : SONICR*/
	GPIO_InitStruct.Pin = SONICR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SONICR_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pins : CAMERA*/
	HAL_GPIO_WritePin(CAMERA_GPIO_Port, CAMERA_Pin, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = CAMERA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAMERA_GPIO_Port, &GPIO_InitStruct);
}

int read_sonicL(void) {
	return HAL_GPIO_ReadPin(SONICL_GPIO_Port, SONICL_Pin) == 0;
}

int read_sonicR(void) {
	return HAL_GPIO_ReadPin(SONICR_GPIO_Port, SONICR_Pin) == 0;
}

void use_front_cam(void) {
	HAL_GPIO_WritePin(CAMERA_GPIO_Port, CAMERA_Pin, GPIO_PIN_RESET);
}

void use_rear_cam(void) {
	HAL_GPIO_WritePin(CAMERA_GPIO_Port, CAMERA_Pin, GPIO_PIN_SET);
}
/* END of FUNCTIONS: ENGINEER_GPIO - related */

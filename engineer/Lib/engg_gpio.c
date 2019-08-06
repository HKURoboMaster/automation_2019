#include "engg_gpio.h"
#include "gpio.h"
#include "servos.h"
#include "init.h"

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
		
	/*Configure GPIO pins : FLIPL*/
	HAL_GPIO_WritePin(FLIP_GPIO_Port, FLIP_Pin, GPIO_PIN_SET);
	GPIO_InitStruct.Pin = FLIP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(FLIP_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pins : EXTEND*/
	HAL_GPIO_WritePin(EXTEND_GPIO_Port, EXTEND_Pin, GPIO_PIN_SET);
	GPIO_InitStruct.Pin = EXTEND_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(EXTEND_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pins : SLIDEL*/
	HAL_GPIO_WritePin(SLIDEL_GPIO_Port, SLIDEL_Pin, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = SLIDEL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SLIDEL_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pins : SLIDER*/
	HAL_GPIO_WritePin(SLIDER_GPIO_Port, SLIDER_Pin, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = SLIDER_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SLIDER_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pins : SEIZE*/
	HAL_GPIO_WritePin(SEIZE_GPIO_Port, SEIZE_Pin, GPIO_PIN_SET);
	GPIO_InitStruct.Pin = SEIZE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SEIZE_GPIO_Port, &GPIO_InitStruct);
}

int read_sonicL(void) {
	return HAL_GPIO_ReadPin(SONICL_GPIO_Port, SONICL_Pin) == 0;
}

int read_sonicR(void) {
	return HAL_GPIO_ReadPin(SONICR_GPIO_Port, SONICR_Pin) == 0;
}

Servo RMcam = {
	2, 0, 270, 0, 4096, 0, 180, 16
}; 

int camservo_state;

int use_screen() {
	//if (camservo_state == LEFT || SERVO_raiseTo(&RMcam, 106) == SERVO_DONE) {
	//	RMcam.current_index = 0;
	//	return LEFT;
	//}
	return FRONT;
}

int use_RMcam() {
	//if (camservo_state == FRONT || SERVO_raiseTo(&RMcam, 51) == SERVO_DONE) {
	//	RMcam.current_index = 0;
	//	return FRONT;
	//}
	return LEFT;
}

void use_front_cam(void) {
	camservo_state = use_screen();
	HAL_GPIO_WritePin(CAMERA_GPIO_Port, CAMERA_Pin, GPIO_PIN_RESET);
}

void use_rear_cam(void) {
	camservo_state = use_screen();
	HAL_GPIO_WritePin(CAMERA_GPIO_Port, CAMERA_Pin, GPIO_PIN_SET);
}

/* END of FUNCTIONS: ENGINEER_GPIO - related */

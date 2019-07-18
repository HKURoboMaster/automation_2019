#include "arduino.h"
#include "i2c.h"



//Initialize Arduino Sending A Echo Stick
int ard_Init(void)
{
	int echo_stick = 101;
	int echo_stick_buffer;
	if(HAL_I2C_Master_Transmit(&hi2c2,ARDUINO_SLAVE,(uint8_t*) &echo_stick,sizeof(uint8_t),1000)==HAL_OK)
		if(HAL_I2C_Master_Receive(&hi2c2,ARDUINO_SLAVE,(uint8_t*)&echo_stick_buffer,sizeof(uint8_t),1000)==HAL_OK)
			return 1;
		else
			return 0;
	else
		return 0;
}	

int ard_send_msg(Ard_Tx msg)
{
	uint8_t buffer[3];
	buffer[0] = msg.command_flag;
	buffer[1] = msg.rotation_flag[0];
	buffer[2] = msg.rotation_flag[1];
	if(HAL_I2C_Master_Transmit(&hi2c2,ARDUINO_SLAVE<<1,buffer,sizeof(buffer),1000)==HAL_OK)
		return 1;
	else
		return 0;
}

int ard_receive_msg(Ard_Rx *msg)
{
	uint8_t buffer[3];
	if(HAL_I2C_Master_Receive(&hi2c2,ARDUINO_SLAVE<<1,buffer,sizeof(buffer),1000)==HAL_OK)
	{
		msg->current_task   = buffer[0];
		msg->rotation_ok[0] = buffer[1];
		msg->rotation_ok[1] = buffer[2];
		return 1;
	}
	else
		return 0;
}

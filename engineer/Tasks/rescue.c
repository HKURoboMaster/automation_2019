#include "rescue.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "dbus.h"
#include "device.h"
magnet_group back_group = {0,0,0};

void gpio_init()
{ 
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_LEFT_MAG|GPIO_RIGHT_MAG;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_MAG_GROUP, &GPIO_InitStruct);
}

void ctrl_magnet(uint8_t direction)
{
		HAL_GPIO_WritePin(GPIO_MAG_GROUP,GPIO_LEFT_MAG,direction);
}

void rescue_task(void const* args)
{
	uint32_t period = osKernelSysTick();
	rc_device_t prc_dev = NULL;
	rc_info_t prc_info = NULL;
	
	prc_dev = rc_device_find("uart_rc");
  if (prc_dev != NULL)
  {
    prc_info = rc_device_get_info(prc_dev);
  }
	
	uint16_t hold_counter = 0;
	
	while(1)
	{
		if(prc_info->kb.bit.CTRL==1 && prc_info->kb.bit.R == 1)
		{
			ctrl_magnet(TURNON_MAG);
		}
		// Disable mode the magnet will be turned off.
		else if(prc_info->kb.bit.CTRL == 1 ||rc_device_get_state(prc_dev,RC_S2_DOWN)== RM_OK)
		{
			if(hold_counter < 200 && rc_device_get_state(prc_dev,RC_S2_DOWN)!= RM_OK)
			{
				hold_counter++;
			}
			else
			{
				hold_counter = 0;
				ctrl_magnet(TURNOFF_MAG);
			}
		}
		osDelayUntil(&period,4);
	}
}
.PHONY: all clean flash debug
print-%  : ; @echo $* = $($*)
sources := application/init.c application/communicate.c application/chassis_task.c application/gimbal_task.c application/shoot_task.c application/timer_task.c application/infantry_cmd.c application/offline_check.c application/referee_system.c bsp/boards/board.c bsp/boards/drv_can.c bsp/boards/drv_imu.c bsp/boards/drv_flash.c bsp/boards/drv_dr16.c bsp/boards/drv_io.c bsp/boards/drv_uart.c components/object/object.c components/devices/device.c components/devices/motor.c components/devices/dbus.c components/devices/detect.c components/controller/controller.c components/controller/pid_controller.c components/modules/chassis.c components/modules/gimbal.c components/modules/shoot.c components/algorithm/mecanum.c components/algorithm/madgwick_ahrs.c components/algorithm/mahony_ahrs.c components/algorithm/pid.c components/algorithm/ramp.c utilities/period.c utilities/soft_timer.c utilities/ulog/ulog.c utilities/ulog/ulog_console.c test/test.c test/test_module.c test/log_test.c application/protocol/protocol.c application/protocol/protocol_common.c application/protocol/protocol_transmit.c application/protocol/protocol_interface.c components/support/fifo.c components/support/mem_mang4.c components/support/mf_crc.c bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/croutine.c bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/list.c bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/queue.c bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/tasks.c application/param.c bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/timers.c bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c bsp/cubemx/Core/Src/can.c bsp/cubemx/Core/Src/dma.c bsp/cubemx/Core/Src/freertos.c bsp/cubemx/Core/Src/gpio.c bsp/cubemx/Core/Src/main.c bsp/cubemx/Core/Src/spi.c bsp/cubemx/Core/Src/stm32f4xx_hal_msp.c bsp/cubemx/Core/Src/stm32f4xx_hal_timebase_TIM.c bsp/cubemx/Core/Src/stm32f4xx_it.c bsp/cubemx/Core/Src/system_stm32f4xx.c bsp/cubemx/Core/Src/tim.c bsp/cubemx/Core/Src/usart.c bsp/cubemx/USB_DEVICE/App/usbd_cdc_if.c bsp/cubemx/USB_DEVICE/App/usb_device.c bsp/cubemx/USB_DEVICE/Target/usbd_conf.c bsp/cubemx/USB_DEVICE/App/usbd_desc.c bsp/cubemx/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c bsp/cubemx/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c bsp/cubemx/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c bsp/cubemx/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_usart.c bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c 
objects := $(patsubst %.c,%.o,$(sources))
depends := $(patsubst %.o,%.d,$(objects))
sources += bsp/cubemx/Core/Src/startup_stm32f427xx_gcc.s 
objects += bsp/cubemx/Core/Src/startup_stm32f427xx_gcc.o 
ifeq ($(CC),cc)
    CC := arm-none-eabi-gcc
endif
ifeq ($(OBJCOPY),)
    OBJCOPY := arm-none-eabi-objcopy
endif
ifeq ($(OPTIMIZE),size)
    CFLAGS += -Os -g
    LDFLAGS += -Os -g
else ifeq ($(OPTIMIZE),speed)
    CFLAGS += -O2 -g
    LDFLAGS += -O2 -g
else
    CFLAGS += -Og -g
    LDFLAGS += -Og -g
endif
CFLAGS += -march=armv7e-m -mcpu=cortex-m4 -mthumb -std=c11 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -finline-functions -ffunction-sections -fdata-sections
LDFLAGS += -march=armv7e-m -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -lm -lc -lgcc -ffunction-sections -fdata-sections -specs=nosys.specs -Wl,--gc-sections,-Tstm32.ld,-Map,rm_infantry.map,-orm_infantry.elf,--no-wchar-size-warning
CFLAGS += -I bsp/cubemx/Core/Inc -I bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Inc -I bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I bsp/cubemx/Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I bsp/cubemx/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I bsp/cubemx/Drivers/CMSIS/Include -I bsp/cubemx/Drivers/CMSIS/Device/ST/STM32F4xx/Include -I bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/include -I bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I bsp/cubemx/USB_DEVICE/App -I bsp/cubemx/USB_DEVICE/Target -I bsp/boards -I components/algorithm -I components/devices -I components/modules -I components/object -I components/support -I application -I components/controller -I test -I utilities -I utilities/ulog -I application/protocol -I config
CFLAGS += -D USE_HAL_DRIVER -D STM32F427xx -D ARM_MATH_CM4 -D _RTE_ -D __UVISION_VERSION=\"524\" -D __MICROLIB

all: rm_infantry.elf rm_infantry.hex rm_infantry.bin

-include $(depends)

rm_infantry.hex: rm_infantry.elf
	$(OBJCOPY) -O ihex rm_infantry.elf rm_infantry.hex

rm_infantry.bin: rm_infantry.elf
	$(OBJCOPY) -O binary rm_infantry.elf rm_infantry.bin

rm_infantry.elf: $(objects)
	$(CC) $(objects) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -MMD -c $< -o $@

%.o: %.s
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf rm_infantry.elf rm_infantry.map rm_infantry.hex rm_infantry.bin $(objects) $(depends)

flash:
	sudo JLinkExe -device STM32F427II -if SWD -speed 4000 -CommanderScript flash.jlink

debug:
	sudo JLinkGDBServer -select USB -device STM32F427II -endian little -if SWD -speed 4000 -halt -rtos /opt/SEGGER/JLink/GDBServer/RTOSPlugin_FreeRTOS.so

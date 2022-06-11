################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
/home/sugihara/ros/catkin_ws/src/aerial_robot/aerial_robot_nerve/spinal/mcu_project/lib/Hydrus_Lib/CANDevice/motor/can_motor.cpp 

OBJS += \
./Application/Hydrus_Lib/CANDevice/motor/can_motor.o 

CPP_DEPS += \
./Application/Hydrus_Lib/CANDevice/motor/can_motor.d 


# Each subdirectory must supply rules for building sources it contributes
Application/Hydrus_Lib/CANDevice/motor/can_motor.o: /home/sugihara/ros/catkin_ws/src/aerial_robot/aerial_robot_nerve/spinal/mcu_project/lib/Hydrus_Lib/CANDevice/motor/can_motor.cpp Application/Hydrus_Lib/CANDevice/motor/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -DSTMCUBEMX_NEW_VERSION -c -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../ros_lib -I../../../../lib/Jsk_Lib -I../../../../lib/Hydrus_Lib -I../../Middlewares/Third_Party/LwIP/src/include -I../../Middlewares/Third_Party/LwIP/system -I../../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../../Middlewares/Third_Party/LwIP/src/include/lwip -I../../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../../Middlewares/Third_Party/LwIP/src/include/netif -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../../Middlewares/Third_Party/LwIP/system/arch -I../../Drivers/BSP/Components/lan8742 -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -DUSE_HAL_DRIVER -DSTM32H743xx -DDEBUG -DSTMCUBEMX_NEW_VERSION -I../../Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-Hydrus_Lib-2f-CANDevice-2f-motor

clean-Application-2f-Hydrus_Lib-2f-CANDevice-2f-motor:
	-$(RM) ./Application/Hydrus_Lib/CANDevice/motor/can_motor.d ./Application/Hydrus_Lib/CANDevice/motor/can_motor.o

.PHONY: clean-Application-2f-Hydrus_Lib-2f-CANDevice-2f-motor


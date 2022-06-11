################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
/home/sugihara/ros/catkin_ws/src/aerial_robot/aerial_robot_nerve/spinal/mcu_project/lib/Hydrus_Lib/Spine/spine.cpp 

OBJS += \
./Application/Hydrus_Lib/Spine/spine.o 

CPP_DEPS += \
./Application/Hydrus_Lib/Spine/spine.d 


# Each subdirectory must supply rules for building sources it contributes
Application/Hydrus_Lib/Spine/spine.o: /home/sugihara/ros/catkin_ws/src/aerial_robot/aerial_robot_nerve/spinal/mcu_project/lib/Hydrus_Lib/Spine/spine.cpp Application/Hydrus_Lib/Spine/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32F746xx -DDEBUG -c -I../../Inc -I../../Drivers/STM32F7xx_HAL_Driver/Inc -I../../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../../Drivers/CMSIS/Include -I../../ros_lib -I../../../../lib/Jsk_Lib -I../../../../lib/Hydrus_Lib -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-Hydrus_Lib-2f-Spine

clean-Application-2f-Hydrus_Lib-2f-Spine:
	-$(RM) ./Application/Hydrus_Lib/Spine/spine.d ./Application/Hydrus_Lib/Spine/spine.o

.PHONY: clean-Application-2f-Hydrus_Lib-2f-Spine


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/lwgps/src/lwgps/lwgps.c 

OBJS += \
./Core/Inc/lwgps/src/lwgps/lwgps.o 

C_DEPS += \
./Core/Inc/lwgps/src/lwgps/lwgps.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/lwgps/src/lwgps/%.o Core/Inc/lwgps/src/lwgps/%.su Core/Inc/lwgps/src/lwgps/%.cyclo: ../Core/Inc/lwgps/src/lwgps/%.c Core/Inc/lwgps/src/lwgps/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/Users/ILKER KESER/STM32CubeIDE/FreeRtos_workspace/008_FreeRTOS_Project_RocketController/Core/Inc/lwgps/src/include/lwgps" -I"C:/Users/ILKER KESER/STM32CubeIDE/FreeRtos_workspace/008_FreeRTOS_Project_RocketController/Third_Party/FreeRTOS/include" -I"C:/Users/ILKER KESER/STM32CubeIDE/FreeRtos_workspace/008_FreeRTOS_Project_RocketController/Third_Party/FreeRTOS/portable/GCC/ARM_CM4F" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-lwgps-2f-src-2f-lwgps

clean-Core-2f-Inc-2f-lwgps-2f-src-2f-lwgps:
	-$(RM) ./Core/Inc/lwgps/src/lwgps/lwgps.cyclo ./Core/Inc/lwgps/src/lwgps/lwgps.d ./Core/Inc/lwgps/src/lwgps/lwgps.o ./Core/Inc/lwgps/src/lwgps/lwgps.su

.PHONY: clean-Core-2f-Inc-2f-lwgps-2f-src-2f-lwgps


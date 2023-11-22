################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display/Src/Display_API.c 

OBJS += \
./Display/Src/Display_API.o 

C_DEPS += \
./Display/Src/Display_API.d 


# Each subdirectory must supply rules for building sources it contributes
Display/Src/%.o Display/Src/%.su: ../Display/Src/%.c Display/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Display-2f-Src

clean-Display-2f-Src:
	-$(RM) ./Display/Src/Display_API.d ./Display/Src/Display_API.o ./Display/Src/Display_API.su

.PHONY: clean-Display-2f-Src


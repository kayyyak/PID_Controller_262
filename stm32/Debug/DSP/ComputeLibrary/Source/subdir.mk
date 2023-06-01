################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DSP/ComputeLibrary/Source/arm_cl_tables.c 

OBJS += \
./DSP/ComputeLibrary/Source/arm_cl_tables.o 

C_DEPS += \
./DSP/ComputeLibrary/Source/arm_cl_tables.d 


# Each subdirectory must supply rules for building sources it contributes
DSP/ComputeLibrary/Source/%.o DSP/ComputeLibrary/Source/%.su: ../DSP/ComputeLibrary/Source/%.c DSP/ComputeLibrary/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/fibo/sophomore/semester 2/fra262/PID/PID_Controller_262/stm32/DSP/ComputeLibrary/Include" -I"D:/fibo/sophomore/semester 2/fra262/PID/PID_Controller_262/stm32/DSP/Include" -I"D:/fibo/sophomore/semester 2/fra262/PID/PID_Controller_262/stm32/DSP/PrivateInclude" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-DSP-2f-ComputeLibrary-2f-Source

clean-DSP-2f-ComputeLibrary-2f-Source:
	-$(RM) ./DSP/ComputeLibrary/Source/arm_cl_tables.d ./DSP/ComputeLibrary/Source/arm_cl_tables.o ./DSP/ComputeLibrary/Source/arm_cl_tables.su

.PHONY: clean-DSP-2f-ComputeLibrary-2f-Source


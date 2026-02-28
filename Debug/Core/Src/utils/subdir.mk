################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/utils/math3d.c \
../Core/Src/utils/ringbuf.c \
../Core/Src/utils/timebase.c 

OBJS += \
./Core/Src/utils/math3d.o \
./Core/Src/utils/ringbuf.o \
./Core/Src/utils/timebase.o 

C_DEPS += \
./Core/Src/utils/math3d.d \
./Core/Src/utils/ringbuf.d \
./Core/Src/utils/timebase.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/utils/%.o Core/Src/utils/%.su Core/Src/utils/%.cyclo: ../Core/Src/utils/%.c Core/Src/utils/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-utils

clean-Core-2f-Src-2f-utils:
	-$(RM) ./Core/Src/utils/math3d.cyclo ./Core/Src/utils/math3d.d ./Core/Src/utils/math3d.o ./Core/Src/utils/math3d.su ./Core/Src/utils/ringbuf.cyclo ./Core/Src/utils/ringbuf.d ./Core/Src/utils/ringbuf.o ./Core/Src/utils/ringbuf.su ./Core/Src/utils/timebase.cyclo ./Core/Src/utils/timebase.d ./Core/Src/utils/timebase.o ./Core/Src/utils/timebase.su

.PHONY: clean-Core-2f-Src-2f-utils


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/drivers/mpu6050.c \
../Core/Src/drivers/uart_logger.c 

OBJS += \
./Core/Src/drivers/mpu6050.o \
./Core/Src/drivers/uart_logger.o 

C_DEPS += \
./Core/Src/drivers/mpu6050.d \
./Core/Src/drivers/uart_logger.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/drivers/%.o Core/Src/drivers/%.su Core/Src/drivers/%.cyclo: ../Core/Src/drivers/%.c Core/Src/drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-drivers

clean-Core-2f-Src-2f-drivers:
	-$(RM) ./Core/Src/drivers/mpu6050.cyclo ./Core/Src/drivers/mpu6050.d ./Core/Src/drivers/mpu6050.o ./Core/Src/drivers/mpu6050.su ./Core/Src/drivers/uart_logger.cyclo ./Core/Src/drivers/uart_logger.d ./Core/Src/drivers/uart_logger.o ./Core/Src/drivers/uart_logger.su

.PHONY: clean-Core-2f-Src-2f-drivers


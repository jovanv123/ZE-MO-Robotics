################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Libraries/Peripherals/encoders.c \
../Core/Libraries/Peripherals/motors.c 

OBJS += \
./Core/Libraries/Peripherals/encoders.o \
./Core/Libraries/Peripherals/motors.o 

C_DEPS += \
./Core/Libraries/Peripherals/encoders.d \
./Core/Libraries/Peripherals/motors.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Libraries/Peripherals/%.o Core/Libraries/Peripherals/%.su Core/Libraries/Peripherals/%.cyclo: ../Core/Libraries/Peripherals/%.c Core/Libraries/Peripherals/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Libraries-2f-Peripherals

clean-Core-2f-Libraries-2f-Peripherals:
	-$(RM) ./Core/Libraries/Peripherals/encoders.cyclo ./Core/Libraries/Peripherals/encoders.d ./Core/Libraries/Peripherals/encoders.o ./Core/Libraries/Peripherals/encoders.su ./Core/Libraries/Peripherals/motors.cyclo ./Core/Libraries/Peripherals/motors.d ./Core/Libraries/Peripherals/motors.o ./Core/Libraries/Peripherals/motors.su

.PHONY: clean-Core-2f-Libraries-2f-Peripherals


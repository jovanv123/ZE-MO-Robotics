################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Libraries/Logic/PID.c \
../Core/Libraries/Logic/SpeedProfile.c \
../Core/Libraries/Logic/odometry.c \
../Core/Libraries/Logic/sequences.c 

OBJS += \
./Core/Libraries/Logic/PID.o \
./Core/Libraries/Logic/SpeedProfile.o \
./Core/Libraries/Logic/odometry.o \
./Core/Libraries/Logic/sequences.o 

C_DEPS += \
./Core/Libraries/Logic/PID.d \
./Core/Libraries/Logic/SpeedProfile.d \
./Core/Libraries/Logic/odometry.d \
./Core/Libraries/Logic/sequences.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Libraries/Logic/%.o Core/Libraries/Logic/%.su Core/Libraries/Logic/%.cyclo: ../Core/Libraries/Logic/%.c Core/Libraries/Logic/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Libraries-2f-Logic

clean-Core-2f-Libraries-2f-Logic:
	-$(RM) ./Core/Libraries/Logic/PID.cyclo ./Core/Libraries/Logic/PID.d ./Core/Libraries/Logic/PID.o ./Core/Libraries/Logic/PID.su ./Core/Libraries/Logic/SpeedProfile.cyclo ./Core/Libraries/Logic/SpeedProfile.d ./Core/Libraries/Logic/SpeedProfile.o ./Core/Libraries/Logic/SpeedProfile.su ./Core/Libraries/Logic/odometry.cyclo ./Core/Libraries/Logic/odometry.d ./Core/Libraries/Logic/odometry.o ./Core/Libraries/Logic/odometry.su ./Core/Libraries/Logic/sequences.cyclo ./Core/Libraries/Logic/sequences.d ./Core/Libraries/Logic/sequences.o ./Core/Libraries/Logic/sequences.su

.PHONY: clean-Core-2f-Libraries-2f-Logic


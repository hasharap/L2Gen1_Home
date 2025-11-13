################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/CSMS.c \
../Core/Inc/OTA.c \
../Core/Inc/STM32F103_EEPROM.c \
../Core/Inc/control_pilot.c \
../Core/Inc/data_structures.c \
../Core/Inc/gpio_feedback.c \
../Core/Inc/power_measure.c \
../Core/Inc/protection.c \
../Core/Inc/rtc.c \
../Core/Inc/state_machine.c \
../Core/Inc/temp_NTC.c \
../Core/Inc/timer.c 

OBJS += \
./Core/Inc/CSMS.o \
./Core/Inc/OTA.o \
./Core/Inc/STM32F103_EEPROM.o \
./Core/Inc/control_pilot.o \
./Core/Inc/data_structures.o \
./Core/Inc/gpio_feedback.o \
./Core/Inc/power_measure.o \
./Core/Inc/protection.o \
./Core/Inc/rtc.o \
./Core/Inc/state_machine.o \
./Core/Inc/temp_NTC.o \
./Core/Inc/timer.o 

C_DEPS += \
./Core/Inc/CSMS.d \
./Core/Inc/OTA.d \
./Core/Inc/STM32F103_EEPROM.d \
./Core/Inc/control_pilot.d \
./Core/Inc/data_structures.d \
./Core/Inc/gpio_feedback.d \
./Core/Inc/power_measure.d \
./Core/Inc/protection.d \
./Core/Inc/rtc.d \
./Core/Inc/state_machine.d \
./Core/Inc/temp_NTC.d \
./Core/Inc/timer.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/%.o Core/Inc/%.su Core/Inc/%.cyclo: ../Core/Inc/%.c Core/Inc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc

clean-Core-2f-Inc:
	-$(RM) ./Core/Inc/CSMS.cyclo ./Core/Inc/CSMS.d ./Core/Inc/CSMS.o ./Core/Inc/CSMS.su ./Core/Inc/OTA.cyclo ./Core/Inc/OTA.d ./Core/Inc/OTA.o ./Core/Inc/OTA.su ./Core/Inc/STM32F103_EEPROM.cyclo ./Core/Inc/STM32F103_EEPROM.d ./Core/Inc/STM32F103_EEPROM.o ./Core/Inc/STM32F103_EEPROM.su ./Core/Inc/control_pilot.cyclo ./Core/Inc/control_pilot.d ./Core/Inc/control_pilot.o ./Core/Inc/control_pilot.su ./Core/Inc/data_structures.cyclo ./Core/Inc/data_structures.d ./Core/Inc/data_structures.o ./Core/Inc/data_structures.su ./Core/Inc/gpio_feedback.cyclo ./Core/Inc/gpio_feedback.d ./Core/Inc/gpio_feedback.o ./Core/Inc/gpio_feedback.su ./Core/Inc/power_measure.cyclo ./Core/Inc/power_measure.d ./Core/Inc/power_measure.o ./Core/Inc/power_measure.su ./Core/Inc/protection.cyclo ./Core/Inc/protection.d ./Core/Inc/protection.o ./Core/Inc/protection.su ./Core/Inc/rtc.cyclo ./Core/Inc/rtc.d ./Core/Inc/rtc.o ./Core/Inc/rtc.su ./Core/Inc/state_machine.cyclo ./Core/Inc/state_machine.d ./Core/Inc/state_machine.o ./Core/Inc/state_machine.su ./Core/Inc/temp_NTC.cyclo ./Core/Inc/temp_NTC.d ./Core/Inc/temp_NTC.o ./Core/Inc/temp_NTC.su ./Core/Inc/timer.cyclo ./Core/Inc/timer.d ./Core/Inc/timer.o ./Core/Inc/timer.su

.PHONY: clean-Core-2f-Inc


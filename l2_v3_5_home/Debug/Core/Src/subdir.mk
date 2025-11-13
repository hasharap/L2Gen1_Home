################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/CSMS.c \
../Core/Src/OTA.c \
../Core/Src/STM32F103_EEPROM.c \
../Core/Src/adc.c \
../Core/Src/charge_config.c \
../Core/Src/control_pilot.c \
../Core/Src/data_structures.c \
../Core/Src/dma.c \
../Core/Src/gpio.c \
../Core/Src/gpio_feedback.c \
../Core/Src/iwdg.c \
../Core/Src/main.c \
../Core/Src/power_measure.c \
../Core/Src/protection.c \
../Core/Src/rtc.c \
../Core/Src/state_machine.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c \
../Core/Src/temp_NTC.c \
../Core/Src/tim.c \
../Core/Src/timer.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/CSMS.o \
./Core/Src/OTA.o \
./Core/Src/STM32F103_EEPROM.o \
./Core/Src/adc.o \
./Core/Src/charge_config.o \
./Core/Src/control_pilot.o \
./Core/Src/data_structures.o \
./Core/Src/dma.o \
./Core/Src/gpio.o \
./Core/Src/gpio_feedback.o \
./Core/Src/iwdg.o \
./Core/Src/main.o \
./Core/Src/power_measure.o \
./Core/Src/protection.o \
./Core/Src/rtc.o \
./Core/Src/state_machine.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o \
./Core/Src/temp_NTC.o \
./Core/Src/tim.o \
./Core/Src/timer.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/CSMS.d \
./Core/Src/OTA.d \
./Core/Src/STM32F103_EEPROM.d \
./Core/Src/adc.d \
./Core/Src/charge_config.d \
./Core/Src/control_pilot.d \
./Core/Src/data_structures.d \
./Core/Src/dma.d \
./Core/Src/gpio.d \
./Core/Src/gpio_feedback.d \
./Core/Src/iwdg.d \
./Core/Src/main.d \
./Core/Src/power_measure.d \
./Core/Src/protection.d \
./Core/Src/rtc.d \
./Core/Src/state_machine.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d \
./Core/Src/temp_NTC.d \
./Core/Src/tim.d \
./Core/Src/timer.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/CSMS.cyclo ./Core/Src/CSMS.d ./Core/Src/CSMS.o ./Core/Src/CSMS.su ./Core/Src/OTA.cyclo ./Core/Src/OTA.d ./Core/Src/OTA.o ./Core/Src/OTA.su ./Core/Src/STM32F103_EEPROM.cyclo ./Core/Src/STM32F103_EEPROM.d ./Core/Src/STM32F103_EEPROM.o ./Core/Src/STM32F103_EEPROM.su ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/charge_config.cyclo ./Core/Src/charge_config.d ./Core/Src/charge_config.o ./Core/Src/charge_config.su ./Core/Src/control_pilot.cyclo ./Core/Src/control_pilot.d ./Core/Src/control_pilot.o ./Core/Src/control_pilot.su ./Core/Src/data_structures.cyclo ./Core/Src/data_structures.d ./Core/Src/data_structures.o ./Core/Src/data_structures.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/gpio_feedback.cyclo ./Core/Src/gpio_feedback.d ./Core/Src/gpio_feedback.o ./Core/Src/gpio_feedback.su ./Core/Src/iwdg.cyclo ./Core/Src/iwdg.d ./Core/Src/iwdg.o ./Core/Src/iwdg.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/power_measure.cyclo ./Core/Src/power_measure.d ./Core/Src/power_measure.o ./Core/Src/power_measure.su ./Core/Src/protection.cyclo ./Core/Src/protection.d ./Core/Src/protection.o ./Core/Src/protection.su ./Core/Src/rtc.cyclo ./Core/Src/rtc.d ./Core/Src/rtc.o ./Core/Src/rtc.su ./Core/Src/state_machine.cyclo ./Core/Src/state_machine.d ./Core/Src/state_machine.o ./Core/Src/state_machine.su ./Core/Src/stm32f1xx_hal_msp.cyclo ./Core/Src/stm32f1xx_hal_msp.d ./Core/Src/stm32f1xx_hal_msp.o ./Core/Src/stm32f1xx_hal_msp.su ./Core/Src/stm32f1xx_it.cyclo ./Core/Src/stm32f1xx_it.d ./Core/Src/stm32f1xx_it.o ./Core/Src/stm32f1xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f1xx.cyclo ./Core/Src/system_stm32f1xx.d ./Core/Src/system_stm32f1xx.o ./Core/Src/system_stm32f1xx.su ./Core/Src/temp_NTC.cyclo ./Core/Src/temp_NTC.d ./Core/Src/temp_NTC.o ./Core/Src/temp_NTC.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/timer.cyclo ./Core/Src/timer.d ./Core/Src/timer.o ./Core/Src/timer.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src


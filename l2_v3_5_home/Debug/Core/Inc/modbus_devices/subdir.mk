################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/modbus_devices/eMeter.c \
../Core/Inc/modbus_devices/imd.c \
../Core/Inc/modbus_devices/modbusRTU.c 

OBJS += \
./Core/Inc/modbus_devices/eMeter.o \
./Core/Inc/modbus_devices/imd.o \
./Core/Inc/modbus_devices/modbusRTU.o 

C_DEPS += \
./Core/Inc/modbus_devices/eMeter.d \
./Core/Inc/modbus_devices/imd.d \
./Core/Inc/modbus_devices/modbusRTU.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/modbus_devices/%.o Core/Inc/modbus_devices/%.su Core/Inc/modbus_devices/%.cyclo: ../Core/Inc/modbus_devices/%.c Core/Inc/modbus_devices/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-modbus_devices

clean-Core-2f-Inc-2f-modbus_devices:
	-$(RM) ./Core/Inc/modbus_devices/eMeter.cyclo ./Core/Inc/modbus_devices/eMeter.d ./Core/Inc/modbus_devices/eMeter.o ./Core/Inc/modbus_devices/eMeter.su ./Core/Inc/modbus_devices/imd.cyclo ./Core/Inc/modbus_devices/imd.d ./Core/Inc/modbus_devices/imd.o ./Core/Inc/modbus_devices/imd.su ./Core/Inc/modbus_devices/modbusRTU.cyclo ./Core/Inc/modbus_devices/modbusRTU.d ./Core/Inc/modbus_devices/modbusRTU.o ./Core/Inc/modbus_devices/modbusRTU.su

.PHONY: clean-Core-2f-Inc-2f-modbus_devices


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/ap/ap.c \
C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/ap/can_manager.c \
C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/ap/comm_handler.c 

C_DEPS += \
./src/ap/ap.d \
./src/ap/can_manager.d \
./src/ap/comm_handler.d 

OBJS += \
./src/ap/ap.o \
./src/ap/can_manager.o \
./src/ap/comm_handler.o 


# Each subdirectory must supply rules for building sources it contributes
src/ap/ap.o: C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/ap/ap.c src/ap/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -c -I../Inc -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/ap" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/bsp" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/bsp/device" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/common" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/common/core" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/common/hw/include" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/hw" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/hw/driver/usb" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib/CMSIS/Include" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib/CMSIS/Device/ST/STM32G4xx/Include" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib/STM32G4xx_HAL_Driver/Inc" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/hw/driver/usb/usb_cdc" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/hw/driver/eeprom" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/ap/thread" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
src/ap/can_manager.o: C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/ap/can_manager.c src/ap/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -c -I../Inc -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/ap" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/bsp" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/bsp/device" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/common" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/common/core" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/common/hw/include" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/hw" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/hw/driver/usb" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib/CMSIS/Include" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib/CMSIS/Device/ST/STM32G4xx/Include" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib/STM32G4xx_HAL_Driver/Inc" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/hw/driver/usb/usb_cdc" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/hw/driver/eeprom" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/ap/thread" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
src/ap/comm_handler.o: C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/ap/comm_handler.c src/ap/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -c -I../Inc -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/ap" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/bsp" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/bsp/device" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/common" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/common/core" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/common/hw/include" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/hw" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/hw/driver/usb" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib/CMSIS/Include" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib/CMSIS/Device/ST/STM32G4xx/Include" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib/STM32G4xx_HAL_Driver/Inc" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/hw/driver/usb/usb_cdc" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/hw/driver/eeprom" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/ap/thread" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-src-2f-ap

clean-src-2f-ap:
	-$(RM) ./src/ap/ap.cyclo ./src/ap/ap.d ./src/ap/ap.o ./src/ap/ap.su ./src/ap/can_manager.cyclo ./src/ap/can_manager.d ./src/ap/can_manager.o ./src/ap/can_manager.su ./src/ap/comm_handler.cyclo ./src/ap/comm_handler.d ./src/ap/comm_handler.o ./src/ap/comm_handler.su

.PHONY: clean-src-2f-ap


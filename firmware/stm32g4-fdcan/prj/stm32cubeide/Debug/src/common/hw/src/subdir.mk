################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/common/hw/src/cli.c \
C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/common/hw/src/mini-printf.c 

C_DEPS += \
./src/common/hw/src/cli.d \
./src/common/hw/src/mini-printf.d 

OBJS += \
./src/common/hw/src/cli.o \
./src/common/hw/src/mini-printf.o 


# Each subdirectory must supply rules for building sources it contributes
src/common/hw/src/cli.o: C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/common/hw/src/cli.c src/common/hw/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -c -I../Inc -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/ap" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/bsp" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/bsp/device" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/common" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/common/core" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/common/hw/include" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/hw" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/hw/driver/usb" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib/CMSIS/Include" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib/CMSIS/Device/ST/STM32G4xx/Include" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib/STM32G4xx_HAL_Driver/Inc" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/hw/driver/usb/usb_cdc" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/hw/driver/eeprom" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/ap/thread" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
src/common/hw/src/mini-printf.o: C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/common/hw/src/mini-printf.c src/common/hw/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -c -I../Inc -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/ap" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/bsp" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/bsp/device" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/common" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/common/core" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/common/hw/include" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/hw" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/hw/driver/usb" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib/CMSIS/Include" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib/CMSIS/Device/ST/STM32G4xx/Include" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib/STM32G4xx_HAL_Driver/Inc" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/hw/driver/usb/usb_cdc" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/hw/driver/eeprom" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/ap/thread" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-src-2f-common-2f-hw-2f-src

clean-src-2f-common-2f-hw-2f-src:
	-$(RM) ./src/common/hw/src/cli.cyclo ./src/common/hw/src/cli.d ./src/common/hw/src/cli.o ./src/common/hw/src/cli.su ./src/common/hw/src/mini-printf.cyclo ./src/common/hw/src/mini-printf.d ./src/common/hw/src/mini-printf.o ./src/common/hw/src/mini-printf.su

.PHONY: clean-src-2f-common-2f-hw-2f-src


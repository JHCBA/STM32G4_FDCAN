################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/bsp/bsp.c 

C_DEPS += \
./src/bsp/bsp.d 

OBJS += \
./src/bsp/bsp.o 


# Each subdirectory must supply rules for building sources it contributes
src/bsp/bsp.o: C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/bsp/bsp.c src/bsp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -c -I../Inc -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/ap" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/bsp" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/bsp/device" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/common" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/common/core" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/common/hw/include" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/hw" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/hw/driver/usb" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib/CMSIS/Include" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib/CMSIS/Device/ST/STM32G4xx/Include" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib/STM32G4xx_HAL_Driver/Inc" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/hw/driver/usb/usb_cdc" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/hw/driver/eeprom" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/ap/thread" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-src-2f-bsp

clean-src-2f-bsp:
	-$(RM) ./src/bsp/bsp.cyclo ./src/bsp/bsp.d ./src/bsp/bsp.o ./src/bsp/bsp.su

.PHONY: clean-src-2f-bsp


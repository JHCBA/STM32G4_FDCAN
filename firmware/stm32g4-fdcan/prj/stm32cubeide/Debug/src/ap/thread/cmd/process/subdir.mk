################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/ap/thread/cmd/process/cmd_boot.c 

C_DEPS += \
./src/ap/thread/cmd/process/cmd_boot.d 

OBJS += \
./src/ap/thread/cmd/process/cmd_boot.o 


# Each subdirectory must supply rules for building sources it contributes
src/ap/thread/cmd/process/cmd_boot.o: C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/ap/thread/cmd/process/cmd_boot.c src/ap/thread/cmd/process/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -c -I../Inc -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/ap" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/bsp" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/bsp/device" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/common" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/common/core" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/common/hw/include" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/hw" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/hw/driver/usb" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib/CMSIS/Include" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib/CMSIS/Device/ST/STM32G4xx/Include" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/lib/STM32G4xx_HAL_Driver/Inc" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/hw/driver/usb/usb_cdc" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/hw/driver/eeprom" -I"C:/Users/JH/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-fdcan/src/ap/thread" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-src-2f-ap-2f-thread-2f-cmd-2f-process

clean-src-2f-ap-2f-thread-2f-cmd-2f-process:
	-$(RM) ./src/ap/thread/cmd/process/cmd_boot.cyclo ./src/ap/thread/cmd/process/cmd_boot.d ./src/ap/thread/cmd/process/cmd_boot.o ./src/ap/thread/cmd/process/cmd_boot.su

.PHONY: clean-src-2f-ap-2f-thread-2f-cmd-2f-process


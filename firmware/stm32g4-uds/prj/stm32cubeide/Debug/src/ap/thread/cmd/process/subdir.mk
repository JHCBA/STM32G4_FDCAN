################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/ap/thread/cmd/process/cmd_boot.c 

C_DEPS += \
./src/ap/thread/cmd/process/cmd_boot.d 

OBJS += \
./src/ap/thread/cmd/process/cmd_boot.o 


# Each subdirectory must supply rules for building sources it contributes
src/ap/thread/cmd/process/cmd_boot.o: C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/ap/thread/cmd/process/cmd_boot.c src/ap/thread/cmd/process/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -c -I../Inc -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/ap" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/bsp" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/bsp/device" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common/core" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common/hw/include" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/hw" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/hw/driver/usb" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib/CMSIS/Include" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib/CMSIS/Device/ST/STM32G4xx/Include" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib/STM32G4xx_HAL_Driver/Inc" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/hw/driver/usb/usb_cdc" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/hw/driver/eeprom" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/ap/thread" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-src-2f-ap-2f-thread-2f-cmd-2f-process

clean-src-2f-ap-2f-thread-2f-cmd-2f-process:
	-$(RM) ./src/ap/thread/cmd/process/cmd_boot.cyclo ./src/ap/thread/cmd/process/cmd_boot.d ./src/ap/thread/cmd/process/cmd_boot.o ./src/ap/thread/cmd/process/cmd_boot.su

.PHONY: clean-src-2f-ap-2f-thread-2f-cmd-2f-process


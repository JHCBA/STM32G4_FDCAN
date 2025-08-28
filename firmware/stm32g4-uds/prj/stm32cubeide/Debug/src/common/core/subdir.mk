################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common/core/qbuffer.c \
C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common/core/util.c 

C_DEPS += \
./src/common/core/qbuffer.d \
./src/common/core/util.d 

OBJS += \
./src/common/core/qbuffer.o \
./src/common/core/util.o 


# Each subdirectory must supply rules for building sources it contributes
src/common/core/qbuffer.o: C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common/core/qbuffer.c src/common/core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -c -I../Inc -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/ap" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/bsp" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/bsp/device" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common/core" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common/hw/include" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/hw" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/hw/driver/usb" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib/CMSIS/Include" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib/CMSIS/Device/ST/STM32G4xx/Include" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib/STM32G4xx_HAL_Driver/Inc" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/hw/driver/usb/usb_cdc" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/hw/driver/eeprom" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/ap/thread" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
src/common/core/util.o: C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common/core/util.c src/common/core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -c -I../Inc -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/ap" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/bsp" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/bsp/device" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common/core" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common/hw/include" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/hw" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/hw/driver/usb" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib/CMSIS/Include" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib/CMSIS/Device/ST/STM32G4xx/Include" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib/STM32G4xx_HAL_Driver/Inc" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/hw/driver/usb/usb_cdc" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/hw/driver/eeprom" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/ap/thread" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-src-2f-common-2f-core

clean-src-2f-common-2f-core:
	-$(RM) ./src/common/core/qbuffer.cyclo ./src/common/core/qbuffer.d ./src/common/core/qbuffer.o ./src/common/core/qbuffer.su ./src/common/core/util.cyclo ./src/common/core/util.d ./src/common/core/util.o ./src/common/core/util.su

.PHONY: clean-src-2f-common-2f-core


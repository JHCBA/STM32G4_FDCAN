################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common/hw/src/can_db.c \
C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common/hw/src/cli.c \
C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common/hw/src/mini-printf.c 

C_DEPS += \
./src/common/hw/src/can_db.d \
./src/common/hw/src/cli.d \
./src/common/hw/src/mini-printf.d 

OBJS += \
./src/common/hw/src/can_db.o \
./src/common/hw/src/cli.o \
./src/common/hw/src/mini-printf.o 


# Each subdirectory must supply rules for building sources it contributes
src/common/hw/src/can_db.o: C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common/hw/src/can_db.c src/common/hw/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -c -I../Inc -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/ap" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/bsp" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/bsp/device" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common/core" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common/hw/include" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/hw" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/hw/driver/usb" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib/CMSIS/Include" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib/CMSIS/Device/ST/STM32G4xx/Include" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib/STM32G4xx_HAL_Driver/Inc" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/hw/driver/usb/usb_cdc" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/hw/driver/eeprom" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/ap/thread" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
src/common/hw/src/cli.o: C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common/hw/src/cli.c src/common/hw/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -c -I../Inc -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/ap" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/bsp" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/bsp/device" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common/core" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common/hw/include" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/hw" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/hw/driver/usb" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib/CMSIS/Include" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib/CMSIS/Device/ST/STM32G4xx/Include" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib/STM32G4xx_HAL_Driver/Inc" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/hw/driver/usb/usb_cdc" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/hw/driver/eeprom" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/ap/thread" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
src/common/hw/src/mini-printf.o: C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common/hw/src/mini-printf.c src/common/hw/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -c -I../Inc -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/ap" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/bsp" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/bsp/device" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common/core" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common/hw/include" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/hw" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/hw/driver/usb" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib/CMSIS/Include" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib/CMSIS/Device/ST/STM32G4xx/Include" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib/STM32G4xx_HAL_Driver/Inc" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/hw/driver/usb/usb_cdc" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/hw/driver/eeprom" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/ap/thread" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-src-2f-common-2f-hw-2f-src

clean-src-2f-common-2f-hw-2f-src:
	-$(RM) ./src/common/hw/src/can_db.cyclo ./src/common/hw/src/can_db.d ./src/common/hw/src/can_db.o ./src/common/hw/src/can_db.su ./src/common/hw/src/cli.cyclo ./src/common/hw/src/cli.d ./src/common/hw/src/cli.o ./src/common/hw/src/cli.su ./src/common/hw/src/mini-printf.cyclo ./src/common/hw/src/mini-printf.d ./src/common/hw/src/mini-printf.o ./src/common/hw/src/mini-printf.su

.PHONY: clean-src-2f-common-2f-hw-2f-src


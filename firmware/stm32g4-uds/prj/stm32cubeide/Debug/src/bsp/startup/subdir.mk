################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/bsp/startup/startup_stm32g431xx.s 

S_DEPS += \
./src/bsp/startup/startup_stm32g431xx.d 

OBJS += \
./src/bsp/startup/startup_stm32g431xx.o 


# Each subdirectory must supply rules for building sources it contributes
src/bsp/startup/startup_stm32g431xx.o: C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/bsp/startup/startup_stm32g431xx.s src/bsp/startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/ap" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/bsp" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/bsp/device" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common/core" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/common/hw/include" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/hw" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/hw/driver/usb" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib/CMSIS/Include" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib/CMSIS/Device/ST/STM32G4xx/Include" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/lib/STM32G4xx_HAL_Driver/Inc" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/hw/driver/usb/usb_cdc" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/hw/driver/eeprom" -I"C:/Users/jhcho/Desktop/GITHUB/STM32G4_FDCAN/firmware/stm32g4-uds/src/ap/thread" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-src-2f-bsp-2f-startup

clean-src-2f-bsp-2f-startup:
	-$(RM) ./src/bsp/startup/startup_stm32g431xx.d ./src/bsp/startup/startup_stm32g431xx.o

.PHONY: clean-src-2f-bsp-2f-startup


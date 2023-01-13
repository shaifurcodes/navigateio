################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/decawaveSrc/decadriver/deca_device.c \
../Src/decawaveSrc/decadriver/deca_params_init.c 

OBJS += \
./Src/decawaveSrc/decadriver/deca_device.o \
./Src/decawaveSrc/decadriver/deca_params_init.o 

C_DEPS += \
./Src/decawaveSrc/decadriver/deca_device.d \
./Src/decawaveSrc/decadriver/deca_params_init.d 


# Each subdirectory must supply rules for building sources it contributes
Src/decawaveSrc/decadriver/deca_device.o: ../Src/decawaveSrc/decadriver/deca_device.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32F105xC -DUSE_HAL_DRIVER -c -I../Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/decawaveSrc/decadriver/deca_device.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/decawaveSrc/decadriver/deca_params_init.o: ../Src/decawaveSrc/decadriver/deca_params_init.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32F105xC -DUSE_HAL_DRIVER -c -I../Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/decawaveSrc/decadriver/deca_params_init.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"


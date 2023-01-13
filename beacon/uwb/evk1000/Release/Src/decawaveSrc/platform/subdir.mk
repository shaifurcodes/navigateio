################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/decawaveSrc/platform/deca_mutex.c \
../Src/decawaveSrc/platform/deca_range_tables.c \
../Src/decawaveSrc/platform/deca_sleep.c \
../Src/decawaveSrc/platform/deca_spi.c \
../Src/decawaveSrc/platform/lcd.c \
../Src/decawaveSrc/platform/port.c 

OBJS += \
./Src/decawaveSrc/platform/deca_mutex.o \
./Src/decawaveSrc/platform/deca_range_tables.o \
./Src/decawaveSrc/platform/deca_sleep.o \
./Src/decawaveSrc/platform/deca_spi.o \
./Src/decawaveSrc/platform/lcd.o \
./Src/decawaveSrc/platform/port.o 

C_DEPS += \
./Src/decawaveSrc/platform/deca_mutex.d \
./Src/decawaveSrc/platform/deca_range_tables.d \
./Src/decawaveSrc/platform/deca_sleep.d \
./Src/decawaveSrc/platform/deca_spi.d \
./Src/decawaveSrc/platform/lcd.d \
./Src/decawaveSrc/platform/port.d 


# Each subdirectory must supply rules for building sources it contributes
Src/decawaveSrc/platform/deca_mutex.o: ../Src/decawaveSrc/platform/deca_mutex.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32F105xC -DUSE_HAL_DRIVER -c -I../Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/decawaveSrc/platform/deca_mutex.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/decawaveSrc/platform/deca_range_tables.o: ../Src/decawaveSrc/platform/deca_range_tables.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32F105xC -DUSE_HAL_DRIVER -c -I../Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/decawaveSrc/platform/deca_range_tables.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/decawaveSrc/platform/deca_sleep.o: ../Src/decawaveSrc/platform/deca_sleep.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32F105xC -DUSE_HAL_DRIVER -c -I../Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/decawaveSrc/platform/deca_sleep.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/decawaveSrc/platform/deca_spi.o: ../Src/decawaveSrc/platform/deca_spi.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32F105xC -DUSE_HAL_DRIVER -c -I../Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/decawaveSrc/platform/deca_spi.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/decawaveSrc/platform/lcd.o: ../Src/decawaveSrc/platform/lcd.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32F105xC -DUSE_HAL_DRIVER -c -I../Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/decawaveSrc/platform/lcd.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/decawaveSrc/platform/port.o: ../Src/decawaveSrc/platform/port.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32F105xC -DUSE_HAL_DRIVER -c -I../Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/decawaveSrc/platform/port.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../My/STM32_Perif/adc_ST.c \
../My/STM32_Perif/crc_ST.c \
../My/STM32_Perif/dma_ST.c \
../My/STM32_Perif/flash_ST.c \
../My/STM32_Perif/gpio_ST.c \
../My/STM32_Perif/i2c_ST.c \
../My/STM32_Perif/iwdg_ST.c \
../My/STM32_Perif/rtc_ST.c \
../My/STM32_Perif/spi_ST.c \
../My/STM32_Perif/stm32f10x_it.c \
../My/STM32_Perif/sys_ST.c \
../My/STM32_Perif/sys_tick_ST.c \
../My/STM32_Perif/timers_ST.c \
../My/STM32_Perif/uart_ST.c 

OBJS += \
./My/STM32_Perif/adc_ST.o \
./My/STM32_Perif/crc_ST.o \
./My/STM32_Perif/dma_ST.o \
./My/STM32_Perif/flash_ST.o \
./My/STM32_Perif/gpio_ST.o \
./My/STM32_Perif/i2c_ST.o \
./My/STM32_Perif/iwdg_ST.o \
./My/STM32_Perif/rtc_ST.o \
./My/STM32_Perif/spi_ST.o \
./My/STM32_Perif/stm32f10x_it.o \
./My/STM32_Perif/sys_ST.o \
./My/STM32_Perif/sys_tick_ST.o \
./My/STM32_Perif/timers_ST.o \
./My/STM32_Perif/uart_ST.o 

C_DEPS += \
./My/STM32_Perif/adc_ST.d \
./My/STM32_Perif/crc_ST.d \
./My/STM32_Perif/dma_ST.d \
./My/STM32_Perif/flash_ST.d \
./My/STM32_Perif/gpio_ST.d \
./My/STM32_Perif/i2c_ST.d \
./My/STM32_Perif/iwdg_ST.d \
./My/STM32_Perif/rtc_ST.d \
./My/STM32_Perif/spi_ST.d \
./My/STM32_Perif/stm32f10x_it.d \
./My/STM32_Perif/sys_ST.d \
./My/STM32_Perif/sys_tick_ST.d \
./My/STM32_Perif/timers_ST.d \
./My/STM32_Perif/uart_ST.d 


# Each subdirectory must supply rules for building sources it contributes
My/STM32_Perif/adc_ST.o: ../My/STM32_Perif/adc_ST.c My/STM32_Perif/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32 -DSTM32F103CBTx -DSTM32F1 -c -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/BootLoader" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Application" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/CMSIS" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Main" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Startup" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/STM32_Perif" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"My/STM32_Perif/adc_ST.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
My/STM32_Perif/crc_ST.o: ../My/STM32_Perif/crc_ST.c My/STM32_Perif/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32 -DSTM32F103CBTx -DSTM32F1 -c -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/BootLoader" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Application" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/CMSIS" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Main" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Startup" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/STM32_Perif" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"My/STM32_Perif/crc_ST.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
My/STM32_Perif/dma_ST.o: ../My/STM32_Perif/dma_ST.c My/STM32_Perif/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32 -DSTM32F103CBTx -DSTM32F1 -c -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/BootLoader" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Application" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/CMSIS" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Main" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Startup" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/STM32_Perif" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"My/STM32_Perif/dma_ST.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
My/STM32_Perif/flash_ST.o: ../My/STM32_Perif/flash_ST.c My/STM32_Perif/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32 -DSTM32F103CBTx -DSTM32F1 -c -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/BootLoader" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Application" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/CMSIS" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Main" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Startup" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/STM32_Perif" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"My/STM32_Perif/flash_ST.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
My/STM32_Perif/gpio_ST.o: ../My/STM32_Perif/gpio_ST.c My/STM32_Perif/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32 -DSTM32F103CBTx -DSTM32F1 -c -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/BootLoader" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Application" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/CMSIS" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Main" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Startup" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/STM32_Perif" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"My/STM32_Perif/gpio_ST.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
My/STM32_Perif/i2c_ST.o: ../My/STM32_Perif/i2c_ST.c My/STM32_Perif/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32 -DSTM32F103CBTx -DSTM32F1 -c -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/BootLoader" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Application" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/CMSIS" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Main" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Startup" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/STM32_Perif" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"My/STM32_Perif/i2c_ST.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
My/STM32_Perif/iwdg_ST.o: ../My/STM32_Perif/iwdg_ST.c My/STM32_Perif/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32 -DSTM32F103CBTx -DSTM32F1 -c -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/BootLoader" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Application" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/CMSIS" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Main" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Startup" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/STM32_Perif" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"My/STM32_Perif/iwdg_ST.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
My/STM32_Perif/rtc_ST.o: ../My/STM32_Perif/rtc_ST.c My/STM32_Perif/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32 -DSTM32F103CBTx -DSTM32F1 -c -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/BootLoader" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Application" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/CMSIS" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Main" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Startup" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/STM32_Perif" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"My/STM32_Perif/rtc_ST.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
My/STM32_Perif/spi_ST.o: ../My/STM32_Perif/spi_ST.c My/STM32_Perif/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32 -DSTM32F103CBTx -DSTM32F1 -c -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/BootLoader" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Application" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/CMSIS" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Main" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Startup" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/STM32_Perif" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"My/STM32_Perif/spi_ST.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
My/STM32_Perif/stm32f10x_it.o: ../My/STM32_Perif/stm32f10x_it.c My/STM32_Perif/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32 -DSTM32F103CBTx -DSTM32F1 -c -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/BootLoader" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Application" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/CMSIS" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Main" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Startup" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/STM32_Perif" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"My/STM32_Perif/stm32f10x_it.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
My/STM32_Perif/sys_ST.o: ../My/STM32_Perif/sys_ST.c My/STM32_Perif/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32 -DSTM32F103CBTx -DSTM32F1 -c -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/BootLoader" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Application" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/CMSIS" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Main" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Startup" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/STM32_Perif" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"My/STM32_Perif/sys_ST.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
My/STM32_Perif/sys_tick_ST.o: ../My/STM32_Perif/sys_tick_ST.c My/STM32_Perif/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32 -DSTM32F103CBTx -DSTM32F1 -c -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/BootLoader" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Application" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/CMSIS" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Main" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Startup" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/STM32_Perif" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"My/STM32_Perif/sys_tick_ST.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
My/STM32_Perif/timers_ST.o: ../My/STM32_Perif/timers_ST.c My/STM32_Perif/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32 -DSTM32F103CBTx -DSTM32F1 -c -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/BootLoader" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Application" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/CMSIS" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Main" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Startup" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/STM32_Perif" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"My/STM32_Perif/timers_ST.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
My/STM32_Perif/uart_ST.o: ../My/STM32_Perif/uart_ST.c My/STM32_Perif/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32 -DSTM32F103CBTx -DSTM32F1 -c -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/BootLoader" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Application" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/CMSIS" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Main" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Startup" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/STM32_Perif" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"My/STM32_Perif/uart_ST.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"


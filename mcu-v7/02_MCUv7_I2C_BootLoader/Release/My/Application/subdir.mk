################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../My/Application/Crc.c \
../My/Application/Delay.c 

OBJS += \
./My/Application/Crc.o \
./My/Application/Delay.o 

C_DEPS += \
./My/Application/Crc.d \
./My/Application/Delay.d 


# Each subdirectory must supply rules for building sources it contributes
My/Application/Crc.o: ../My/Application/Crc.c My/Application/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32 -DSTM32F103CBTx -DSTM32F1 -c -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/BootLoader" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Application" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/CMSIS" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Main" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Startup" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/STM32_Perif" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"My/Application/Crc.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
My/Application/Delay.o: ../My/Application/Delay.c My/Application/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DSTM32 -DSTM32F103CBTx -DSTM32F1 -c -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/BootLoader" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Application" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/CMSIS" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Main" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/Startup" -I"D:/04_Git/GitLab/mcu-v7/02_MCUv7_I2C_BootLoader/My/STM32_Perif" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"My/Application/Delay.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"


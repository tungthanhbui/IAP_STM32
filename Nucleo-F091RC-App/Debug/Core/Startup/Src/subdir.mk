################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Startup/Src/main.c \
../Core/Startup/Src/stm32f0xx_hal_msp.c \
../Core/Startup/Src/stm32f0xx_it.c \
../Core/Startup/Src/syscalls.c \
../Core/Startup/Src/sysmem.c \
../Core/Startup/Src/system_stm32f0xx.c 

OBJS += \
./Core/Startup/Src/main.o \
./Core/Startup/Src/stm32f0xx_hal_msp.o \
./Core/Startup/Src/stm32f0xx_it.o \
./Core/Startup/Src/syscalls.o \
./Core/Startup/Src/sysmem.o \
./Core/Startup/Src/system_stm32f0xx.o 

C_DEPS += \
./Core/Startup/Src/main.d \
./Core/Startup/Src/stm32f0xx_hal_msp.d \
./Core/Startup/Src/stm32f0xx_it.d \
./Core/Startup/Src/syscalls.d \
./Core/Startup/Src/sysmem.d \
./Core/Startup/Src/system_stm32f0xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/Src/%.o: ../Core/Startup/Src/%.c Core/Startup/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F091xC -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"


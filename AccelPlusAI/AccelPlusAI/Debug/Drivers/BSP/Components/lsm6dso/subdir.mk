################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/lsm6dso/lsm6dso.c \
../Drivers/BSP/Components/lsm6dso/lsm6dso_reg.c 

OBJS += \
./Drivers/BSP/Components/lsm6dso/lsm6dso.o \
./Drivers/BSP/Components/lsm6dso/lsm6dso_reg.o 

C_DEPS += \
./Drivers/BSP/Components/lsm6dso/lsm6dso.d \
./Drivers/BSP/Components/lsm6dso/lsm6dso_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/lsm6dso/lsm6dso.o: ../Drivers/BSP/Components/lsm6dso/lsm6dso.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DDEBUG -c -I../Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/BSP/NUCLEO-F401RE -I"C:/Users/Usuario/Desktop/COPIAS_SEGURIDAD/AccelPlusAI/AccelPlusAI/Drivers/BSP/Components/lsm6dso" -I../Drivers/BSP/Components/lsm6dso -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/Components/lsm6dso/lsm6dso.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/BSP/Components/lsm6dso/lsm6dso_reg.o: ../Drivers/BSP/Components/lsm6dso/lsm6dso_reg.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DDEBUG -c -I../Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/BSP/NUCLEO-F401RE -I"C:/Users/Usuario/Desktop/COPIAS_SEGURIDAD/AccelPlusAI/AccelPlusAI/Drivers/BSP/Components/lsm6dso" -I../Drivers/BSP/Components/lsm6dso -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/Components/lsm6dso/lsm6dso_reg.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"


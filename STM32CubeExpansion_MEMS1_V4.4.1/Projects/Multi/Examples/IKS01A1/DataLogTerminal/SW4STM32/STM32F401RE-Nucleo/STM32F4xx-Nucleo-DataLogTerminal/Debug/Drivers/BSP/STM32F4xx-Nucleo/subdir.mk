################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Usuario/Desktop/COPIAS_SEGURIDAD/STM32CubeExpansion_MEMS1_V4.4.1/Drivers/BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.c 

OBJS += \
./Drivers/BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.o 

C_DEPS += \
./Drivers/BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.o: C:/Users/Usuario/Desktop/COPIAS_SEGURIDAD/STM32CubeExpansion_MEMS1_V4.4.1/Drivers/BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DUSE_STM32F4XX_NUCLEO -c -I../../../../Inc -I../../../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../../../../../../../Drivers/BSP/STM32F4xx-Nucleo -I../../../../../../../../../Drivers/CMSIS/Include -I../../../../../../../../../Drivers/BSP/X_NUCLEO_IKS01A1 -I../../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../../../Drivers/BSP/Components/lis3mdl -I../../../../../../../../../Drivers/BSP/Components/lps25hb -I../../../../../../../../../Drivers/BSP/Components/lsm6ds0 -I../../../../../../../../../Drivers/BSP/Components/lsm6ds3 -I../../../../../../../../../Drivers/BSP/Components/lps22hb -O3 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"


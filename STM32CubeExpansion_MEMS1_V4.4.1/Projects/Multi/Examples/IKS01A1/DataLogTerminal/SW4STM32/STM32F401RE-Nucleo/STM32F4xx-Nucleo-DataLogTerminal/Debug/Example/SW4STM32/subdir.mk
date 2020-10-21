################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
C:/Users/Usuario/Desktop/COPIAS_SEGURIDAD/STM32CubeExpansion_MEMS1_V4.4.1/Projects/Multi/Examples/IKS01A1/DataLogTerminal/SW4STM32/STM32F401RE-Nucleo/startup_stm32f401xe.s 

OBJS += \
./Example/SW4STM32/startup_stm32f401xe.o 

S_DEPS += \
./Example/SW4STM32/startup_stm32f401xe.d 


# Each subdirectory must supply rules for building sources it contributes
Example/SW4STM32/startup_stm32f401xe.o: C:/Users/Usuario/Desktop/COPIAS_SEGURIDAD/STM32CubeExpansion_MEMS1_V4.4.1/Projects/Multi/Examples/IKS01A1/DataLogTerminal/SW4STM32/STM32F401RE-Nucleo/startup_stm32f401xe.s
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -I../../../../Inc -I../../../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../../../../../../../Drivers/BSP/STM32F4xx-Nucleo -I../../../../../../../../../Drivers/CMSIS/Include -I../../../../../../../../../Drivers/BSP/X_NUCLEO_IKS01A1 -I../../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../../../Drivers/BSP/Components/lis3mdl -I../../../../../../../../../Drivers/BSP/Components/lps25hb -I../../../../../../../../../Drivers/BSP/Components/lsm6ds0 -I../../../../../../../../../Drivers/BSP/Components/lsm6ds3 -I../../../../../../../../../Drivers/BSP/Components/lps22hb -x assembler-with-cpp -MMD -MP -MF"Example/SW4STM32/startup_stm32f401xe.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"


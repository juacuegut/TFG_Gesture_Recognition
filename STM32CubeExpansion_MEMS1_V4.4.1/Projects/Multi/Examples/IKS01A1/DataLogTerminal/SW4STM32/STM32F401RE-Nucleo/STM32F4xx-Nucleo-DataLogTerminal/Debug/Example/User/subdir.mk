################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Usuario/Desktop/COPIAS_SEGURIDAD/STM32CubeExpansion_MEMS1_V4.4.1/Projects/Multi/Examples/IKS01A1/DataLogTerminal/Src/com.c \
C:/Users/Usuario/Desktop/COPIAS_SEGURIDAD/STM32CubeExpansion_MEMS1_V4.4.1/Projects/Multi/Examples/IKS01A1/DataLogTerminal/Src/cube_hal_f4.c \
C:/Users/Usuario/Desktop/COPIAS_SEGURIDAD/STM32CubeExpansion_MEMS1_V4.4.1/Projects/Multi/Examples/IKS01A1/DataLogTerminal/Src/main.c \
C:/Users/Usuario/Desktop/COPIAS_SEGURIDAD/STM32CubeExpansion_MEMS1_V4.4.1/Projects/Multi/Examples/IKS01A1/DataLogTerminal/Src/stm32f4xx_hal_msp.c \
C:/Users/Usuario/Desktop/COPIAS_SEGURIDAD/STM32CubeExpansion_MEMS1_V4.4.1/Projects/Multi/Examples/IKS01A1/DataLogTerminal/Src/stm32f4xx_it.c \
C:/Users/Usuario/Desktop/COPIAS_SEGURIDAD/STM32CubeExpansion_MEMS1_V4.4.1/Projects/Multi/Examples/IKS01A1/DataLogTerminal/SW4STM32/STM32F401RE-Nucleo/syscalls.c 

OBJS += \
./Example/User/com.o \
./Example/User/cube_hal_f4.o \
./Example/User/main.o \
./Example/User/stm32f4xx_hal_msp.o \
./Example/User/stm32f4xx_it.o \
./Example/User/syscalls.o 

C_DEPS += \
./Example/User/com.d \
./Example/User/cube_hal_f4.d \
./Example/User/main.d \
./Example/User/stm32f4xx_hal_msp.d \
./Example/User/stm32f4xx_it.d \
./Example/User/syscalls.d 


# Each subdirectory must supply rules for building sources it contributes
Example/User/com.o: C:/Users/Usuario/Desktop/COPIAS_SEGURIDAD/STM32CubeExpansion_MEMS1_V4.4.1/Projects/Multi/Examples/IKS01A1/DataLogTerminal/Src/com.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DUSE_STM32F4XX_NUCLEO -c -I../../../../Inc -I../../../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../../../../../../../Drivers/BSP/STM32F4xx-Nucleo -I../../../../../../../../../Drivers/CMSIS/Include -I../../../../../../../../../Drivers/BSP/X_NUCLEO_IKS01A1 -I../../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../../../Drivers/BSP/Components/lis3mdl -I../../../../../../../../../Drivers/BSP/Components/lps25hb -I../../../../../../../../../Drivers/BSP/Components/lsm6ds0 -I../../../../../../../../../Drivers/BSP/Components/lsm6ds3 -I../../../../../../../../../Drivers/BSP/Components/lps22hb -O3 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Example/User/com.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Example/User/cube_hal_f4.o: C:/Users/Usuario/Desktop/COPIAS_SEGURIDAD/STM32CubeExpansion_MEMS1_V4.4.1/Projects/Multi/Examples/IKS01A1/DataLogTerminal/Src/cube_hal_f4.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DUSE_STM32F4XX_NUCLEO -c -I../../../../Inc -I../../../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../../../../../../../Drivers/BSP/STM32F4xx-Nucleo -I../../../../../../../../../Drivers/CMSIS/Include -I../../../../../../../../../Drivers/BSP/X_NUCLEO_IKS01A1 -I../../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../../../Drivers/BSP/Components/lis3mdl -I../../../../../../../../../Drivers/BSP/Components/lps25hb -I../../../../../../../../../Drivers/BSP/Components/lsm6ds0 -I../../../../../../../../../Drivers/BSP/Components/lsm6ds3 -I../../../../../../../../../Drivers/BSP/Components/lps22hb -O3 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Example/User/cube_hal_f4.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Example/User/main.o: C:/Users/Usuario/Desktop/COPIAS_SEGURIDAD/STM32CubeExpansion_MEMS1_V4.4.1/Projects/Multi/Examples/IKS01A1/DataLogTerminal/Src/main.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DUSE_STM32F4XX_NUCLEO -c -I../../../../Inc -I../../../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../../../../../../../Drivers/BSP/STM32F4xx-Nucleo -I../../../../../../../../../Drivers/CMSIS/Include -I../../../../../../../../../Drivers/BSP/X_NUCLEO_IKS01A1 -I../../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../../../Drivers/BSP/Components/lis3mdl -I../../../../../../../../../Drivers/BSP/Components/lps25hb -I../../../../../../../../../Drivers/BSP/Components/lsm6ds0 -I../../../../../../../../../Drivers/BSP/Components/lsm6ds3 -I../../../../../../../../../Drivers/BSP/Components/lps22hb -O3 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Example/User/main.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Example/User/stm32f4xx_hal_msp.o: C:/Users/Usuario/Desktop/COPIAS_SEGURIDAD/STM32CubeExpansion_MEMS1_V4.4.1/Projects/Multi/Examples/IKS01A1/DataLogTerminal/Src/stm32f4xx_hal_msp.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DUSE_STM32F4XX_NUCLEO -c -I../../../../Inc -I../../../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../../../../../../../Drivers/BSP/STM32F4xx-Nucleo -I../../../../../../../../../Drivers/CMSIS/Include -I../../../../../../../../../Drivers/BSP/X_NUCLEO_IKS01A1 -I../../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../../../Drivers/BSP/Components/lis3mdl -I../../../../../../../../../Drivers/BSP/Components/lps25hb -I../../../../../../../../../Drivers/BSP/Components/lsm6ds0 -I../../../../../../../../../Drivers/BSP/Components/lsm6ds3 -I../../../../../../../../../Drivers/BSP/Components/lps22hb -O3 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Example/User/stm32f4xx_hal_msp.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Example/User/stm32f4xx_it.o: C:/Users/Usuario/Desktop/COPIAS_SEGURIDAD/STM32CubeExpansion_MEMS1_V4.4.1/Projects/Multi/Examples/IKS01A1/DataLogTerminal/Src/stm32f4xx_it.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DUSE_STM32F4XX_NUCLEO -c -I../../../../Inc -I../../../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../../../../../../../Drivers/BSP/STM32F4xx-Nucleo -I../../../../../../../../../Drivers/CMSIS/Include -I../../../../../../../../../Drivers/BSP/X_NUCLEO_IKS01A1 -I../../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../../../Drivers/BSP/Components/lis3mdl -I../../../../../../../../../Drivers/BSP/Components/lps25hb -I../../../../../../../../../Drivers/BSP/Components/lsm6ds0 -I../../../../../../../../../Drivers/BSP/Components/lsm6ds3 -I../../../../../../../../../Drivers/BSP/Components/lps22hb -O3 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Example/User/stm32f4xx_it.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Example/User/syscalls.o: C:/Users/Usuario/Desktop/COPIAS_SEGURIDAD/STM32CubeExpansion_MEMS1_V4.4.1/Projects/Multi/Examples/IKS01A1/DataLogTerminal/SW4STM32/STM32F401RE-Nucleo/syscalls.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DUSE_STM32F4XX_NUCLEO -c -I../../../../Inc -I../../../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../../../../../../../Drivers/BSP/STM32F4xx-Nucleo -I../../../../../../../../../Drivers/CMSIS/Include -I../../../../../../../../../Drivers/BSP/X_NUCLEO_IKS01A1 -I../../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../../../Drivers/BSP/Components/lis3mdl -I../../../../../../../../../Drivers/BSP/Components/lps25hb -I../../../../../../../../../Drivers/BSP/Components/lsm6ds0 -I../../../../../../../../../Drivers/BSP/Components/lsm6ds3 -I../../../../../../../../../Drivers/BSP/Components/lps22hb -O3 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Example/User/syscalls.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"


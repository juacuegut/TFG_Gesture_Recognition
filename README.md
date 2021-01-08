# TFG_Gesture_Recognition
Sistema de control de versiones sobre el código desarrollado en el TFG.

## Carpetas:

* **STM32CubeExpansion_MEMS1_V4.4.1**: Contiene ejemplos de uso de las tarjetas de expansión X-NUCLEO-IKS01A1 + NUCLEO-64 STM32F401RE.
Para acceder a la carpeta donde se encuentra el ejemplo del acelerómetro funcionando como único sensor, utilizar la siguiente ruta:

*STM32CubeExpansion_MEMS1_V4.4.1\Projects\Multi\Examples\IKS01A1\DataLogTerminal\SW4STM32\STM32F401RE-Nucleo\STM32F4xx-Nucleo-DataLogTerminal*

[Link directo al archivo .cproject](https://github.com/juacuegut/TFG_Gesture_Recognition/tree/main/STM32CubeExpansion_MEMS1_V4.4.1/Projects/Multi/Examples/IKS01A1/DataLogExtended/SW4STM32/STM32F401RE-Nucleo)

Una vez dentro, seleccionar el archivo .project (o .cproject) para importar el ejemplo al workspace de STM32CubeIDE.

* **Accelerometer_I2C_SPLITTED**: Contiene los ficheros LSM6DS0_CUSTOM.c y LSM6DS0_CUSTOM.h, que permiten usar el Accelerometro LSM6DS0 en cualquier nuevo proyecto.
Contiene además los ficheros retaget.c y retarget.h que implementan la función *printf*.

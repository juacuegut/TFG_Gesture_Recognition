/**
 ******************************************************************************
 * @file    Projects/Multi/Examples/IKS01A1/DataLogTerminal/Src/main.c
 * @author  CL
 * @version V4.0.0
 * @date    1-May-2017
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include <string.h> /* strlen */
#include <stdio.h>  /* snprintf */
#include <math.h>   /* trunc */
#include "main.h"

/** @addtogroup X_NUCLEO_IKS01A1_Examples
  * @{
  */

/** @addtogroup DATALOG_TERMINAL
  * @{
  */

extern int use_LSI;
int RTC_SYNCH_PREDIV;

/* Private typedef -----------------------------------------------------------*/
typedef struct displayFloatToInt_s {
  int8_t sign; /* 0 means positive, 1 means negative*/
  uint32_t  out_int;
  uint32_t  out_dec;
} displayFloatToInt_t;

/* Private define ------------------------------------------------------------*/
#define MAX_BUF_SIZE 256

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile uint8_t acquire_data_enable_request  = 1;
static volatile uint8_t acquire_data_disable_request = 0;
static volatile uint8_t no_X_LSM6DS3_DIL24 = 0;
static volatile uint8_t no_G_LSM6DS3_DIL24 = 0;
static volatile uint8_t no_P_LPS25HB_DIL24 = 0;
static volatile uint8_t no_T_LPS25HB_DIL24 = 0;
static volatile uint8_t no_P_LPS22HB_DIL24 = 0;
static volatile uint8_t no_T_LPS22HB_DIL24 = 0;

static char dataOut[MAX_BUF_SIZE];

static void *LSM6DS0_X_0_handle = NULL;
static void *LSM6DS3_X_0_handle = NULL;



/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main function is to show how to use sensor expansion board to send data from a Nucleo board
 *         using UART to a connected PC or Desktop and display it on generic applications like TeraTerm.
 *         After connection has been established:
 *         - the user can view the data from various on-board environment sensors like Temperature, Humidity, and Pressure.
 *         - the user can also view data from various on-board MEMS sensors as well like Accelerometer, Gyroscope, and Magnetometer.
 * @param  None
 * @retval Integer
 */
int main( void )
{

  /* STM32F4xx HAL library initialization:
  - Configure the Flash prefetch, instruction and Data caches
  - Configure the Systick to generate an interrupt each 1 msec
  - Set NVIC Group Priority to 4
  - Global MSP (MCU Support Package) initialization
  */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize LED */
  BSP_LED_Init( LED2 );

  /* Initialize button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* Initialize UART */
  USARTConfig();

  BSP_ACCELERO_Init( LSM6DS0_X_0, &LSM6DS0_X_0_handle );
  if(BSP_ACCELERO_Init( LSM6DS3_X_0, &LSM6DS3_X_0_handle ) == COMPONENT_ERROR)
  {
    no_X_LSM6DS3_DIL24 = 1;
  }

  uint8_t id;
  SensorAxes_t acceleration;
  uint8_t status;

  while (1)
  {

	  BSP_ACCELERO_Sensor_Enable( LSM6DS0_X_0_handle );
	  if(!no_X_LSM6DS3_DIL24)
	  {
	    BSP_ACCELERO_Sensor_Enable( LSM6DS3_X_0_handle );
	  }

    	  BSP_ACCELERO_Get_Instance( LSM6DS0_X_0_handle, &id );

    	  //La variable status se ha mantenido por estar definida en otros ficheros del acelerometro
    	  BSP_ACCELERO_IsInitialized( LSM6DS0_X_0_handle, &status );

    	  if ( status == 1 )
    	  {
    	    if ( BSP_ACCELERO_Get_Axes( LSM6DS0_X_0_handle, &acceleration ) == COMPONENT_ERROR )
    	    {
    	      acceleration.AXIS_X = 0;
    	      acceleration.AXIS_Y = 0;
    	      acceleration.AXIS_Z = 0;
    	    }

    	    snprintf( dataOut, MAX_BUF_SIZE, "%d,%d,%d\r\n", (int)acceleration.AXIS_X,
    	             (int)acceleration.AXIS_Y, (int)acceleration.AXIS_Z);

    	    HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );
      //Accelero_Sensor_Handler( LSM6DS0_X_0_handle );

      HAL_Delay( 1000 );
    	  }
  }
}





#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed( uint8_t *file, uint32_t line )
{

  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\r\n", file, line) */

  while (1)
  {}
}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

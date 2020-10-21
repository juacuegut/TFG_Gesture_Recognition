/**
 ******************************************************************************
 * @file    Projects/Multi/Examples/IKS01A2/LSM6DSL_SelfTest/Src/main.c
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
#include <math.h>   /* fabs */
#include "main.h"

/** @addtogroup X_NUCLEO_IKS01A2_Examples
 * @{
 */

/** @addtogroup SELFTEST
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/**
 * @brief  Handle DEMO State Machine
 */
typedef enum
{
  STATUS_SELFTEST,
  STATUS_SLEEP
} DEMO_STATUS;


/* Private define ------------------------------------------------------------*/
#define INDICATION_DELAY  1000 /*!< LED is ON for at least this period [ms] */

#define X_POWER_UP_DELAY    100 /*!< Delay after accelero power-up [ms] */
#define X_ST_ENABLED_DELAY  100 /*!< Delay after accelero self-test enabled [ms] */
#define G_POWER_UP_DELAY    150 /*!< Delay after gyro power-up [ms] */
#define G_ST_ENABLED_DELAY   50 /*!< Delay after gyro self-test enabled [ms] */

#define N_SAMPLES  5 /*!< Number of samples */

#define X_LO_LIM      90 /*!< Accelero low test limit [mg] */
#define X_HI_LIM    1700 /*!< Accelero high test limit [mg] */
#define G_LO_LIM  150000 /*!< Gyro low test limit [mdps] */
#define G_HI_LIM  700000 /*!< Gyro high test limit [mdps] */

#define UART_TRANSMIT_TIMEOUT  5000

#define MAX_BUF_SIZE 256

/* Private variables ---------------------------------------------------------*/
static char dataOut[MAX_BUF_SIZE];
static void *LSM6DSL_X_0_handle = NULL;
static void *LSM6DSL_G_0_handle = NULL;

/* This variable MUST be volatile because it could change into a ISR */
static volatile DEMO_STATUS demo_status = STATUS_SLEEP;

/* Refer to Datasheet / Application Note documents for details about following register settings */
uint8_t reg_addr[]        = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19};
uint8_t x_st_reg_values[] = {0x38, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t g_st_reg_values[] = {0x00, 0x5C, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

#define ST_REG_COUNT  (sizeof(reg_addr) / sizeof(uint8_t))


/* Private function prototypes -----------------------------------------------*/
static DrvStatusTypeDef Init_All_Sensors(void);
static DrvStatusTypeDef Enable_All_Sensors(void);
static void Sleep_Mode(void);
static DrvStatusTypeDef LSM6DSL_X_SelfTest(void);
static DrvStatusTypeDef LSM6DSL_G_SelfTest(void);
static DrvStatusTypeDef LSM6DSL_X_Get_Data(SensorAxes_t *data);
static DrvStatusTypeDef LSM6DSL_G_Get_Data(SensorAxes_t *data);



/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Main function is to show how to use sensor expansion board to run the
 *         LSM6DSL accelerometer and gyroscope selftest
 * @param  None
 * @retval Integer
 */
int main(void)
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
  BSP_LED_Init(LED2);

  /* Initialize button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* Initialize UART */
  USARTConfig();

  if (Init_All_Sensors() == COMPONENT_ERROR)
  {
    Error_Handler(__func__);
  }

  if (Enable_All_Sensors() == COMPONENT_ERROR)
  {
    Error_Handler(__func__);
  }

  snprintf(dataOut, MAX_BUF_SIZE, "\r\n------ LSM6DSL self-test DEMO ------\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

  while (1)
  {

    /* Handle DEMO State Machine */
    switch (demo_status)
    {
      case STATUS_SELFTEST:

        if (LSM6DSL_X_SelfTest() == COMPONENT_ERROR)
        {
          Error_Handler(__func__);
        }

        if (LSM6DSL_G_SelfTest() == COMPONENT_ERROR)
        {
          Error_Handler(__func__);
        }

        demo_status = STATUS_SLEEP;
        break;

      case STATUS_SLEEP:

        snprintf(dataOut, MAX_BUF_SIZE, "\r\nPress USER button to start the DEMO ...\r\n");
        HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

        /* Enter sleep mode */
        Sleep_Mode();
        break;

      default:
        Error_Handler(__func__);
        break;
    }
  }
}



/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval COMPONENT_OK
 * @retval COMPONENT_ERROR
 */
static DrvStatusTypeDef Init_All_Sensors(void)
{
  if (BSP_ACCELERO_Init(LSM6DSL_X_0, &LSM6DSL_X_0_handle) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  if (BSP_GYRO_Init(LSM6DSL_G_0, &LSM6DSL_G_0_handle) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief  Enable all sensors
 * @param  None
 * @retval COMPONENT_OK
 * @retval COMPONENT_ERROR
 */
static DrvStatusTypeDef Enable_All_Sensors(void)
{
  if (BSP_ACCELERO_Sensor_Enable(LSM6DSL_X_0_handle) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  if (BSP_GYRO_Sensor_Enable(LSM6DSL_G_0_handle) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief  Enter sleep mode and wait for interrupt
 * @param  None
 * @retval None
 * @retval None
 */
static void Sleep_Mode(void)
{
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk; /* Systick IRQ OFF */
  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; /* Systick IRQ ON */
}



/**
 * @brief  Performs LSM6DSL accelerometer self-test
 * @param  None
 * @retval COMPONENT_OK
 * @retval COMPONENT_ERROR
 */
static DrvStatusTypeDef LSM6DSL_X_SelfTest(void)
{
  int i = 0;
  DrvStatusTypeDef test_result = COMPONENT_OK;
  SensorAxes_t data_nost;
  SensorAxes_t data_st;
  SensorAxes_t data;
  uint8_t prev_reg_values[ST_REG_COUNT];

  snprintf(dataOut, MAX_BUF_SIZE, "\r\nStarting LSM6DSL accelerometer self-test ...\r\nKeep the device still!!!\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

  HAL_Delay(INDICATION_DELAY);
  BSP_LED_On(LED2);

  /* Store current settings of the sensor */
  for (i = 0; i < ST_REG_COUNT; i++)
  {
    if (BSP_ACCELERO_Read_Reg(LSM6DSL_X_0_handle, reg_addr[i], &prev_reg_values[i]) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  /* Set the sensor for self-test */
  for (i = 0; i < ST_REG_COUNT; i++)
  {
    if (BSP_ACCELERO_Write_Reg(LSM6DSL_X_0_handle, reg_addr[i], x_st_reg_values[i]) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  /* Wait defined time for stable output */
  HAL_Delay(X_POWER_UP_DELAY);

  /* Read first data and discard it */
  if (LSM6DSL_X_Get_Data(&data) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  data_nost.AXIS_X = 0;
  data_nost.AXIS_Y = 0;
  data_nost.AXIS_Z = 0;

  /* Read valid data multiple times and average it */
  for (i = 0; i < N_SAMPLES; i++)
  {
    if (LSM6DSL_X_Get_Data(&data) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
    data_nost.AXIS_X += data.AXIS_X;
    data_nost.AXIS_Y += data.AXIS_Y;
    data_nost.AXIS_Z += data.AXIS_Z;
  }
  data_nost.AXIS_X /= N_SAMPLES;
  data_nost.AXIS_Y /= N_SAMPLES;
  data_nost.AXIS_Z /= N_SAMPLES;

  /* Enable self-test */
  if (BSP_ACCELERO_Set_SelfTest_Ext(LSM6DSL_X_0_handle, LSM6DSL_ACC_GYRO_ST_XL_POS_SIGN_TEST) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Wait defined time for stable output */
  HAL_Delay(X_ST_ENABLED_DELAY);

  /* Read first data and discard it */
  if (LSM6DSL_X_Get_Data(&data) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  data_st.AXIS_X = 0;
  data_st.AXIS_Y = 0;
  data_st.AXIS_Z = 0;

  /* Read valid data multiple times and average it */
  for (i = 0; i < N_SAMPLES; i++)
  {
    if (LSM6DSL_X_Get_Data(&data) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
    data_st.AXIS_X += data.AXIS_X;
    data_st.AXIS_Y += data.AXIS_Y;
    data_st.AXIS_Z += data.AXIS_Z;
  }
  data_st.AXIS_X /= N_SAMPLES;
  data_st.AXIS_Y /= N_SAMPLES;
  data_st.AXIS_Z /= N_SAMPLES;

  /* Restore previous settings of the sensor */
  for (i = 0; i < ST_REG_COUNT; i++)
  {
    if (BSP_ACCELERO_Write_Reg(LSM6DSL_X_0_handle, reg_addr[i], prev_reg_values[i]) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  /* Evaluate the test */
  if (fabsf(data_st.AXIS_X - data_nost.AXIS_X) < X_LO_LIM || fabsf(data_st.AXIS_X - data_nost.AXIS_X) > X_HI_LIM)
  {
    test_result = COMPONENT_ERROR;
  }
  if (fabsf(data_st.AXIS_Y - data_nost.AXIS_Y) < X_LO_LIM || fabsf(data_st.AXIS_Y - data_nost.AXIS_Y) > X_HI_LIM)
  {
    test_result = COMPONENT_ERROR;
  }
  if (fabsf(data_st.AXIS_Z - data_nost.AXIS_Z) < X_LO_LIM || fabsf(data_st.AXIS_Z - data_nost.AXIS_Z) > X_HI_LIM)
  {
    test_result = COMPONENT_ERROR;
  }

  /* Print measured data */
  snprintf(dataOut, MAX_BUF_SIZE, "\r\nMeasured acceleration [mg]:\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  snprintf(dataOut, MAX_BUF_SIZE, "\r\n     AXIS     | PRE-SELFTEST |   SELFTEST\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  snprintf(dataOut, MAX_BUF_SIZE, "--------------|--------------|--------------\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  snprintf(dataOut, MAX_BUF_SIZE, "       X      | %8ld     | %8ld\r\n", data_nost.AXIS_X, data_st.AXIS_X);
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  snprintf(dataOut, MAX_BUF_SIZE, "       Y      | %8ld     | %8ld\r\n", data_nost.AXIS_Y, data_st.AXIS_Y);
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  snprintf(dataOut, MAX_BUF_SIZE, "       Z      | %8ld     | %8ld\r\n", data_nost.AXIS_Z, data_st.AXIS_Z);
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

  /* Print test limits and data */
  snprintf(dataOut, MAX_BUF_SIZE, "\r\nTest limits and data [mg]:\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  snprintf(dataOut, MAX_BUF_SIZE, "\r\n  LOW LIMIT   |  DIFFERENCE  |  HIGH LIMIT\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  snprintf(dataOut, MAX_BUF_SIZE, "--------------|--------------|--------------\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  snprintf(dataOut, MAX_BUF_SIZE, "%8d      | %8d     | %8d\r\n", X_LO_LIM, (int)fabsf(data_st.AXIS_X - data_nost.AXIS_X), X_HI_LIM);
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  snprintf(dataOut, MAX_BUF_SIZE, "%8d      | %8d     | %8d\r\n", X_LO_LIM, (int)fabsf(data_st.AXIS_Y - data_nost.AXIS_Y), X_HI_LIM);
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  snprintf(dataOut, MAX_BUF_SIZE, "%8d      | %8d     | %8d\r\n", X_LO_LIM, (int)fabsf(data_st.AXIS_Z - data_nost.AXIS_Z), X_HI_LIM);
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

  /* Print the test result */
  if (test_result == COMPONENT_OK)
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nLSM6DSL accelerometer self-test PASSED!\r\n");
  }
  else
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nLSM6DSL accelerometer self-test FAILED!\r\n");
  }
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

  BSP_LED_Off(LED2);

  return COMPONENT_OK;
}



/**
 * @brief  Performs LSM6DSL gyroscope self-test
 * @param  None
 * @retval COMPONENT_OK
 * @retval COMPONENT_ERROR
 */
static DrvStatusTypeDef LSM6DSL_G_SelfTest(void)
{
  int i = 0;
  DrvStatusTypeDef test_result = COMPONENT_OK;
  SensorAxes_t data_nost;
  SensorAxes_t data_st;
  SensorAxes_t data;
  uint8_t prev_reg_values[ST_REG_COUNT];

  snprintf(dataOut, MAX_BUF_SIZE, "\r\nStarting LSM6DSL gyroscope self-test ...\r\nKeep the device still!!!\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

  HAL_Delay(INDICATION_DELAY);
  BSP_LED_On(LED2);

  /* Store current settings of the sensor */
  for (i = 0; i < ST_REG_COUNT; i++)
  {
    if (BSP_GYRO_Read_Reg(LSM6DSL_G_0_handle, reg_addr[i], &prev_reg_values[i]) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  /* Set the sensor for self-test */
  for (i = 0; i < ST_REG_COUNT; i++)
  {
    if (BSP_GYRO_Write_Reg(LSM6DSL_G_0_handle, reg_addr[i], g_st_reg_values[i]) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  /* Wait defined time for stable output */
  HAL_Delay(G_POWER_UP_DELAY);

  /* Read first data and discard it */
  if (LSM6DSL_G_Get_Data(&data) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  data_nost.AXIS_X = 0;
  data_nost.AXIS_Y = 0;
  data_nost.AXIS_Z = 0;

  /* Read valid data multiple times and average it */
  for (i = 0; i < N_SAMPLES; i++)
  {
    if (LSM6DSL_G_Get_Data(&data) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
    data_nost.AXIS_X += data.AXIS_X;
    data_nost.AXIS_Y += data.AXIS_Y;
    data_nost.AXIS_Z += data.AXIS_Z;
  }
  data_nost.AXIS_X /= N_SAMPLES;
  data_nost.AXIS_Y /= N_SAMPLES;
  data_nost.AXIS_Z /= N_SAMPLES;

  /* Enable self-test */
  if (BSP_GYRO_Set_SelfTest_Ext(LSM6DSL_G_0_handle, LSM6DSL_ACC_GYRO_ST_G_POS_SIGN_TEST) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Wait defined time for stable output */
  HAL_Delay(G_ST_ENABLED_DELAY);

  /* Read first data and discard it */
  if (LSM6DSL_G_Get_Data(&data) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  data_st.AXIS_X = 0;
  data_st.AXIS_Y = 0;
  data_st.AXIS_Z = 0;

  /* Read valid data multiple times and average it */
  for (i = 0; i < N_SAMPLES; i++)
  {
    if (LSM6DSL_G_Get_Data(&data) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
    data_st.AXIS_X += data.AXIS_X;
    data_st.AXIS_Y += data.AXIS_Y;
    data_st.AXIS_Z += data.AXIS_Z;
  }
  data_st.AXIS_X /= N_SAMPLES;
  data_st.AXIS_Y /= N_SAMPLES;
  data_st.AXIS_Z /= N_SAMPLES;

  /* Restore previous settings of the sensor */
  for (i = 0; i < ST_REG_COUNT; i++)
  {
    if (BSP_GYRO_Write_Reg(LSM6DSL_G_0_handle, reg_addr[i], prev_reg_values[i]) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  /* Evaluate the test */
  if (fabsf(data_st.AXIS_X - data_nost.AXIS_X) < G_LO_LIM || fabsf(data_st.AXIS_X - data_nost.AXIS_X) > G_HI_LIM)
  {
    test_result = COMPONENT_ERROR;
  }
  if (fabsf(data_st.AXIS_Y - data_nost.AXIS_Y) < G_LO_LIM || fabsf(data_st.AXIS_Y - data_nost.AXIS_Y) > G_HI_LIM)
  {
    test_result = COMPONENT_ERROR;
  }
  if (fabsf(data_st.AXIS_Z - data_nost.AXIS_Z) < G_LO_LIM || fabsf(data_st.AXIS_Z - data_nost.AXIS_Z) > G_HI_LIM)
  {
    test_result = COMPONENT_ERROR;
  }

  /* Print measured data */
  snprintf(dataOut, MAX_BUF_SIZE, "\r\nMeasured angular velocity [mdps]:\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  snprintf(dataOut, MAX_BUF_SIZE, "\r\n     AXIS     | PRE-SELFTEST |   SELFTEST\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  snprintf(dataOut, MAX_BUF_SIZE, "--------------|--------------|--------------\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  snprintf(dataOut, MAX_BUF_SIZE, "       X      |  %8ld    |  %8ld\r\n", data_nost.AXIS_X, data_st.AXIS_X);
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  snprintf(dataOut, MAX_BUF_SIZE, "       Y      |  %8ld    |  %8ld\r\n", data_nost.AXIS_Y, data_st.AXIS_Y);
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  snprintf(dataOut, MAX_BUF_SIZE, "       Z      |  %8ld    |  %8ld\r\n", data_nost.AXIS_Z, data_st.AXIS_Z);
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

  /* Print test limits and data */
  snprintf(dataOut, MAX_BUF_SIZE, "\r\nTest limits and data [mdps]:\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  snprintf(dataOut, MAX_BUF_SIZE, "\r\n  LOW LIMIT   |  DIFFERENCE  |  HIGH LIMIT\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  snprintf(dataOut, MAX_BUF_SIZE, "--------------|--------------|--------------\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  snprintf(dataOut, MAX_BUF_SIZE, "  %8d    |  %8d    |  %8d\r\n", G_LO_LIM, (int)fabsf(data_st.AXIS_X - data_nost.AXIS_X), G_HI_LIM);
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  snprintf(dataOut, MAX_BUF_SIZE, "  %8d    |  %8d    |  %8d\r\n", G_LO_LIM, (int)fabsf(data_st.AXIS_Y - data_nost.AXIS_Y), G_HI_LIM);
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  snprintf(dataOut, MAX_BUF_SIZE, "  %8d    |  %8d    |  %8d\r\n", G_LO_LIM, (int)fabsf(data_st.AXIS_Z - data_nost.AXIS_Z), G_HI_LIM);
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

  /* Print the test result */
  if (test_result == COMPONENT_OK)
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nLSM6DSL gyroscope self-test PASSED!\r\n");
  }
  else
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nLSM6DSL gyroscope self-test FAILED!\r\n");
  }
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

  BSP_LED_Off(LED2);

  return COMPONENT_OK;
}



/**
 * @brief  Wait for data ready and get data
 * @param  data the sensor data
 * @retval COMPONENT_OK
 * @retval COMPONENT_ERROR
 */
static DrvStatusTypeDef LSM6DSL_X_Get_Data(SensorAxes_t *data)
{
  uint8_t status;

  /* Wait for data ready */
  do
  {
    if (BSP_ACCELERO_Get_DRDY_Status(LSM6DSL_X_0_handle, &status) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  while (status == 0);

  /* Read accelero data */
  if (BSP_ACCELERO_Get_Axes(LSM6DSL_X_0_handle, data) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief  Wait for data ready and get data
 * @param  data the sensor data
 * @retval COMPONENT_OK
 * @retval COMPONENT_ERROR
 */
static DrvStatusTypeDef LSM6DSL_G_Get_Data(SensorAxes_t *data)
{
  uint8_t status;

  /* Wait for data ready */
  do
  {
    if (BSP_GYRO_Get_DRDY_Status(LSM6DSL_G_0_handle, &status) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  while (status == 0);

  /* Read accelero data */
  if (BSP_GYRO_Get_Axes(LSM6DSL_G_0_handle, data) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief  EXTI line detection callbacks
 * @param  GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* User button pressed */
  if(GPIO_Pin == KEY_BUTTON_PIN)
  {
    if (BSP_PB_GetState(BUTTON_KEY) == GPIO_PIN_RESET)
    {
      /* _NOTE_: Pushing button creates interrupt/event and wakes up MCU from sleep mode */
      demo_status = STATUS_SELFTEST;
    }
  }

  /* ERROR */
  else
  {
    Error_Handler(__func__);
  }
}



/**
 * @brief  This function is executed in case of error occurrence, turns LED2 ON and ends in infinite loop
 * @param  None
 * @retval None
 */
void Error_Handler(const char *function_name)
{
  snprintf(dataOut, MAX_BUF_SIZE, "\r\nError in '%s' function.\r\n", function_name);
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

  while (1)
  {
    BSP_LED_On(LED2);
    HAL_Delay(INDICATION_DELAY);
    BSP_LED_Off(LED2);
    HAL_Delay(INDICATION_DELAY);
  }
}



#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred
 * @param  file pointer to the source file name
 * @param  line assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
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

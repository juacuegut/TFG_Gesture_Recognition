/**
  ******************************************************************************
  * @file    main.c
  * @author  MEMS Application Team
  * @version V4.3.0
  * @date    26-February-2018
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

#include <string.h>
#include <stdio.h>
#include <math.h>
#include "main.h"
#include "vcom.h"
#include "DemoSerial.h"

/** @addtogroup SENSORTILE_Examples
  * @{
  */

/** @addtogroup DATALOG_EXTENDED
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

typedef struct displayFloatToInt_s {
  int8_t   sign;     /* 0 means positive, 1 means negative*/
  uint32_t out_int;
  uint32_t out_dec;
} displayFloatToInt_t;

/* Private define ------------------------------------------------------------*/

#define DATA_SIZE  256

/* Private macro -------------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/

uint32_t Sensors_Enabled  = 0;                  /*!< Sensors Enabled/Disabled */
uint32_t DataTxPeriod     = 50;                 /*!< TX DATA Period */
uint8_t  DataLoggerActive = 0;                  /*!< DataLogger ON/OFF */

void *ACCELERO_handle    = NULL;
void *GYRO_handle        = NULL;
void *MAGNETO_handle     = NULL;
void *HUMIDITY_handle    = NULL;
void *TEMPERATURE_handle = NULL;
void *PRESSURE_handle    = NULL;

/* Private variables ---------------------------------------------------------*/

static uint8_t  TerminalActive           = 0;  /*!< COM port terminal active */
static uint32_t Previous_Sensors_Enabled = 0;  /*!< Previously Stored Sensors Enabled/Disabled Flag */

// static char dataIn[DATA_SIZE];              /*!< DataIn buffer */
static char dataOut[DATA_SIZE];                /*!< DataOut buffer */

static SensorAxes_t ACC_Value;                 /*!< Acceleration Value */
static SensorAxes_t GYR_Value;                 /*!< Gyroscope Value */
static SensorAxes_t MAG_Value;                 /*!< Magnetometer Value */
static float PRESSURE_Value;                   /*!< Pressure Value */
static float HUMIDITY_Value;                   /*!< Humidity Value */
static float TEMPERATURE_Value;                /*!< Temperature Value */

static uint8_t new_data       = 0;
static uint8_t new_data_flags = 0;

static RTC_HandleTypeDef RtcHandle;            /*!< RTC HANDLE */

static int RTC_SYNCH_PREDIV;

/* Private function prototypes -----------------------------------------------*/

static void RTC_Config(void);
static void RTC_TimeStampConfig(void);
static void RTC_Handler(TMsg *Msg);

static void initializeAllSensors(void);
static void enableDisableSensors(void);

static void Accelero_Sensor_Handler(TMsg *Msg);
static void Gyro_Sensor_Handler(TMsg *Msg);
static void Magneto_Sensor_Handler(TMsg *Msg);
static void Pressure_Sensor_Handler(TMsg *Msg);
static void Humidity_Sensor_Handler(TMsg *Msg);
static void Temperature_Sensor_Handler(TMsg *Msg);

static void floatToInt(float in, displayFloatToInt_t *out_value, int32_t dec_prec);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main function is to show how to use X_NUCLEO_IKS01A2 expansion board to send data from a Nucleo board
  *         using UART to a connected PC or Desktop and display it on generic applications like
  *         TeraTerm and specific application like Unicleo GUI, which is developed by STMicroelectronics
  *         and provided with a separated package.
  *
  *         After connection has been established:
  *         - the user can view the data from various on-board environment sensors like Temperature, Humidity, and Pressure
  *         - the user can also view data from various on-board MEMS sensors as well like Accelerometer, Gyrometer, and Magnetometer
  *         - the user can also visualize this data as graphs using Sensors_DataLog application provided with this package
  *
  * @param  None
  * @retval None
  */
int main(void)
{
  TMsg Msg;

  /* Init 'Msg' to defaults */
  Msg.Len = 0;
  memset(Msg.Data, 0, sizeof(Msg.Data));

  /* STM32L4xx HAL library initialization:
  - Configure the Flash prefetch, instruction and Data caches
  - Configure the Systick to generate an interrupt each 1 msec
  - Set NVIC Group Priority to 4
  - Global MSP (MCU Support Package) initialization
   */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize LEDs */
  BSP_LED_Init(LED1);
  BSP_LED_Off(LED1);

  /* Initialize VCOM (Virtual COM Port) */
  VCOM_init();

  /* Initialize RTC */
  RTC_Config();
  RTC_TimeStampConfig();

  /* Initialize sensors */
  initializeAllSensors();

  while (1)
  {
    if (VCOM_receive_MSG(&Msg, TMsg_MaxLen) == OK)
    {
      if (Msg.Data[0] == DEV_ADDR)
      {
        HandleMSG(&Msg);
        if (DataLoggerActive)
        {
          TerminalActive = 0;
        }
      }
    }

    if (Previous_Sensors_Enabled != Sensors_Enabled)
    {
      Previous_Sensors_Enabled = Sensors_Enabled;
      enableDisableSensors();
    }

    RTC_Handler(&Msg);

    if (Sensors_Enabled & PRESSURE_SENSOR)
    {
      Pressure_Sensor_Handler(&Msg);
    }

    if (Sensors_Enabled & HUMIDITY_SENSOR)
    {
      Humidity_Sensor_Handler(&Msg);
    }

    if (Sensors_Enabled & TEMPERATURE_SENSOR)
    {
      Temperature_Sensor_Handler(&Msg);
    }

    if (Sensors_Enabled & ACCELEROMETER_SENSOR)
    {
      Accelero_Sensor_Handler(&Msg);
    }

    if (Sensors_Enabled & GYROSCOPE_SENSOR)
    {
      Gyro_Sensor_Handler(&Msg);
    }

    if (Sensors_Enabled & MAGNETIC_SENSOR)
    {
      Magneto_Sensor_Handler(&Msg);
    }

    if (DataLoggerActive || TerminalActive)
    {
      BSP_LED_Toggle(LED1);
    }
    else
    {
      BSP_LED_Off(LED1);
    }

    if(DataLoggerActive)
    {
      if (new_data != 0)
      {
        INIT_STREAMING_HEADER(&Msg);
        Msg.Data[55] = new_data_flags;
        Msg.Len = STREAMING_MSG_LENGTH;
        VCOM_send_MSG(&Msg);
        new_data = 0;
        new_data_flags = 0;
      }
    }

    if (TerminalActive)
    {
      HAL_Delay(DataTxPeriod);
    }
  }
}


/**
  * @brief  Initialize all sensors
  * @param  None
  * @retval None
  */
static void initializeAllSensors(void)
{
  /* Try to use LSM6DSM (SensorTile), then try to use LSM303AGR (SensorTile) */
  BSP_ACCELERO_Init(ACCELERO_SENSORS_AUTO, &ACCELERO_handle);

  /* Try to use LSM6DSM (SensorTile) */
  BSP_GYRO_Init(GYRO_SENSORS_AUTO, &GYRO_handle);

  /* Try to use LSM303AGR (SensorTile) */
  BSP_MAGNETO_Init(MAGNETO_SENSORS_AUTO, &MAGNETO_handle);

  /* Try to use HTS221 (SensorTile Cradle) */
  BSP_HUMIDITY_Init(HUMIDITY_SENSORS_AUTO, &HUMIDITY_handle);

  /* Try to use HTS221 (SensorTile Cradle), then try to use LPS22HB (SensorTile) */
  BSP_TEMPERATURE_Init(TEMPERATURE_SENSORS_AUTO, &TEMPERATURE_handle);

  /* Try to use LPS22HB (SensorTile) */
  BSP_PRESSURE_Init(PRESSURE_SENSORS_AUTO, &PRESSURE_handle);
}


/**
 * @brief  Enable/disable desired sensors
 * @param  None
 * @retval None
 */
static void enableDisableSensors(void)
{
  if (Sensors_Enabled & PRESSURE_SENSOR)
  {
    BSP_PRESSURE_Sensor_Enable(PRESSURE_handle);
  }
  else
  {
    BSP_PRESSURE_Sensor_Disable(PRESSURE_handle);
  }

  if (Sensors_Enabled & HUMIDITY_SENSOR)
  {
    BSP_HUMIDITY_Sensor_Enable(HUMIDITY_handle);
  }
  else
  {
    BSP_HUMIDITY_Sensor_Disable(HUMIDITY_handle);
  }

  if (Sensors_Enabled & TEMPERATURE_SENSOR)
  {
    BSP_TEMPERATURE_Sensor_Enable(TEMPERATURE_handle);
  }
  else
  {
    BSP_TEMPERATURE_Sensor_Disable(TEMPERATURE_handle);
  }

  if (Sensors_Enabled & ACCELEROMETER_SENSOR)
  {
    BSP_ACCELERO_Sensor_Enable(ACCELERO_handle);
  }
  else
  {
    BSP_ACCELERO_Sensor_Disable(ACCELERO_handle);
  }

  if (Sensors_Enabled & GYROSCOPE_SENSOR)
  {
    BSP_GYRO_Sensor_Enable(GYRO_handle);
  }
  else
  {
    BSP_GYRO_Sensor_Disable(GYRO_handle);
  }

  if (Sensors_Enabled & MAGNETIC_SENSOR)
  {
    BSP_MAGNETO_Sensor_Enable(MAGNETO_handle);
  }
  else
  {
    BSP_MAGNETO_Sensor_Disable(MAGNETO_handle);
  }
}


/**
 * @brief  Handles the time+date getting/sending
 * @param  Msg the time+date part of the stream
 * @retval None
 */
static void RTC_Handler(TMsg *Msg)
{
  uint8_t subSec = 0;
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructure;

  if (DataLoggerActive || TerminalActive)
  {
    HAL_RTC_GetTime(&RtcHandle, &stimestructure, FORMAT_BIN);
    HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, FORMAT_BIN);
    subSec = ((((((int)RTC_SYNCH_PREDIV) - ((int) stimestructure.SubSeconds)) * 100) / (RTC_SYNCH_PREDIV + 1)) & 0xFF);
  }

  if (DataLoggerActive)
  {
    Msg->Data[3] = (uint8_t)stimestructure.Hours;
    Msg->Data[4] = (uint8_t)stimestructure.Minutes;
    Msg->Data[5] = (uint8_t)stimestructure.Seconds;
    Msg->Data[6] = subSec;
  }
  else if (TerminalActive)
  {
    snprintf(dataOut, DATA_SIZE, "\nTimeStamp: %02d:%02d:%02d.%02d\n", stimestructure.Hours, stimestructure.Minutes, stimestructure.Seconds, subSec);
    VCOM_write(dataOut, strlen(dataOut));
  }
}


/**
 * @brief  Handles the ACCELERO axes data getting/sending
 * @param  Msg the ACCELERO part of the stream
 * @retval None
 */
static void Accelero_Sensor_Handler(TMsg *Msg)
{
  int32_t data[6];
  uint8_t status = 0;
  uint8_t drdy = 0;

  if (BSP_ACCELERO_IsInitialized(ACCELERO_handle, &status) == COMPONENT_OK && status == 1)
  {
    BSP_ACCELERO_Get_DRDY_Status(ACCELERO_handle, &drdy);

    if (drdy != 0)
    {
      new_data++;
      new_data_flags |= 1;

      BSP_ACCELERO_Get_Axes(ACCELERO_handle, &ACC_Value);

      if (DataLoggerActive)
      {
        Serialize_s32(&Msg->Data[19], ACC_Value.AXIS_X, 4);
        Serialize_s32(&Msg->Data[23], ACC_Value.AXIS_Y, 4);
        Serialize_s32(&Msg->Data[27], ACC_Value.AXIS_Z, 4);
      }
      else if (TerminalActive)
      {
        data[0] = ACC_Value.AXIS_X;
        data[1] = ACC_Value.AXIS_Y;
        data[2] = ACC_Value.AXIS_Z;

        snprintf(dataOut, DATA_SIZE, "ACC_X: %d, ACC_Y: %d, ACC_Z: %d\n", (int)data[0], (int)data[1], (int)data[2]);
        VCOM_write(dataOut, strlen(dataOut));
      }
    }
  }
}


/**
 * @brief  Handles the GYRO axes data getting/sending
 * @param  Msg the GYRO part of the stream
 * @retval None
 */
static void Gyro_Sensor_Handler(TMsg *Msg)
{
  int32_t data[6];
  uint8_t status = 0;
  uint8_t drdy = 0;

  if (BSP_GYRO_IsInitialized(GYRO_handle, &status) == COMPONENT_OK && status == 1)
  {
    BSP_GYRO_Get_DRDY_Status(GYRO_handle, &drdy);

    if (drdy != 0)
    {
      new_data++;
      new_data_flags |= 2;

      BSP_GYRO_Get_Axes(GYRO_handle, &GYR_Value);

      if (DataLoggerActive)
      {
        Serialize_s32(&Msg->Data[31], GYR_Value.AXIS_X, 4);
        Serialize_s32(&Msg->Data[35], GYR_Value.AXIS_Y, 4);
        Serialize_s32(&Msg->Data[39], GYR_Value.AXIS_Z, 4);
      }
      else if (TerminalActive)
      {
        data[3] = GYR_Value.AXIS_X;
        data[4] = GYR_Value.AXIS_Y;
        data[5] = GYR_Value.AXIS_Z;

        snprintf(dataOut, DATA_SIZE, "GYR_X: %d, GYR_Y: %d, GYR_Z: %d\n", (int)data[3], (int)data[4], (int)data[5]);
        VCOM_write(dataOut, strlen(dataOut));
      }
    }
  }
}


/**
 * @brief  Handles the MAGNETO axes data getting/sending
 * @param  Msg the MAGNETO part of the stream
 * @retval None
 */
static void Magneto_Sensor_Handler(TMsg *Msg)
{
  int32_t data[3];
  uint8_t status = 0;
  uint8_t drdy = 0;

  if (BSP_MAGNETO_IsInitialized(MAGNETO_handle, &status) == COMPONENT_OK && status == 1)
  {
    BSP_MAGNETO_Get_DRDY_Status(MAGNETO_handle, &drdy);

    if (drdy != 0)
    {
      new_data++;
      new_data_flags |= 4;

      BSP_MAGNETO_Get_Axes(MAGNETO_handle, &MAG_Value);

      if (DataLoggerActive)
      {
        Serialize_s32(&Msg->Data[43], MAG_Value.AXIS_X, 4);
        Serialize_s32(&Msg->Data[47], MAG_Value.AXIS_Y, 4);
        Serialize_s32(&Msg->Data[51], MAG_Value.AXIS_Z, 4);
      }
      else if (TerminalActive)
      {
        data[0] = MAG_Value.AXIS_X;
        data[1] = MAG_Value.AXIS_Y;
        data[2] = MAG_Value.AXIS_Z;

        snprintf(dataOut, DATA_SIZE, "MAG_X: %d, MAG_Y: %d, MAG_Z: %d\n", (int)data[0], (int)data[1], (int)data[2]);
        VCOM_write(dataOut, strlen(dataOut));
      }
    }
  }
}


/**
 * @brief  Handles the PRESSURE sensor data getting/sending
 * @param  Msg the PRESSURE part of the stream
 * @retval None
 */
static void Pressure_Sensor_Handler(TMsg *Msg)
{
  uint8_t status = 0;
  uint8_t drdy = 0;

  if (BSP_PRESSURE_IsInitialized(PRESSURE_handle, &status) == COMPONENT_OK && status == 1)
  {
    BSP_PRESSURE_Get_DRDY_Status(PRESSURE_handle, &drdy);

    if (drdy != 0)
    {
      new_data++;
      new_data_flags |= 8;

      BSP_PRESSURE_Get_Press(PRESSURE_handle, &PRESSURE_Value);

      if (DataLoggerActive)
      {
        memcpy(&Msg->Data[7], (void *)&PRESSURE_Value, sizeof(float));
      }
      else if (TerminalActive)
      {
        displayFloatToInt_t out_value;
        floatToInt(PRESSURE_Value, &out_value, 2);
        snprintf(dataOut, DATA_SIZE, "PRESS: %d.%02d\n", (int)out_value.out_int, (int)out_value.out_dec);
        VCOM_write(dataOut, strlen(dataOut));
      }
    }
  }
}


/**
 * @brief  Handles the HUMIDITY sensor data getting/sending
 * @param  Msg the HUMIDITY part of the stream
 * @retval None
 */
static void Humidity_Sensor_Handler(TMsg *Msg)
{
  uint8_t status = 0;
  uint8_t drdy = 0;

  if (BSP_HUMIDITY_IsInitialized(HUMIDITY_handle, &status) == COMPONENT_OK && status == 1)
  {
    BSP_HUMIDITY_Get_DRDY_Status(HUMIDITY_handle, &drdy);

    if (drdy != 0)
    {
      new_data++;
      new_data_flags |= 16;

      BSP_HUMIDITY_Get_Hum(HUMIDITY_handle, &HUMIDITY_Value);

      if (DataLoggerActive)
      {
        memcpy(&Msg->Data[15], (void *)&HUMIDITY_Value, sizeof(float));
      }
      else if (TerminalActive)
      {
        displayFloatToInt_t out_value;
        floatToInt(HUMIDITY_Value, &out_value, 2);
        snprintf(dataOut, DATA_SIZE, "HUM: %d.%02d\n", (int)out_value.out_int, (int)out_value.out_dec);
        VCOM_write(dataOut, strlen(dataOut));
      }
    }
  }
}


/**
 * @brief  Handles the TEMPERATURE sensor data getting/sending
 * @param  Msg the TEMPERATURE part of the stream
 * @retval None
 */
static void Temperature_Sensor_Handler(TMsg *Msg)
{
  uint8_t status = 0;
  uint8_t drdy = 0;

  if (BSP_TEMPERATURE_IsInitialized(TEMPERATURE_handle, &status) == COMPONENT_OK && status == 1)
  {
    BSP_TEMPERATURE_Get_DRDY_Status(TEMPERATURE_handle, &drdy);

    if (drdy != 0)
    {
      new_data++;
      new_data_flags |= 32;

      BSP_TEMPERATURE_Get_Temp(TEMPERATURE_handle, &TEMPERATURE_Value);

      if (DataLoggerActive)
      {
        memcpy(&Msg->Data[11], (void *)&TEMPERATURE_Value, sizeof(float));
      }
      else if (TerminalActive)
      {
        displayFloatToInt_t out_value;
        floatToInt(TEMPERATURE_Value, &out_value, 2);
        snprintf(dataOut, DATA_SIZE, "TEMP: %c%d.%02d\n", ((out_value.sign) ? '-' : '+'), (int)out_value.out_int, (int)out_value.out_dec);
        VCOM_write(dataOut, strlen(dataOut));
      }
    }
  }
}


/**
  * @brief  Configures the RTC
  * @param  None
  * @retval None
  */
static void RTC_Config(void)
{
  RtcHandle.Instance = RTC;

  /* Configure RTC prescaler and RTC data registers */
  /* RTC configured as follow:
    - Hour Format    = Format 12
    - Asynch Prediv  = Value according to source clock
    - Synch Prediv   = Value according to source clock
    - OutPut         = Output Disable
    - OutPutPolarity = High Polarity
    - OutPutType     = Open Drain
   */
  RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_12;
  RtcHandle.Init.AsynchPrediv   = RTC_ASYNCH_PREDIV_LSE;
  RtcHandle.Init.SynchPrediv    = RTC_SYNCH_PREDIV_LSE;
  RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;

  RTC_SYNCH_PREDIV = RTC_SYNCH_PREDIV_LSE;

  if (HAL_RTC_Init(&RtcHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}


/**
  * @brief  Configures the current time and date
  * @param  None
  * @retval None
  */
static void RTC_TimeStampConfig(void)
{
  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;

  /* Configure the Date using BCD format */
  /* Set Date: Monday January 1st 2000 */
  sdatestructure.Year    = 0x00;
  sdatestructure.Month   = RTC_MONTH_JANUARY;
  sdatestructure.Date    = 0x01;
  sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;

  if (HAL_RTC_SetDate(&RtcHandle, &sdatestructure, FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Configure the Time using BCD format */
  /* Set Time: 00:00:00 */
  stimestructure.Hours          = 0x00;
  stimestructure.Minutes        = 0x00;
  stimestructure.Seconds        = 0x00;
  stimestructure.TimeFormat     = RTC_HOURFORMAT12_AM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}


/**
  * @brief  Configures the current date
  * @param  y the year value to be set
  * @param  m the month value to be set
  * @param  d the day value to be set
  * @param  dw the day-week value to be set
  * @retval None
  */
void RTC_DateRegulate(uint8_t y, uint8_t m, uint8_t d, uint8_t dw)
{
  RTC_DateTypeDef sdatestructure;

  sdatestructure.Year    = y;
  sdatestructure.Month   = m;
  sdatestructure.Date    = d;
  sdatestructure.WeekDay = dw;

  if (HAL_RTC_SetDate(&RtcHandle, &sdatestructure,FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}


/**
  * @brief  Configures the current time and date
  * @param  hh the hour value to be set
  * @param  mm the minute value to be set
  * @param  ss the second value to be set
  * @retval None
  */
void RTC_TimeRegulate(uint8_t hh, uint8_t mm, uint8_t ss)
{

  RTC_TimeTypeDef stimestructure;

  stimestructure.TimeFormat     = RTC_HOURFORMAT12_AM;
  stimestructure.Hours          = hh;
  stimestructure.Minutes        = mm;
  stimestructure.Seconds        = ss;
  stimestructure.SubSeconds     = 0;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}


/**
  * @brief  TIM period elapsed callback
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    CDC_TIM_PeriodElapsedCallback(htim);
}


/**
  * @brief  This function is executed in case of error occurrence
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  while (1)
  {}
}

/**
  * @brief  Splits a float into two integer values.
  * @param  in the float value as input
  * @param  out_value the pointer to the output integer structure
  * @param  dec_prec the decimal precision to be used
  * @retval None
  */
static void floatToInt(float in, displayFloatToInt_t *out_value, int32_t dec_prec)
{
  if(in >= 0.0f)
  {
    out_value->sign = 0;
  }
  else
  {
    out_value->sign = 1;
    in = -in;
  }

  out_value->out_int = (int32_t)in;
  in = in - (float)(out_value->out_int);
  out_value->out_dec = (int32_t)trunc(in * pow(10, dec_prec));
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{

  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\n", file, line) */

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

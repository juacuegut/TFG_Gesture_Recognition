/**
  ******************************************************************************
  * @file        DemoSerial.h
  * @author      MEMS Application Team
  * @version     V2.0.0
  * @date        01-May-2017
  * @brief       Header for DemoSerial.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEMO_SERIAL_H__
#define __DEMO_SERIAL_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "cube_hal.h"
#include "serial_protocol.h"
#include "Serial_CMD.h"

/* Private defines -----------------------------------------------------------*/
#define SENDER_UART  0x01
#define SENDER_USB   0x02
#define SENDER_SPI   0x03

#define DEV_ADDR                   ((uint8_t)50)
#define I2C_DATA_MAX_LENGTH_BYTES            16
#define STREAMING_MSG_LENGTH                 59
#define MIN(A,B) ((A)<(B)?(A):(B))

/* Exported variables --------------------------------------------------------*/
extern uint8_t DataLoggerActive;
extern uint32_t Sensors_Enabled;
extern TIM_HandleTypeDef CP_TimHandle;

extern void *ACCELERO_handle;
extern void *GYRO_handle;
extern void *MAGNETO_handle;
extern void *HUMIDITY_handle;
extern void *TEMPERATURE_handle;
extern void *PRESSURE_handle;
extern UART_HandleTypeDef UartHandle;

/* Exported functions ------------------------------------------------------- */
int HandleMSG(TMsg *Msg);

/* Private functions ------------------------------------------------------- */
void BUILD_REPLY_HEADER(TMsg *Msg);
void INIT_STREAMING_MSG(TMsg *Msg);
void BUILD_NACK_HEADER(TMsg *Msg);
void INIT_STREAMING_HEADER(TMsg *Msg);

#ifdef __cplusplus
}
#endif

#endif /* __DEMO_SERIAL_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

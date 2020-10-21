/**
  ******************************************************************************
  * @file    usbd_cdc_interface.c
  * @author  MEMS Application Team
  * @version V4.3.0
  * @date    26-February-2018
  * @brief   Source file for USBD CDC interface
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "vcom.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define APP_RX_DATA_SIZE  2048
#define APP_TX_DATA_SIZE  2048

/* Private macro -------------------------------------------------------------*/
/* Shared variables ----------------------------------------------------------*/
extern volatile uint8_t VCOM_RxData;
extern volatile uint8_t *VCOM_RxBuffer;
extern volatile uint32_t VCOM_RxLength;

USBD_CDC_LineCodingTypeDef LineCoding =
{
  115200, /* baud rate */
  0x00,   /* stop bits - 1 */
  0x00,   /* parity - none */
  0x08    /* nb. of bits 8 */
};

uint8_t UserRxBuffer[APP_RX_DATA_SIZE]; /* Data received via USB */
uint8_t UserTxBuffer[APP_TX_DATA_SIZE]; /* Data to be sent via USB - circular
                                           buffer: can hold at most
                                           (APP_TX_DATA_SIZE - 1) data elements */

volatile uint32_t UserTxBufPtrOut = 0; /* Points to beginning of data
                                          to be sent via USB */
                                       /* NOTE: Changed by callback function */

uint32_t UserTxBufPtrIn           = 0; /* Points to end of data
                                          to be sent via USB */

/* TIM handler declaration */
TIM_HandleTypeDef  TimHandle;

/* USB handler declaration */
extern USBD_HandleTypeDef  USBD_Device;

/* Shared function prototypes ------------------------------------------------*/
extern void Error_Handler(void);

/* Private function prototypes -----------------------------------------------*/
static int8_t CDC_Itf_Init(void);
static int8_t CDC_Itf_DeInit(void);
static int8_t CDC_Itf_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Itf_Receive(uint8_t *pbuf, uint32_t *Len);

static void CDC_TIM_Config(void);

USBD_CDC_ItfTypeDef USBD_CDC_fops =
{
  CDC_Itf_Init,
  CDC_Itf_DeInit,
  CDC_Itf_Control,
  CDC_Itf_Receive
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  CDC_Itf_Init
  *         Initializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Init(void)
{
  /* Enable TIM peripherals Clock */
  TIMx_CLK_ENABLE();

   /* Configure the NVIC for TIMx */
  /* Set Interrupt Group Priority */
  HAL_NVIC_SetPriority(TIMx_IRQn, 0x6, 0);

  /* Enable the TIMx global Interrupt */
  HAL_NVIC_EnableIRQ(TIMx_IRQn);

  /* Configure the TIM Base generation */
  CDC_TIM_Config();

  /* Start the TIM Base generation in interrupt mode - Start Channel1 */
  if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&USBD_Device, UserTxBuffer, 0);
  USBD_CDC_SetRxBuffer(&USBD_Device, UserRxBuffer);

  return (USBD_OK);
}



/**
  * @brief  CDC_Itf_DeInit
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_DeInit(void)
{
  return (USBD_OK);
}



/**
  * @brief  CDC_Itf_Control
  *         Manage the CDC class requests
  * @param  Cmd: Command code
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
  switch (cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:
    /* Add your code here */
    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:
    /* Add your code here */
    break;

  case CDC_SET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_GET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_CLEAR_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_SET_LINE_CODING:
    LineCoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |\
                            (pbuf[2] << 16) | (pbuf[3] << 24));
    LineCoding.format     = pbuf[4];
    LineCoding.paritytype = pbuf[5];
    LineCoding.datatype   = pbuf[6];
    break;

  case CDC_GET_LINE_CODING:
    pbuf[0] = (uint8_t)(LineCoding.bitrate);
    pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
    pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
    pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
    pbuf[4] = LineCoding.format;
    pbuf[5] = LineCoding.paritytype;
    pbuf[6] = LineCoding.datatype;
    break;

  case CDC_SET_CONTROL_LINE_STATE:
    /* Add your code here */
    break;

  case CDC_SEND_BREAK:
     /* Add your code here */
    break;

  default:
    break;
  }

  return (USBD_OK);
}



/**
  * @brief  Fill the USB TX buffer
  * @param  Buf: pointer to the TX buffer
  * @param  TotalLen: number of bytes to be sent
  * @param  Written: Number of really written data bytes.
  * @retval Result of the operation: USBD_OK if all operations are OK,
  *         else USBD_FAIL
  */
uint8_t CDC_Fill_Buffer(uint8_t *Buf, uint32_t TotalLen, uint32_t *Written)
{
  uint32_t i;

  for (i = 0; i < TotalLen; i++)
  {
    /* ERROR: Buffer overrun.
       NOTE: Write pointer 'UserTxBufPtrIn' points to last free position, which
       is -1 position from read pointer 'UserTxBufPtrOut' and MUST NEVER be written.
    */
    if ((UserTxBufPtrIn + 1) % APP_TX_DATA_SIZE == UserTxBufPtrOut)
    {
      *Written = i;
      return (USBD_FAIL);
    }

    UserTxBuffer[UserTxBufPtrIn] = Buf[i];
    UserTxBufPtrIn = (UserTxBufPtrIn + 1) % APP_TX_DATA_SIZE;
  }

  return (USBD_OK);
}



/**
  * @brief  Initiate next USB packet transfer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK,
  *         else USBD_FAIL
  */
uint8_t CDC_Next_Packet_Rx(void)
{

  /* VCOM data overrun - current VCOM data received are not complete read by application. */
  if (VCOM_RxData == 1)
  {
    Error_Handler();
  }

  return USBD_CDC_ReceivePacket(&USBD_Device);
}



/**
  * @brief  CDC_TIM period elapsed callback
  * @param  htim: CDC_TIM handle
  * @retval None
  */
void CDC_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t buffptr;
  uint32_t buffsize;

  if(UserTxBufPtrOut != UserTxBufPtrIn)
  {
    if(UserTxBufPtrOut > UserTxBufPtrIn) /* Rollback */
    {
      buffsize = APP_TX_DATA_SIZE - UserTxBufPtrOut;
    }
    else
    {
      buffsize = UserTxBufPtrIn - UserTxBufPtrOut;
    }

    buffptr = UserTxBufPtrOut;

    USBD_CDC_SetTxBuffer(&USBD_Device, &UserTxBuffer[buffptr], buffsize);

    if (USBD_CDC_TransmitPacket(&USBD_Device) == USBD_OK)
    {
      UserTxBufPtrOut += buffsize;
      if (UserTxBufPtrOut >= APP_TX_DATA_SIZE)
      {
        UserTxBufPtrOut = 0;
      }
    }
  }
}



/**
  * @brief  CDC_Itf_DataRx
  *         Data received over USB OUT endpoint - callback.
  * @param  Buf: Buffer of data received via USB
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Receive(uint8_t *Buf, uint32_t *Len)
{
  VCOM_RxData   = 1;
  VCOM_RxBuffer = Buf;
  VCOM_RxLength = *Len;
  return (USBD_OK);
}



/**
  * @brief  CDC_TIM_Config: Configure TIMx timer
  * @param  None.
  * @retval None.
  */
static void CDC_TIM_Config(void)
{
  /* Set TIMx instance */
  TimHandle.Instance = TIMx;

  /* Initialize TIM3 peripheral as follow:
      - Period            = 10000 - 1
      - Prescaler         = ((SystemCoreClock / 2) / 10000) - 1
      - ClockDivision     = 0
      - Counter direction = Up
  */
  TimHandle.Init.Period        = (CDC_POLLING_INTERVAL * 1000) - 1;
  TimHandle.Init.Prescaler     = 80 - 1;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;

  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

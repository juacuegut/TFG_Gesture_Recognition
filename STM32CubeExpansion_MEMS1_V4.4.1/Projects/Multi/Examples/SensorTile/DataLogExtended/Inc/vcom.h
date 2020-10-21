/**
  *******************************************************************************
  * @file    vcom.h
  * @author  MEMS Application Team
  * @version V4.3.0
  * @date    26-February-2018
  * @brief   Header for vcom.c.
  *******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
  ********************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VCOM__H
#define __VCOM__H

/* Includes ------------------------------------------------------------------*/
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"
#include "serial_protocol.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief  Serial message status values enumeration
 */
typedef enum
{
  OK,              /* Message read OK. */
  NO_DATA,         /* No data in message buffer. */
  UNEXP_END_ERR,   /* Unexpected end of message found. */
  WRONG_SEQ_ERR,   /* Wrong sequence found. */
  WRONG_CHECK_ERR, /* Wrong message checksum. */
  MSG_LEN_ERR,     /* Message length exceeded */
  UNKNOWN_ERROR    /* Unknown error. */
} vcom_msg_status_t;

/* Exported defines ----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
int VCOM_init(void);
vcom_msg_status_t VCOM_receive_MSG(TMsg *Msg, uint32_t len_max);
vcom_msg_status_t VCOM_send_MSG(TMsg *Msg);
uint32_t VCOM_read(char *buffer, uint32_t len_max);
uint32_t VCOM_write(char *buffer, uint32_t len_max);

#endif /* __VCOM__H */

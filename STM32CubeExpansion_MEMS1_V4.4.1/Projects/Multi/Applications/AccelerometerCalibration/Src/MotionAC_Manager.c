/**
  ******************************************************************************
  * @file        MotionAC_Manager.c
  * @author      MEMS Application Team
  * @version     V2.1.0
  * @date        01-November-2017
  * @brief       This file includes accelerometer calibration interface functions
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
#include "MotionAC_Manager.h"

/** @addtogroup MOTION_AC_Applications
  * @{
  */

/** @addtogroup ACCELEROMETER_CALIBRATION
  * @{
  */

/** @addtogroup AC_Driver AC_Driver
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Exported Functions --------------------------------------------------------*/
// TODO: Must be implemented for each platform separately, because its implementation is platform dependent.
//       No need to call this function, library call this function automatically
char MotionAC_LoadCalFromNVM(unsigned short int dataSize, unsigned int *data);

// TODO: Must be implemented for each platform separately, because its implementation is platform dependent.
//       No need to call this function, library call this function automatically
char MotionAC_SaveCalInNVM(unsigned short int dataSize, unsigned int *data);

/**
  * @brief  Initializes MotionAC algorithm
  * @param  enable  0 to disable library, 1 to enable library
  * @retval None
  */
void MotionAC_manager_init(MAC_enable_lib_t enable)
{
  MotionAC_Initialize((uint8_t)enable);
}

/**
  * @brief  Run accelerometer calibration algorithm
  * @param  data_in  pointer to acceleration [g] and timestamp values [ms]
  * @param  is_calibrated  pointer value returns 1 if calibration is done with current sample, 0 otherwise
  * @retval None
  */
void MotionAC_manager_update(MAC_input_t *data_in, uint8_t *is_calibrated)
{
  MotionAC_Update(data_in, is_calibrated);
}

/**
  * @brief  Run accelerometer calibration algorithm
  * @param  data_out  pointer to actual accelerometer offset [g], scale factor correction matrix [-] and quality factor
  * @retval None
  */
void MotionAC_manager_get_params(MAC_output_t *data_out)
{
  MotionAC_GetCalParams(data_out);
}

/**
  * @brief  Get the library version
  * @param  version  library version string (must be array of 35 char)
  * @retval none
  */
void MotionAC_manager_get_version(char *version, int *length)
{
  *length = MotionAC_GetLibVersion(version);
}

/**
  * @brief  Do offset & scale factor calibration
  * @param  dataIn  raw accelerometer data
  * @param  dataOut  calibrated data
  * @retval none
  */
void MotionAC_manager_compensate(SensorAxes_t* DataIn, SensorAxes_t* DataOut)
{
  MAC_output_t acc_calib;

  MotionAC_GetCalParams(&acc_calib);

  DataOut->AXIS_X = (int32_t) ((DataIn->AXIS_X - acc_bias_to_mg(acc_calib.AccBias[0])) * acc_calib.SF_Matrix[0][0]);
  DataOut->AXIS_Y = (int32_t) ((DataIn->AXIS_Y - acc_bias_to_mg(acc_calib.AccBias[1])) * acc_calib.SF_Matrix[1][1]);
  DataOut->AXIS_Z = (int32_t) ((DataIn->AXIS_Z - acc_bias_to_mg(acc_calib.AccBias[2])) * acc_calib.SF_Matrix[2][2]);
}

/**
  * @brief  Convert accelerometer bias from [g] to [mg]
  * @param  acc_bias  accelerometer bias in [g]
  * @retval accelerometer bias in [mg]
  */
int16_t acc_bias_to_mg(float acc_bias)
{
  if (acc_bias >= 0)
  {
    return (int16_t) (acc_bias * 1000.0f + 0.5f);
  }
  else
  {
    return (int16_t) (acc_bias * 1000.0f - 0.5f);
  }
}

// TODO: Must be implemented for each platform separately, because its implementation is platform dependent.
//       No need to call this function, library call this function automatically
/**
  * @brief MotionAC_LoadCalFromNVM - Load the calibration parameters from storage
  * @param dataSize  size of data
  * @param data  pointer of data
  * @retval Will return 0 the if it is success and 1 if it is failure
 */
char MotionAC_LoadCalFromNVM(unsigned short int dataSize, unsigned int *data)
{
  return 1; /* FAILURE: Read from NVM not implemented. */
}

// TODO: Must be implemented for each platform separately, because its implementation is platform dependent.
//       No need to call this function, library call this function automatically
/**
  * @brief FMotionAC_SaveCalInNVM - Save the calibration parameters in storage
  * @param dataSize  size of data
  * @param data  pointer of data
  * @retval Will return 0 the if it is success and 1 if it is failure
 */
char MotionAC_SaveCalInNVM(unsigned short int dataSize, unsigned int *data)
{
  return 1; /* FAILURE: Write to NVM not implemented. */
}


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

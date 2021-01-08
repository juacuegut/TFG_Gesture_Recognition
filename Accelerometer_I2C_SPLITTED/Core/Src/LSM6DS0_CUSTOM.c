#include "LSM6DS0_CUSTOM.h"
#include "main.h"

#include "stm32f4xx_hal_i2c.h"

//DECLARACIONES DE FUNCIONES
void Obtener_Ejes_LSM6DS0(I2C_HandleTypeDef *I2C_handler, SensorAxesFloat_t *acceleration);


//FUNCIONES
/**
 * @brief Get the LSM6DS0 accelerometer sensor axes
 * @param handle the device handle
 * @param acceleration pointer where the values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
void Obtener_Ejes_LSM6DS0(I2C_HandleTypeDef *I2C_handler, SensorAxesFloat_t *acceleration)
{
	HAL_StatusTypeDef ret;
	uint8_t buf[13];
	SensorAxes_t aux_acceleration;

	  buf[0]=REG_ACC;
	  ret = HAL_I2C_Master_Transmit(I2C_handler, LSM6DS0_ADDR, buf, 1, HAL_MAX_DELAY);
	  if(ret != HAL_OK){
		  strcpy((char*)buf,"Error Tx!!\r\n");
		  printf("Error Tx!!\r\n");
	  } else {
    ret = HAL_I2C_Master_Receive(I2C_handler, LSM6DS0_ADDR, buf, 2, HAL_MAX_DELAY);
    if(ret != HAL_OK){
   		  strcpy((char*)buf,"Error Rx!!\r\n");
   		  printf("Error Rx!!\r\n");
   	  } else {
   		 strcpy((char*)buf,"Todo OK!!\r\n");

 	  int16_t dataRaw[3];
	  float sensitivity = 0;
	  uint8_t regValue[6] = {0, 0, 0, 0, 0, 0};

	  uint8_t i, j, k;
	  uint8_t numberOfByteForDimension;

	  numberOfByteForDimension = 6 / 3;

	  k = 0;
	  for (i = 0; i < 3; i++ )
	  {
		for (j = 0; j < numberOfByteForDimension; j++ )
		{
		  //if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_OUT_X_L_XL + k, &regValue[k], 1 ))
			HAL_I2C_Mem_Read(I2C_handler, LSM6DS0_ADDR, LSM6DS0_ACC_GYRO_OUT_X_L_XL + k, I2C_MEMADD_SIZE_8BIT, &regValue[k], 1, I2C_EXPBD_Timeout);
		  k++;
		}
	  }

	  dataRaw[0] = ( ( ( ( int16_t )regValue[1] ) << 8 ) + ( int16_t )regValue[0] );
	  dataRaw[1] = ( ( ( ( int16_t )regValue[3] ) << 8 ) + ( int16_t )regValue[2] );
	  dataRaw[2] = ( ( ( ( int16_t )regValue[5] ) << 8 ) + ( int16_t )regValue[4] );

  	  //LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG6_XL, (u8_t *)value, 1)
  LSM6DS0_ACC_GYRO_FS_XL_t fullScale;
  //LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG6_XL, (u8_t *)value, 1)
  HAL_I2C_Mem_Read(I2C_handler, LSM6DS0_ADDR, LSM6DS0_ACC_GYRO_CTRL_REG6_XL, I2C_MEMADD_SIZE_8BIT, &fullScale, 1, I2C_EXPBD_Timeout);
  //*value=fullScale
  fullScale &= LSM6DS0_ACC_GYRO_FS_XL_MASK; //mask

  //El resultado es -> fullScale = LSM6DS0_ACC_GYRO_FS_XL_2g

  switch( fullScale )
    {
      case LSM6DS0_ACC_GYRO_FS_XL_2g:
        sensitivity = LSM6DS0_ACC_SENSITIVITY_FOR_FS_2G;
        break;
      case LSM6DS0_ACC_GYRO_FS_XL_4g:
        sensitivity = LSM6DS0_ACC_SENSITIVITY_FOR_FS_4G;
        break;
      case LSM6DS0_ACC_GYRO_FS_XL_8g:
        sensitivity = LSM6DS0_ACC_SENSITIVITY_FOR_FS_8G;
        break;
      case LSM6DS0_ACC_GYRO_FS_XL_16g:
        sensitivity = LSM6DS0_ACC_SENSITIVITY_FOR_FS_16G;
        break;
      default:
        sensitivity = -1.0f;
    }

  /* Calculate the data. */

  aux_acceleration.AXIS_X = ( int32_t )( dataRaw[0] * sensitivity ) ;
  aux_acceleration.AXIS_Y = ( int32_t )( dataRaw[1] * sensitivity ) ;
  aux_acceleration.AXIS_Z = ( int32_t )( dataRaw[2] * sensitivity ) ;

  acceleration->AXIS_X = ( float ) aux_acceleration.AXIS_X * CONVERT_MG_TO_MS2 ;
  acceleration->AXIS_Y = ( float ) aux_acceleration.AXIS_Y * CONVERT_MG_TO_MS2 ;
  acceleration->AXIS_Z = ( float ) aux_acceleration.AXIS_Z * CONVERT_MG_TO_MS2 ;

   	  	  }

	  	  	  }
}

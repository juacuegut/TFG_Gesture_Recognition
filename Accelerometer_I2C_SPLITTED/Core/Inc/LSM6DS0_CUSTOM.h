//INCLUIR LIBRERIAS ESTANDAR
#include <string.h>
#include <stdint.h>

//DEFINES PARA DEBUG DE ERRORES
#define REG_ACC 0x00 //buffer

//DEFINES ACELEROMETRO LSM6DS0
#define LSM6DS0_ADDR 0xD6
#define MAX_BUF_SIZE 256
#define handle 0x20000380
#define LSM6DS0_ACC_GYRO_OUT_X_L_XL 0X28
#define I2C_MEMADD_SIZE_8BIT 0x00000001U
#define I2C_EXPBD_Timeout 0x1000
#define LSM6DS0_ACC_GYRO_CTRL_REG6_XL   0X20
#define LSM6DS0_ACC_SENSITIVITY_FOR_FS_2G   0.061  /**< Sensitivity value for 2 g full scale [mg/LSB] */
#define LSM6DS0_ACC_SENSITIVITY_FOR_FS_4G   0.122  /**< Sensitivity value for 4 g full scale [mg/LSB] */
#define LSM6DS0_ACC_SENSITIVITY_FOR_FS_8G   0.244  /**< Sensitivity value for 8 g full scale [mg/LSB] */
#define LSM6DS0_ACC_SENSITIVITY_FOR_FS_16G  0.732  /**< Sensitivity value for 16 g full scale [mg/LSB] */
#define   LSM6DS0_ACC_GYRO_FS_XL_MASK   0x18


#define CONVERT_MG_TO_MS2    0.00980665f

//ESTRUCTURAS
typedef struct
{
  int32_t AXIS_X;
  int32_t AXIS_Y;
  int32_t AXIS_Z;
} SensorAxes_t;

typedef struct
{
  float AXIS_X;
  float AXIS_Y;
  float AXIS_Z;
} SensorAxesFloat_t;

typedef enum
{
  LSM6DS0_ACC_GYRO_FS_XL_2g      = 0x00,
  LSM6DS0_ACC_GYRO_FS_XL_16g     = 0x08,
  LSM6DS0_ACC_GYRO_FS_XL_4g      = 0x10,
  LSM6DS0_ACC_GYRO_FS_XL_8g      = 0x18,
} LSM6DS0_ACC_GYRO_FS_XL_t;  //SENSIBILIDAD



//FUNCIONES

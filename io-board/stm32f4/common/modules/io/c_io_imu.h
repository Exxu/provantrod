/**
  ******************************************************************************
  * @file    modules/io/c_io_imu.h
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    11-February-2014
  * @brief   Funções para IMUs (incialmente baseadas no CIs ITG3205 e ADXL345).
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_IO_IMU_H
#define C_IO_IMU_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"

#include "c_common_i2c.h"
#include "c_common_utils.h"
#include "c_common_gpio.h"

#define ARM_MATH_CM4
#include "arm_math.h"
#include <math.h>

/* FreeRTOS kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define PV_IMU_ROLL        0
#define PV_IMU_PITCH       1
#define PV_IMU_YAW         2
#define PV_IMU_DROLL       3
#define PV_IMU_DPITCH      4
#define PV_IMU_DYAW        5

#define PV_IMU_X           0
#define PV_IMU_Y           1
#define PV_IMU_Z           2

//Gravidade
#define G				   	9.81 //Ver se nao esta definindo denovo

/* Offset Giroscopio*/
#define OFFSET_GYRO_X		   	-0.051979079497908
#define OFFSET_GYRO_Y		   	-0.012014143530644
#define OFFSET_GYRO_Z			0.009635983263598

 // Accelerometer
 // "accel x,y,z (min/max) = X_MIN/X_MAX Y_MIN/Y_MAX Z_MIN/Z_MAX *1000"
 #define ACCEL_X_MIN ((float) -1074)
 #define ACCEL_X_MAX ((float) 988)
 #define ACCEL_Y_MIN ((float) -1015)
 #define ACCEL_Y_MAX ((float) 1040)
 #define ACCEL_Z_MIN ((float) -1097)
 #define ACCEL_Z_MAX ((float) 890)
 // Magnetometer (standard calibration mode)
 // "magn x,y,z (min/max) = X_MIN/X_MAX Y_MIN/Y_MAX Z_MIN/Z_MAX"
 #define MAGN_X_MIN ((float) -600)
 #define MAGN_X_MAX ((float) 600)
 #define MAGN_Y_MIN ((float) -600)
 #define MAGN_Y_MAX ((float) 600)
 #define MAGN_Z_MIN ((float) -600)
 #define MAGN_Z_MAX ((float) 600)
 // Magnetometer (extended calibration mode)
 // Uncommend to use extended magnetometer calibration (compensates hard & soft iron errors)
 //const float magn_ellipsoid_center[3] = {0, 0, 0};
 //const float magn_ellipsoid_transform[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

 // Sensor calibration scale and offset values
 #define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2000.0f)
 #define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2000.0f)
 #define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2000.0f)
 #define ACCEL_X_SCALE (G / (ACCEL_X_MAX - ACCEL_X_OFFSET))
 #define ACCEL_Y_SCALE (G / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
 #define ACCEL_Z_SCALE (G / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

 #define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
 #define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
 #define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
 #define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
 #define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
 #define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))




/* Exported macro ------------------------------------------------------------*/
#define C_IO_IMU_USE_ITG_ADXL_HMC
//#define C_IO_IMU_USE_MPU6050_HMC5883

/* Exported functions ------------------------------------------------------- */
void c_io_imu_init(I2C_TypeDef* I2Cx);
void c_io_imu_getRaw(float  * accRaw, float * gyrRaw, float * magRaw);
void c_io_imu_getComplimentaryRPY(float * acce_raw, float * gyro_raw, float * magn_raw, float sample_time, float * rpy);
void c_io_imu_getKalmanFilterRPY(float * rpy, float * acce_raw, float * gyro_raw, float * magn_raw);
void c_io_imu_initKalmanFilter();
void c_io_imu_calibrate();
void c_io_imu_Quaternion2Euler(float * q, float * rpy);
void c_io_imu_Quaternion2EulerMadgwick(float * q, float * rpy);
void c_io_imu_EulerMatrix(float * rpy, float * velAngular);

#ifdef __cplusplus
}
#endif

#endif //C_IO_IMU_H

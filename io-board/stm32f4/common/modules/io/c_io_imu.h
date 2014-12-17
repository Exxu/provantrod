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

/* Offset Giroscopio*/
//#define OFFSET_GYRO_X		   	-0.051979079497908
//#define OFFSET_GYRO_Y		   	-0.012014143530644
//#define OFFSET_GYRO_Z			0.009635983263598
//#define OFFSET_GYRO_X		   	-0.430
//#define OFFSET_GYRO_Y		   	-0.129
//#define OFFSET_GYRO_Z			0.073

#define OFFSET_GYRO_X		   	-0.054287654320988
#define OFFSET_GYRO_Y		   	-0.017202469135802
#define OFFSET_GYRO_Z			0.008808641975309

 // Accelerometer
 // "accel x,y,z (min/max) = X_MIN/X_MAX Y_MIN/Y_MAX Z_MIN/Z_MAX *1000"
// #define ACCEL_X_MIN ((float) -1.074)
// #define ACCEL_X_MAX ((float) 0.988)
// #define ACCEL_Y_MIN ((float) -1.015)
// #define ACCEL_Y_MAX ((float) 1.040)
// #define ACCEL_Z_MIN ((float) -1.097)
// #define ACCEL_Z_MAX ((float) 0.890)
#define ACCEL_X_MIN ((float) -0.996)
#define ACCEL_X_MAX ((float) 1.097)
#define ACCEL_Y_MIN ((float) -1.031)
#define ACCEL_Y_MAX ((float) 1.070)
#define ACCEL_Z_MIN ((float) -1.115)
#define ACCEL_Z_MAX ((float) 0.914)
 // Magnetometer (standard calibration mode)
 // "magn x,y,z (min/max) = X_MIN/X_MAX Y_MIN/Y_MAX Z_MIN/Z_MAX"
// #define MAGN_X_MIN ((float) -247)
// #define MAGN_X_MAX ((float) 379)
// #define MAGN_Y_MIN ((float) -461)
// #define MAGN_Y_MAX ((float) 161)
// #define MAGN_Z_MIN ((float) -380)
// #define MAGN_Z_MAX ((float) 105)

 //Magnetometer parameters for calibration
 //see:http://diydrones.com/profiles/blogs/advanced-hard-and-soft-iron-magnetometer-calibration-for-dummies?id=705844%3ABlogPost%3A1676387&page=2#comments
 #define M11 1.807
 #define M12 -0.055
 #define M13 0.052
 #define M21 0.214
 #define M22 1.926
 #define M23 0.003
 #define M31 0.02
 #define M32 -0.048
 #define M33 2.071

 #define Bx -106.511
 #define By -150.561
 #define Bz -417.946

 // Sensor calibration scale and offset values
// #define ACCEL_SENSIBILITY 256
 #define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
 #define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
 #define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
 #define ACCEL_X_SCALE (1 / (ACCEL_X_MAX - ACCEL_X_OFFSET))
 #define ACCEL_Y_SCALE (1 / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
 #define ACCEL_Z_SCALE (1 / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

// #define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
// #define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
// #define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
// #define MAGN_X_SCALE (1 / (MAGN_X_MAX - MAGN_X_OFFSET))
// #define MAGN_Y_SCALE (1 / (MAGN_Y_MAX - MAGN_Y_OFFSET))
// #define MAGN_Z_SCALE (1 / (MAGN_Z_MAX - MAGN_Z_OFFSET))

 // define to use the calibration data. If not defined then the raw values of the sensors are used
 #define CALIBRATE
// #define CALIBRATION__MAGN_USE_EXTENDED
//float mag_ellipsoid_center[3] = {79.8977, -113.117, -136.064};
//const float mag_ellipsoid_transform[3][3] = {{0.792428, -0.00418974, 0.00504922}, {-0.00418974, 0.841005, -0.0430735}, {0.00504922, -0.0430735, 0.988147}};
//arm_matrix_instance_f32 magn_ellipsoid_transform;




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
float abs2(float num);

#ifdef __cplusplus
}
#endif

#endif //C_IO_IMU_H

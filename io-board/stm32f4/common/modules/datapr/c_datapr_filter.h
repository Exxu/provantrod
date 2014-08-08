/**
  ******************************************************************************
  * @file    modules/datapr/c_datapr_filter.c
  * @author  Rodrigo Donadel
  * @version V1.0.0
  * @date    06-August-2014
  * @brief   Filtros discretos, a principio para a filtragem de sinais dos sensores.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_DATAPR_FILTER_H
#define C_DATAPR_FILTER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"
#include <math.h>


#ifdef __cplusplus
 extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

 /* Se ID for zero entao filtragem do componente somente retorna o valor passado,
 utilizado para testes*/
// #define FILTER_GYRO_BAND_BUTTER_4OD 	//4th order bandpass butterworth filter with f1=0.6 Hz and f2=6 Hz
// #define FILTER_GYRO_BAND_BUTTER_2OD     //2nd order bandpass butterworth filter with f1=0.2 Hz and f2=20 Hz
// #define FILTER_GYRO_LOW_BUTTER_1OD      //1st order lowpass butterworth filter with f=20 Hz
//   #define FILTER_GYRO_LOW_BUTTER_1OD_5HZ      //1st order lowpass butterworth filter with f=5 Hz
#define FILTER_GYRO_LOW_BUTTER_1OD_10HZ      //1st order lowpass butterworth filter with f=5 Hz
// #define FILTER_GYRO_BAND_BUTTER_6OD      //1st order lowpass butterworth filter with f=20 Hz
 #define FILTER_SONAR_ID			//TODO comentar

/* Exported macro ------------------------------------------------------------*/


/* Exported functions ------------------------------------------------------- */
 int c_datapr_filter_gyro(float *raw_gyro, float *sinal_filtrado);
 float c_datapr_filter_sonar(float raw_sonar);

#ifdef __cplusplus
}
#endif

#endif //C_IO_IMU_H

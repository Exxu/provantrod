/**
  ******************************************************************************
  * @file    modules/rc/c_rc_SFC.h
  * @author  Rodrigo Donadel
  * @version V1.0.0
  * @date    08-December-2014
  * @brief   Controle de estabilizacao por realimentacao de estados.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_RC_SFC_H
#define C_RC_SFC_H

#define ARM_MATH_CM4
#include "arm_math.h"

#include "pv_typedefs.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define STABILITY_CONTROL
//#define PATH_TRACKING_CONTROL

#ifdef STABILITY_CONTROL
 #define STATE_Z		0
 #define STATE_ROLL		1
 #define STATE_PITCH	2
 #define STATE_YAW		3
 #define STATE_DZ		4
 #define STATE_DROLL	5
 #define STATE_DPITCH	6
 #define STATE_DYAW		7
#endif

 // Saturation limits for the anti-windup
#define ROLL_LOWER_INT_ER_LIMIT		-0.1 //radians
#define ROLL_UPPER_INT_ER_LIMIT		 0.1 //radians
#define PITCH_LOWER_INT_ER_LIMIT	-0.1 //radians
#define PITCH_UPPER_INT_ER_LIMIT	 0.1 //radians
#define YAW_LOWER_INT_ER_LIMIT		-0.1 //radians
#define YAW_UPPER_INT_ER_LIMIT		 0.1 //radians
#define Z_LOWER_INT_ER_LIMIT		-0.3 //meters
#define Z_UPPER_INT_ER_LIMIT		 0.3 //meters

 //Enable the integration in relation to time
#define ENABLE_INT_Z
#define ENABLE_INT_ROLL
#define ENABLE_INT_PITCH
#define ENABLE_INT_YAW

 // Fixed Sample Time
 #define CONTROL_SAMPLE_TIME 	0.005f

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void c_rc_SFC_init();
pv_msg_io_actuation c_rc_SFC_LQR_controller(pv_msg_datapr_attitude attitude,
				  pv_msg_datapr_attitude attitude_reference,
				  pv_msg_datapr_position position,
				  pv_msg_datapr_position position_reference);

#ifdef __cplusplus
}
#endif

#endif //C_RC_SFC_H

/**
  ******************************************************************************
  * @file    modules/io/pv_module_io.h
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    02-Dezember-2013
  * @brief   Implementação do módulo de gerenciamento de sensores e atuadores.
  *
  * TODO
  *
  * \todo	1- Conferir se na descricão de pv_interface_io não deveria trocar RC por IO.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PV_MODULE_IO_H
#define PV_MODULE_IO_H

#ifdef __cplusplus
 extern "C" {
#endif


//numero de iteracoes para inicializar atuadores e sensores.
#define INIT_ITERATIONS 1000 //For a period of 5ms, it is 5 seconds

/* Includes ------------------------------------------------------------------*/

/* FreeRTOS kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* proVANT includes */
#include "c_common_gpio.h"
#include "c_common_i2c.h"
#include "c_common_uart.h"
#include "c_common_utils.h"

#include "c_io_blctrl.h"
#include "c_io_rx24f.h"
#include "c_io_imu.h"
#include "c_io_sonar.h"

//#include "c_rc_control.h"
#include "c_rc_receiver.h"
#include "c_rc_LQR_control.h"
#include "c_rc_BS_control.h"

#include "pv_typedefs.h"
#include "c_datapr_MultWii.h"

 /* Filtros Complementares */
#include "c_datapr_MahonyAHRS.h"
//#include "c_datapr_MadgwickAHRS.h"

/* Filtro */
//#include "c_datapr_filter.h"


/* Exported types ------------------------------------------------------------*/

 /** \brief Ponteiro de in- e outboxes do módulo de RC.
   *
   * Armazena e envia dados brutos ou pré-processados de sensores, e recebe
   * setpoints ou comandos para atuadores.
   */
//struct pv_interface_io {
//	xQueueHandle oAttitude;		/** Output da Orientação do VANT. Tipo pv_msg_datapr_attitude . **/
//	xQueueHandle oPosition;		/** \todo Implementar output da posição do VANT. **/
//	xQueueHandle oSensorTime;	/** Informacões de tempo relativos aos sensores. Tipo pv_msg_datapr_sensor_time . **/
//	xQueueHandle iActuation;	/** Sinais de atuação para módulo de IO. Tipo pv_msg_io_actuation . **/
//} pv_interface_io;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/



/* Exported functions ------------------------------------------------------- */
void module_io_init();
void module_io_run();

#ifdef __cplusplus
}
#endif

#endif //PV_MODULE_IO_H

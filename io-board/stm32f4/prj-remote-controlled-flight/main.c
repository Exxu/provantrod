/**
  ******************************************************************************
  * @file    main/main.c
  * @author  Martin Vincent Bloedorn & Patrick José Pereira
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   Startup do projeto.
  * @warning Modificar os arquivos 
    provant-software/io-board/stm32f4/common/modules/common/stm32f4xx_conf.h
    provant-software/io-board/stm32f4/lib/cmsis/inc/stm32f4xx.h
    provant-software/io-board/stm32f4/lib/cmsis/inc/stm32f4xx_conf.h
    dependendo da placa que esta trabalhando.
    Recompilando com:
    make allclean
    make
  *	TODO
  *
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/

/* Std includes */
#include <stdio.h>
#include <math.h>
#include "inttypes.h"

/* Hardware includes. */
#include "stm32f4xx_conf.h"
#include "system_stm32f4xx.h"
#define ARM_MATH_CM4
#include "arm_math.h"

/* FreeRTOS kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"
#include "task.h"

/* FreeRTOS+Trace Includes */
#include "trcUser.h"

/* ProVANT Modules */
#include "pv_module_rc.h"
#include "pv_module_io.h"

/* Common Components, FOR TESTING */
#include "c_common_gpio.h"
#include "c_io_blctrl.h"
#include "c_io_sonar.h"
#include "c_io_rx24f.h"

/** @addtogroup ProVANT_Modules
  * \brief Ponto de entrada do software geral do VANT.
  *
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
GPIOPin LED_builtin;
GPIOPin LED_builtin2;

/* Private define ------------------------------------------------------------*/
#ifdef STM32F4_H407
  bool BOARD_DISCOVERY=0;
  bool BOARD_H407=1;
#endif
  
#ifdef STM32F4_DISCOVERY
  bool BOARD_DISCOVERY=1;
  bool BOARD_H407=0;
#endif
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void vApplicationTickHook() {};
void vApplicationIdleHook() {};
void vApplicationStackOverflowHook() {};
void vApplicationMallocFailedHook() {};


void FPU_init(){
	/* Enable FPU.*/
	__asm("  LDR.W R0, =0xE000ED88\n"
		"  LDR R1, [R0]\n"
		"  ORR R1, R1, #(0xF << 20)\n"
		"  STR R1, [R0]");

	//#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
		SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
	//#endif
}

/* Tasks ----------------------------------------------------------------------*/

// `I'm alive` kind of task
void blink_led_task(void *pvParameters)
{
  if(BOARD_H407)
    LED_builtin = c_common_gpio_init(GPIOC, GPIO_Pin_13, GPIO_Mode_OUT);
  else
  {
    LED_builtin = c_common_gpio_init(GPIOA, GPIO_Pin_7, GPIO_Mode_OUT);
    LED_builtin2 = c_common_gpio_init(GPIOB, GPIO_Pin_1, GPIO_Mode_OUT);
    c_common_gpio_toggle(LED_builtin2);
  }
    while(1)
    {
      if(BOARD_DISCOVERY)
        c_common_gpio_toggle(LED_builtin2);
      c_common_gpio_toggle(LED_builtin);
      vTaskDelay(100/portTICK_RATE_MS);
    }
}

// Module RC task
void module_rc_task(void *pvParameters)
{
	module_rc_run();
}

// Module IO taske
void module_io_task(void *pvParameters)
{
	module_io_run();
}


void sonar_task(void *pvParameters)
{

  while(1)
  {
    char str[64];
    sprintf(str, "Distance: %f \n\r", c_io_sonar_read());
    c_common_usart_puts(USART2, str);
    vTaskDelay(300/portTICK_RATE_MS);
  }
}

/* Main ----------------------------------------------------------------------*/
int main(void)
{
	/* Init system and trace */
	SystemInit();
	FPU_init();

	vTraceInitTraceData();
	vTraceConsoleMessage("Starting application...");
	if (! uiTraceStart() )
		vTraceConsoleMessage("Could not start recorder!");

	/* Init modules */
	module_io_init(); //IO precisa ser inicializado antes de outros.
	module_rc_init();

	/* Connect modules: interface1.o* = interface2.i* */
//	pv_interface_io.oAttitude  = pv_interface_rc.iAttitude;
//	pv_interface_rc.oActuation = pv_interface_io.iActuation;
//	pv_interface_io.oSensorTime = pv_interface_rc.iSensorTime;

	c_common_usart_puts(USART2, "Iniciando!\n\r");

	/* create tasks
	 * Prioridades - quanto maior o valor, maior a prioridade
	 * */
	xTaskCreate(blink_led_task, (signed char *)"Blink led", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+1, NULL);
	//xTaskCreate(module_rc_task, (signed char *)"module_rc", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+2, NULL);
	xTaskCreate(module_io_task, (signed char *)"module_io", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+3, NULL);

	//xTaskCreate(sonar_task, (signed char *)"Sonar task", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+1, NULL);

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* should never reach here! */
	for(;;);
}

/**
  * @}
  */

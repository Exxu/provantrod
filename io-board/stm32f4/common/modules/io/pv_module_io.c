/**
  ******************************************************************************
  * @file    modules/io/pv_module_io.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    02-Dezember-2013
  * @brief   Implementação do módulo de gerenciamento de sensores e atuadores.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "pv_module_io.h"

/** @addtogroup ProVANT_Modules
  * @{
  */

/** @addtogroup Module_IO
  * \brief Componentes para atuação e sensoriamento do VANT.
  *
  * Reunião de todos os componentes relacionados às operações de I/O do VANT.
  * Leituras de todos os sensores, comandos para atuadores. O processamento destes
  * dados brutos NÃO é feito neste módulo.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MODULE_PERIOD	   5//ms

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
portTickType lastWakeTime;
//long last_IMU_time=0; /** Último valor do SysTick quando a função de leitura da IMU foi executada - para integracão numérica */
bool first_pv_io = true; // Primeira iteracao do codigo
char str[256];
int counte=0;

GPIOPin LED_builtin_io;

float attitude_quaternion[4]={1,0,0,0};
int securityStop=0; //Promove uma parada de seguranca - desliga os atuadores

/* Inboxes buffers */
pv_msg_io_actuation    iActuation;

/* Outboxes buffers*/
pv_msg_datapr_attitude oAttitude;
pv_msg_datapr_position oPosition;
pv_msg_datapr_sensor_time oSensorTime;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao componentes de IO.
  *
  * Incializa o hardware para comunicar com os sensores e atuadores. Rotinas de teste
  * ainda precisam ser executadas.
  * @param  None
  * @retval None
  */
void module_io_init() 
{
	//float accRaw[3], gyrRaw[3], magRaw[3]; // TODO Tirar daqui junto com o init

	/* Inicialização do hardware do módulo */
	LED_builtin_io = c_common_gpio_init(GPIOD, GPIO_Pin_13, GPIO_Mode_OUT);
	c_common_i2c_init(I2C3); //esc 
	c_common_i2c_init(I2C1); //imu

	c_common_usart2_init(460800);

	/* Inicializar do sonar */
	c_io_sonar_init();

	/* Inicializar os servos */
	c_io_rx24f_init(1000000);
	c_common_utils_delayms(2);
//	c_io_rx24f_setSpeed(1, 20);
//	c_io_rx24f_setSpeed(2, 20);
	c_io_rx24f_setSpeed(1, 100);
	c_io_rx24f_setSpeed(2, 100);
	c_common_utils_delayms(2);
	/* CCW Compliance Margin e CCW Compliance margin */
	c_io_rx24f_write(1, 0x1A,0x03);
	c_io_rx24f_write(1, 0x1B,0x03);
	c_io_rx24f_write(2, 0x1A,0x03);
	c_io_rx24f_write(2, 0x1B,0x03);
	c_common_utils_delayms(2);
	c_io_rx24f_move(1, 130);
	c_io_rx24f_move(2, 150);
	c_common_utils_delayms(100);

	/* Inicialização da imu */
	c_io_imu_init(I2C1);   

	/* inicializacao dos PPM */
	c_io_blctrl_init_i2c(I2C3);
	//c_io_blctrl_init_ppm();

	/* Inicialização das filas do módulo. Apenas inboxes (i*!) são criadas! */
	pv_interface_io.iActuation = xQueueCreate(1, sizeof(pv_msg_io_actuation));

	/* Inicializando outboxes em 0 */
	pv_interface_io.oAttitude = 0;
	pv_interface_io.oPosition = 0;
	pv_interface_io.oSensorTime = 0;

	/* Verificação de criação correta das filas */
	if(pv_interface_io.iActuation == 0) {
		vTraceConsoleMessage("Could not create queue in pv_interface_io!");
		while(1);
	}
}

/** \brief Caso detecte overflow dos ticks do sistema, soma 25565 TODO rever valor */
long verifyOverflow(long int deltaT){
	if (deltaT < 0)
		deltaT = deltaT + 25565; //Valor que dá overflow - REVER valor

	return deltaT;
}

/**\ brief Calcula o set point do ESC a partir da forca passada por argumento
 * Curva retirada dos ensaios com os motores brushless no INEP
 */
unsigned char setPointESC_Forca(float forca){
//	Coefficientes do polinomio
//	float p1 = -0.0013112;
//	float p2 = 0.04625;
//	float p3 = -0.50308;
//	float p4 = 1.0065;
//	float p5 = 23.443;
//	float p6 = -3.254;
//
//	if (forca <= 0.18)
//		return 1;
//	else
//		return p1*pow(forca,5) + p2*pow(forca,4) + p3*pow(forca,3) + p4*pow(forca,2) + p5*forca + p6;

//	Coefficients:
	float  p1 = 0.13104;
	float  p2 = -2.9901;
	float  p3 = 36.554;
	float  p4 = -7.945;
//forca = forca -6;
	if (forca <= 0.23)
		return (unsigned char)1;
	else
		return (unsigned char)(p1*pow(forca,3) + p2*pow(forca,2) + p3*forca + p4);

}

/** \brief Função principal do módulo de IO.
  * @param  None
  * @retval None
  *
  * Loop que amostra sensores e escreve nos atuadores como necessário.
  *
  */

//bool trigger = true;
float altitude_sonar=0;
float altitude_sonar_anterior=0;
float altitude_sonar_filtrado=0;
void module_io_run() 
{
	float accRaw[3], gyrRaw[3], magRaw[3];
	float rpy[] = {0,0,0,0,0,0};
	float velAngular[3]={0,0,0};
	int iterations=0;
	int patrick=1;
	float correcao_yaw=0;

	// Deixar global?
	pv_msg_io_actuation    actuation = {0,0.0f,0.0f,0.0f,0.0f};
	pv_msg_datapr_attitude attitude  = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
	pv_msg_datapr_attitude attitude_reference = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
	pv_msg_datapr_position position  = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
	pv_msg_datapr_position position_reference = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

	while(1)
	{
		c_common_gpio_toggle(LED_builtin_io);
		lastWakeTime = xTaskGetTickCount();

		xQueueReceive(pv_interface_io.iActuation, &iActuation, 0);

		/// IMU DATA
		#if 1
		 	c_io_imu_getRaw(accRaw, gyrRaw, magRaw);
			c_datapr_MahonyAHRSupdate(attitude_quaternion, velAngular, gyrRaw[0],gyrRaw[1],gyrRaw[2],accRaw[0],accRaw[1],accRaw[2],magRaw[0],magRaw[1],magRaw[2]);
			//c_datapr_MahonyAHRSupdate(attitude_quaternion,gyrRaw[0],gyrRaw[1],gyrRaw[2],accRaw[0],accRaw[1],accRaw[2],0,0,0);
			c_io_imu_Quaternion2Euler(attitude_quaternion, rpy);
			c_io_imu_EulerMatrix(rpy, velAngular);

			if ( (rpy[2]*RAD_TO_DEG < -160) || (rpy[2]*RAD_TO_DEG > 160) )
				securityStop=1;

			//SOMENTE PARA TESTES - mudando o yaw para o valor inicial ser 0
//			if (iterations < 500){
//				correcao_yaw = rpy[2];
//				first_pv_io = false;}
//
//			rpy[2]= rpy[2]-correcao_yaw;
		#endif


		/// SONAR
		#if 1
			//Get sonar value in cm
			altitude_sonar = c_io_sonar_read()/100;//the altitude must be in meters

			altitude_sonar_filtrado = 0.9277*altitude_sonar_filtrado + 0.03614*altitude_sonar + 0.03614*altitude_sonar_anterior;
			altitude_sonar_anterior = altitude_sonar;

			// Derivada = (dado_atual-dado_anterior )/(tempo entre medicoes)
			position.dotZ = (altitude_sonar_filtrado - position.z)/MODULE_PERIOD;
			position.z = altitude_sonar;
			position_reference.z = 0.85;
		#endif

		/// DADOS OUT
		oAttitude.roll     = rpy[PV_IMU_ROLL  ];
		oAttitude.pitch    = rpy[PV_IMU_PITCH ];
		oAttitude.yaw      = rpy[PV_IMU_YAW   ];
		oAttitude.dotRoll  = rpy[PV_IMU_DROLL ];
		oAttitude.dotPitch = rpy[PV_IMU_DPITCH];
		oAttitude.dotYaw   = rpy[PV_IMU_DYAW  ];
//		oSensorTime.IMU_sample_time = MODULE_PERIOD/1000;
		oSensorTime.IMU_sample_time = 0.005;




		#if 1
			iActuation = RC_controller(oAttitude,attitude_reference,position,position_reference,oSensorTime,1);
			// Ajusta o eixo de referencia do servo (montado ao contrario)
			iActuation.servoLeft = -iActuation.servoLeft;
		#endif

		// Parada de emergencia caso o yaw chegue perto da descontinuidade - Bota todos atuadores para zeros
		if (securityStop){
			iActuation.servoLeft=0;
			iActuation.servoRight=0;
			iActuation.escLeftSpeed=0;
			iActuation.escRightSpeed=0;}


		/// SERVOS
		#if 1

			if( (iActuation.servoRight*RAD_TO_DEG<70) && (iActuation.servoRight*RAD_TO_DEG>-70) )
				c_io_rx24f_move(1, 130+iActuation.servoRight*RAD_TO_DEG);
			if( (iActuation.servoLeft*RAD_TO_DEG<70) && (iActuation.servoLeft*RAD_TO_DEG>-70) )
				c_io_rx24f_move(2, 150+iActuation.servoLeft*RAD_TO_DEG);

		#endif


		// set points para os ESCs
		/// ESCS
			unsigned char sp_right;
			unsigned char sp_left;
			sp_right = setPointESC_Forca(iActuation.escRightSpeed);
			sp_left = setPointESC_Forca(iActuation.escLeftSpeed);
		#if 0

		/*
					iActuation.escRightSpeed = 8.0f;
					iActuation.escLeftSpeed  = 8.0f;

					if ( (iActuation.escLeftSpeed > 50))
					{
						if (trigger)
						{
							sp_right++;
							sp_left++;
						}
					trigger = false;
					}
					else
						trigger = true;


					if ( (iActuation.escLeftSpeed < -50))
					{
						if ( trigger && (sp_right > 9) && (sp_left > 9))
						{
							sp_right--;
							sp_left--;
							trigger = false;
						}
					}
		*/

//			sp_right = setPointESC_Forca(iActuation.escRightSpeed);
//			sp_left = setPointESC_Forca(iActuation.escLeftSpeed);

			//taskENTER_CRITICAL();
			// 100 iteracoes com a thread periodica de 10ms = 1segundo
			if (iterations < 500)
			{
				c_io_blctrl_setSpeed(0, 10 );
				c_common_utils_delayus(10);
				c_io_blctrl_setSpeed(1, 10 );
			}
			else
			{
//				if(!securityStop){
				c_io_blctrl_setSpeed(0, sp_right );//sp_right
				c_common_utils_delayus(10);
				c_io_blctrl_setSpeed(1, sp_left );//sp_left
				//c_io_blctrl_updateBuffer(1);
//				}
			}
			//taskEXIT_CRITICAL();
		#endif


		/// DEBUG
		#if 1
	    	// multwii
	    	#if 0

		    	c_common_datapr_multwii_bicopter_identifier();
		    	c_common_datapr_multwii_motor_pins();
		    	c_common_datapr_multwii_motor(iActuation.escLeftSpeed+2,iActuation.escRightSpeed+2);
		    	c_common_datapr_multwii_attitude(rpy[PV_IMU_ROLL  ]*RAD_TO_DEG, rpy[PV_IMU_PITCH  ]*RAD_TO_DEG, rpy[PV_IMU_YAW  ]*RAD_TO_DEG );
		    	arm_scale_f32(accRaw,RAD_TO_DEG,accRaw,3);
		    	arm_scale_f32(gyrRaw,RAD_TO_DEG,gyrRaw,3);
		    	c_common_datapr_multwii_raw_imu(accRaw,gyrRaw,magRaw);
		    	c_common_datapr_multwii_servos((iActuation.servoLeft*RAD_TO_DEG),(iActuation.servoRight*RAD_TO_DEG));
		    	c_common_datapr_multwii_debug(iActuation.escLeftSpeed,iActuation.escRightSpeed,iActuation.servoLeft*RAD_TO_DEG,iActuation.servoLeft*RAD_TO_DEG);

		    	c_common_datapr_multwii_sendstack(USART2);
	    	#else  
	    	// serial
	    	 	#if 0
				sprintf(str,
					">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n\r OUT ->S:%d \t rpy: %d \t %d \t %d \t drpy: %d \t %d \t %d \t servo: %d \t %d \t motor: %d \t %d\n\r" ,
				(int)c_io_sonar_read(), (int)(rpy[PV_IMU_ROLL  ]*RAD_TO_DEG),
				(int)(rpy[PV_IMU_PITCH  ]*RAD_TO_DEG), (int)(rpy[PV_IMU_YAW  ]*RAD_TO_DEG),
				(int)(rpy[PV_IMU_DROLL  ]*RAD_TO_DEG),(int)(rpy[PV_IMU_DPITCH  ]*RAD_TO_DEG), (int)(rpy[PV_IMU_DYAW  ]*RAD_TO_DEG),
				(int)(iActuation.servoRight*RAD_TO_DEG),(int)(iActuation.servoLeft*RAD_TO_DEG),
				(int)iActuation.escRightSpeed,(int)iActuation.escLeftSpeed);

				c_common_usart_puts(USART2, str);
				#else
				int scale=100;
				sprintf(str,"%d  %d  %d | %d  | %d %d | %d %d | %d \n\r",
				(int)(rpy[PV_IMU_ROLL  ]*RAD_TO_DEG),(int)(rpy[PV_IMU_PITCH  ]*RAD_TO_DEG), (int)(rpy[PV_IMU_YAW  ]*RAD_TO_DEG),
				(int)(altitude_sonar_filtrado*100),
				(int)sp_right,(int)sp_left,
				(int)(iActuation.servoRight*RAD_TO_DEG*scale),(int)(iActuation.servoLeft*RAD_TO_DEG*scale),
				(int)securityStop);
				c_common_usart_puts(USART2, str);
				#endif

				
			#endif
		#endif

		iterations++;

		if(pv_interface_io.oAttitude != 0){
      		xQueueOverwrite(pv_interface_io.oAttitude, &oAttitude);
      		xQueueOverwrite(pv_interface_io.oSensorTime, &oSensorTime);
		}
		c_common_gpio_toggle(LED_builtin_io);
		vTaskDelayUntil( &lastWakeTime, (MODULE_PERIOD / portTICK_RATE_MS));
	}
}
/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */




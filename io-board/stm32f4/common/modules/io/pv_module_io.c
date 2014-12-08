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
#define MODULE_PERIOD	   10//ms

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
portTickType lastWakeTime;
char str[256];
GPIOPin LED_builtin_io;
//GPIOPin pino_teste;
float attitude_quaternion[4]={1,0,0,0};
int securityStop=0; //Promove uma parada de seguranca - desliga os atuadores
int init=1; //Se 1 entao o UAV está em fase de inicializacao

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
	/* Inicialização do hardware do módulo */
	LED_builtin_io = c_common_gpio_init(GPIOD, GPIO_Pin_13, GPIO_Mode_OUT);
//	pino_teste = c_common_gpio_init(GPIOE, GPIO_Pin_13, GPIO_Mode_OUT);
	c_common_i2c_init(I2C3); //esc 
	c_common_i2c_init(I2C1); //imu

	c_common_usart2_init(460800);
	/* Inicializar do sonar */
	c_io_sonar_init();
	/* Inicializar os servos */
	c_io_rx24f_init(1000000);
	c_common_utils_delayms(2);
	c_io_rx24f_setSpeed(1, 200);
	c_io_rx24f_setSpeed(2, 200);
	c_common_utils_delayms(2);
	/* CCW Compliance Margin e CCW Compliance margin */
	c_io_rx24f_write(1, 0x1A,0x03);
	c_io_rx24f_write(1, 0x1B,0x03);
	c_io_rx24f_write(2, 0x1A,0x03);
	c_io_rx24f_write(2, 0x1B,0x03);
	c_common_utils_delayms(2);
	c_io_rx24f_move(2, 130);
	c_io_rx24f_move(1, 150);
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




/**\ brief Calcula o set point do ESC a partir da forca passada por argumento
 * Curva retirada dos ensaios com os motores brushless no INEP
 */
unsigned char setPointESC_Forca(float forca){
	//	Coefficients:
	float p1 = 0.00088809;
	float p2 = -0.039541;
	float p3 = 0.67084;
	float p4 = -5.2113;
	float p5 = 16.33;
	float p6 = 10.854;
	float p7 = 3.0802;
	float set_point=0;

	if (forca <= 0)
		return (unsigned char)1;
	else{
		set_point = (p1*pow(forca,6) + p2*pow(forca,5) + p3*pow(forca,4) + p4*pow(forca,3)
								+ p5*pow(forca,2) + p6*forca + p7);
	    if (set_point >= 255)
	    	return (unsigned char)255;
	    else
	    	return (unsigned char)set_point;}
}





/** \brief Função principal do módulo de IO.
  * @param  None
  * @retval None
  *
  * Loop que amostra sensores e escreve nos atuadores como necessário.
  *
  */
void module_io_run() 
{
	float accRaw[3], gyrRaw[3], magRaw[3];
	float rpy[] = {0,0,0,0,0,0};
	int iterations=0;
	int canalA=0;
	float canalTHROTTLE=0;
	float altitude_sonar=0.0;

	pv_msg_io_actuation    actuation = {0,0.0f,0.0f,0.0f,0.0f};
	pv_msg_datapr_attitude attitude  = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
	pv_msg_datapr_attitude attitude_reference = {0.0f,-0.0791f,0.0f,0.0f,0.0f,0.0f};
	pv_msg_datapr_position position  = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
	pv_msg_datapr_position position_reference = {0.0f,0.0f,0.8f,0.0f,0.0f,0.0f};

	while(1)
	{
//		c_common_gpio_toggle(LED_builtin_io);
		lastWakeTime = xTaskGetTickCount();

//		xQueueReceive(pv_interface_io.iActuation, &iActuation, 0);

		// Leitura canais do controle remoto
		canalA = (int)c_rc_receiver_getChannel(C_RC_CHANNEL_A);
		canalTHROTTLE = (c_rc_receiver_getChannel(C_RC_CHANNEL_THROTTLE))/100;

		// Canal A é o switch no controle remoto que deve parar totalmente o VANT, aka bota de emergencia
		if (canalA)
			securityStop = 1;

		if (iterations > INIT_ITERATIONS)
			init = 0; //Sai da fase de inicializacao

		/// IMU DATA
		#if 1
		 	c_io_imu_getRaw(accRaw, gyrRaw, magRaw);

		 	if (!init)
		 		c_datapr_setBetaOperational();

//		 	c_datapr_filter_gyro(gyrRaw, gyrFiltrado);
		 	c_datapr_MahonyAHRSupdate(attitude_quaternion, gyrRaw[0],gyrRaw[1],gyrRaw[2],accRaw[0],accRaw[1],accRaw[2],magRaw[0],magRaw[1],magRaw[2]);
//		 	c_datapr_MahonyAHRSupdate(attitude_quaternion, gyrRaw[0],gyrRaw[1],gyrRaw[2],accRaw[0],accRaw[1],accRaw[2],0,0,0);
		 	c_io_imu_Quaternion2Euler(attitude_quaternion, rpy);
			c_io_imu_EulerMatrix(rpy,gyrRaw);
//			c_io_imu_EulerMatrix(rpy,gyrFiltrado);

			// Se o yaw está perto da zona de perigo a emergencia é acionada e o birotor é desligado
			if ( (rpy[2]*RAD_TO_DEG < -160) || (rpy[2]*RAD_TO_DEG > 160) )
				securityStop=1;

		#endif


		/// SONAR
		#if 1

			//Get sonar value in cm
//			altitude_sonar = (int)c_io_sonar_read();
			altitude_sonar = c_io_sonar_read()/100;//the altitude must be in meters
			if (altitude_sonar > 1.5)
				altitude_sonar=1.5;
			//
//			// Derivada = (dado_atual-dado_anterior )/(tempo entre medicoes)
			position.dotZ = (altitude_sonar - position.z)/MODULE_PERIOD;
			position.z = altitude_sonar;
//			position_reference.z = 1.2;
		#endif

		/// DADOS OUT
		oAttitude.roll     = rpy[PV_IMU_ROLL  ];
		oAttitude.pitch    = rpy[PV_IMU_PITCH ];
		oAttitude.yaw      = rpy[PV_IMU_YAW   ];
		oAttitude.dotRoll  = rpy[PV_IMU_DROLL ];
		oAttitude.dotPitch = rpy[PV_IMU_DPITCH];
		oAttitude.dotYaw   = rpy[PV_IMU_DYAW  ];
		oSensorTime.IMU_sample_time = 0.005;

		// A referencia é a orientacao que o UAV é iniciado
		if (init){
			attitude_reference.yaw   = rpy[PV_IMU_YAW];
		}

		// CONTROLE
		#if 1
			if (init){
				iActuation.escRightSpeed = 0;
				iActuation.escLeftSpeed  = 0;
				iActuation.servoRight    = 0;
				iActuation.servoLeft     = 0;
			}
			else{
					iActuation = RC_controller(oAttitude,attitude_reference,position,position_reference,oSensorTime,1);
					// Ajusta o eixo de referencia do servo (montado ao contrario)
					iActuation.servoLeft = -iActuation.servoLeft;


				}
		#endif


		/// SERVOS
		#if 1
		// inicializacao
			if (securityStop){
				c_io_rx24f_move(1, 130+0);
				c_io_rx24f_move(2, 150+0);
			}
			else{
				if (init)
				{
						c_io_rx24f_move(1, 130+0);
						c_io_rx24f_move(2, 150+0);
				}
				else{
					if( (iActuation.servoRight*RAD_TO_DEG<70) && (iActuation.servoRight*RAD_TO_DEG>-70) )
						c_io_rx24f_move(2, 150+iActuation.servoRight*RAD_TO_DEG);
					if( (iActuation.servoLeft*RAD_TO_DEG<70) && (iActuation.servoLeft*RAD_TO_DEG>-70) )
						c_io_rx24f_move(1, 130+iActuation.servoLeft*RAD_TO_DEG);
				}
			}
		#endif


		// set points para os ESCs
		/// ESCS
			unsigned char sp_right;
			unsigned char sp_left;
			sp_right = setPointESC_Forca(iActuation.escRightSpeed*canalTHROTTLE);
			sp_left = setPointESC_Forca(iActuation.escLeftSpeed*canalTHROTTLE);
		#if 1
			if (securityStop){
				c_io_blctrl_setSpeed(1, 0 );//sp_right
				c_common_utils_delayus(10);
				c_io_blctrl_setSpeed(0, 0 );//sp_left
			}
			else{
				//inicializacao
				if (init)
				{
					c_io_blctrl_setSpeed(1, 10 );
					c_common_utils_delayus(10);
					c_io_blctrl_setSpeed(0, 10 );
				}
				else
				{
					c_io_blctrl_setSpeed(1, sp_right );//sp_right
					c_common_utils_delayus(10);
					c_io_blctrl_setSpeed(0, sp_left );//sp_left
				}
			}
		#endif


		/// DEBUG
		#if 1
	    	#if 1 // multwii
			float gyrRaw_scaled[3], accRaw_scaled[3];

	    	arm_scale_f32(gyrRaw,RAD_TO_DEG,gyrRaw_scaled,3);
	    	arm_scale_f32(accRaw,1000,accRaw_scaled,3);

	    	c_common_datapr_multwii_bicopter_identifier();
	    	c_common_datapr_multwii_motor_pins();
		    c_common_datapr_multwii_motor(sp_left,sp_right);
	    	c_common_datapr_multwii_attitude(rpy[PV_IMU_ROLL  ]*RAD_TO_DEG, rpy[PV_IMU_PITCH  ]*RAD_TO_DEG, rpy[PV_IMU_YAW  ]*RAD_TO_DEG );
	    	c_common_datapr_multwii_raw_imu(accRaw_scaled,gyrRaw_scaled,magRaw);
	    	c_common_datapr_multwii_servos((iActuation.servoLeft*RAD_TO_DEG),(iActuation.servoRight*RAD_TO_DEG));
	    	c_common_datapr_multwii_debug((int)(altitude_sonar*100),sp_right,iActuation.servoLeft*RAD_TO_DEG,iActuation.servoLeft*RAD_TO_DEG);

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
				sprintf(str,"%d  %d  %d\n\r",
				(int)(rpy[PV_IMU_ROLL  ]*RAD_TO_DEG),(int)(rpy[PV_IMU_PITCH  ]*RAD_TO_DEG), (int)(rpy[PV_IMU_YAW  ]*RAD_TO_DEG));
				c_common_usart_puts(USART2, str);
				#endif

				
			#endif
		#endif

		iterations++;

//		if(pv_interface_io.oAttitude != 0){
//      		xQueueOverwrite(pv_interface_io.oAttitude, &oAttitude);
//      		xQueueOverwrite(pv_interface_io.oSensorTime, &oSensorTime);
//		}
//		c_common_gpio_toggle(LED_builtin_io);
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




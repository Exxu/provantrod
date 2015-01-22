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
char str[256];
GPIOPin LED_builtin_io;
//GPIOPin pino_teste;
float attitude_quaternion[4]={1,0,0,0};
int securityStop=0; //Promove uma parada de seguranca - desliga os atuadores
int init=1; //Se 1 entao o UAV está em fase de inicializacao

/* Inboxes buffers */
pv_msg_io_actuation    iActuation;
//
///* Outboxes buffers*/
//pv_msg_datapr_attitude oAttitude;
//pv_msg_datapr_position oPosition;
//pv_msg_datapr_sensor_time oSensorTime;

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
	LED_builtin_io = c_common_gpio_init(GPIOD, GPIO_Pin_15, GPIO_Mode_OUT);

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
//	c_io_rx24f_setSpeed(1, 1023);//maximum speed
//	c_io_rx24f_setSpeed(2, 1023);//maximum speed
	c_common_utils_delayms(2);
	/* CW Compliance Margin e CCW Compliance margin */
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
//	pv_interface_io.iActuation = xQueueCreate(1, sizeof(pv_msg_io_actuation));

	/* Inicializando outboxes em 0 */
//	pv_interface_io.oAttitude = 0;
//	pv_interface_io.oPosition = 0;
//	pv_interface_io.oSensorTime = 0;

	/* Verificação de criação correta das filas */
//	if(pv_interface_io.iActuation == 0) {
//		vTraceConsoleMessage("Could not create queue in pv_interface_io!");
//		while(1);
//	}

	//Inicializacao do controle
	c_rc_commons_init(true);//Comeca em modo de voo manual para a altura
	c_rc_LQR_control_init();
	c_rc_BS_control_init();
}




/**\ brief Calcula o set point do ESC a partir da forca passada por argumento
 * Curva retirada dos ensaios com os motores brushless no INEP
 */
unsigned char setPointESC_Forca(float forca){
	//	Coefficients:
	float p1 = 0.00088809, p2 = -0.039541, p3 = 0.67084, p4 = -5.2113, p5 = 16.33, p6 = 10.854, p7 = 3.0802, set_point=0;

	if (forca <= 0)
		return (unsigned char) ESC_MINIMUM_VELOCITY;
	else{
		set_point = (p1*pow(forca,6) + p2*pow(forca,5) + p3*pow(forca,4) + p4*pow(forca,3)
								+ p5*pow(forca,2) + p6*forca + p7);
	    if (set_point >= 255)
	    	return (unsigned char)255;
	    else
	    	return (unsigned char)set_point;}
}



// Filter memory
float sonar_raw_k_minus_1=0.0f, sonar_raw_k_minus_2=0.0f, sonar_filtered_k_minus_1=0.0f, sonar_filtered_k_minus_2=0.0f, dotZ_filtered_k_minus_1=0.0f, dotZ_k_minus_1=0.0f;
/** \brief Função principal do módulo de IO.
  * @param  None
  * @retval None
  *
  * Loop que amostra sensores e escreve nos atuadores como necessário.
  *
  */
void module_io_run() 
{
	bool lock_increment_roll=false, lock_increment_pitch=false, lock_increment_yaw=false, enable_integration=false, lock_increment_z=false;
	float accRaw[3]={0}, gyrRaw[3]={0}, magRaw[3]={0}, rpy[6] = {0}, attitude_yaw_initial=0.0f, last_valid_sonar_raw=0.35f, position_reference_initial=0.0f;
	int channel_A=0, channel_B=0, channel_VR=0, iterations=1, channel_flight_mode=0, sample=0;
	float channel_THROTTLE=0.0f, channel_ROLL=0.0f, channel_PITCH=0.0f, channel_YAW=0.0f, sonar_raw=0.0f, sonar_raw_real=0.0f, sonar_raw_filter=0.0f, sonar_corrected_debug=0.0f, sonar_corrected=0.0f, sonar_filtered=0.0f, dotZ=0.0f, dotZ_filtered=0.0f;
//	float channel_THROTTLE_initial = 0.0f, throttle_moduler=0.0f;
	int valid_sonar_measurements=0;
	int n_valid_samples=0;


	pv_msg_io_actuation    actuation = {0,0.0f,0.0f,0.0f,0.0f};
	pv_msg_datapr_attitude attitude  = {0}, attitude_reference = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};//roll=0.03491; pitch -0.0791f
	pv_msg_datapr_position position  = {0}, position_reference = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

	while(1)
	{
		c_common_gpio_toggle(LED_builtin_io);
		lastWakeTime = xTaskGetTickCount();
		/* PARA TESTES COM OS SERVOS - DELETAR*/
//		c_io_rx24f_move(1, 130+0);
//		c_io_rx24f_move(2, 150+0);
		/*-----------------------------------------*/
//		xQueueReceive(pv_interface_io.iActuation, &iActuation, 0);

		// Leitura canais do controle remoto
		channel_A = 		(int)c_rc_receiver_getChannel(C_RC_CHANNEL_A);  //Emergency (forces the actuators to zero)
		channel_B = 		(int)c_rc_receiver_getChannel(C_RC_CHANNEL_B);  //Switch manual/automatic height control
//		channel_VR = 		(int)c_rc_receiver_getChannel(C_RC_CHANNEL_VR); //Height reference
		channel_THROTTLE =  c_rc_receiver_getChannel(C_RC_CHANNEL_THROTTLE);//Total force in Z_i axis when in manual height control

		channel_ROLL = 		c_rc_receiver_getChannel(C_RC_CHANNEL_ROLL); 	//Roll angle reference
		channel_PITCH = 	c_rc_receiver_getChannel(C_RC_CHANNEL_PITCH); 	//Pitch angle reference
		channel_flight_mode = c_rc_receiver_getChannel(C_RC_CHANNEL_YAW); 	//Yaw angle reference

		// Canal A é o switch no controle remoto que deve parar totalmente o VANT, aka botao de emergencia
		if (!channel_A && !init)
			securityStop = 1;
		else
			if (channel_A)
				securityStop = 0;


		if (iterations > INIT_ITERATIONS)
			init = 0; //Sai da fase de inicializacao

		/// IMU DATA
		#if 1
		 	c_io_imu_getRaw(accRaw, gyrRaw, magRaw);

		 	c_datapr_MahonyAHRSupdate(attitude_quaternion, gyrRaw[0],gyrRaw[1],gyrRaw[2],accRaw[0],accRaw[1],accRaw[2],magRaw[0],magRaw[1],magRaw[2]);
//		 	c_datapr_MahonyAHRSupdate(attitude_quaternion, gyrRaw[0],gyrRaw[1],gyrRaw[2],accRaw[0],accRaw[1],accRaw[2],0,0,0);
		 	c_io_imu_Quaternion2Euler(attitude_quaternion, rpy);
			c_io_imu_EulerMatrix(rpy,gyrRaw);

			// Se o yaw está perto da zona de perigo a emergencia é acionada e o birotor é desligado
			if ( (rpy[2]*RAD_TO_DEG < -160) || (rpy[2]*RAD_TO_DEG > 160) )
				securityStop=1;

		#endif


		/// SONAR
		#if 1
			float k1_1o_10Hz=0.7265, k2_1o_10Hz=0.1367, k3_1o_10Hz=0.1367;
			float k1_2o_10Hz=1.56102, k2_2o_10Hz=-0.64135, k3_2o_10Hz=0.02008, k4_2o_10Hz=0.04017, k5_2o_10Hz=0.02008;


			//Get sonar value in m
			sonar_raw_real=c_io_sonar_read();
			sonar_raw= sonar_raw_real/100;

		#ifdef LIMIT_SONAR_VAR

			// height obtained from integration the accelerometers
//			height_acc = c_datapr_filter_estimate_height_acc(accRaw, rpy);

			//if (sample<=SONAR_AVERAGE_WINDOW){
				if ( ( (position_reference.z-SONAR_MAX_VAR)<sonar_raw && (position_reference.z+SONAR_MAX_VAR)>sonar_raw ) || init){
//					last_valid_sonar_raw = sonar_raw;
					//sonar_raw_filter=sonar_raw_filter+sonar_raw;
					//n_valid_samples++;
					sonar_corrected = (sonar_raw)*cos(rpy[PV_IMU_ROLL])*cos(rpy[PV_IMU_PITCH]);//the altitude must be in meters
				}
				//sample++;
			/*}/*
			else{
				if (n_valid_samples <= 0)
					sonar_corrected= last_valid_sonar_raw;
				else{
					sonar_corrected_debug= sonar_raw_filter/n_valid_samples;
					sonar_corrected = (sonar_corrected_debug)*cos(rpy[PV_IMU_ROLL])*cos(rpy[PV_IMU_PITCH]);//the altitude must be in meters
				}

				sample=0;
				sonar_raw_filter=0;
				n_valid_samples=0;
			}*/
			#else
				sonar_corrected = sonar_raw*cos(rpy[PV_IMU_ROLL])*cos(rpy[PV_IMU_PITCH]);

		#endif



//			#ifdef LIMIT_SONAR_VAR
//			// corrects the measurement of sonar
//			if ( ( (last_valid_sonar_raw-SONAR_MAX_VAR)<sonar_raw && (last_valid_sonar_raw+SONAR_MAX_VAR)>sonar_raw ))
//				last_valid_sonar_raw = sonar_raw;
//			else
//				sonar_raw = last_valid_sonar_raw;
//			#endif
//
//			#ifdef FILTER_SONAR_100ms
//			//Measurement of sonar with the average of 20 samples
//			if (sample<=20){
//				sonar_raw_filter=sonar_raw_filter+sonar_raw;
//				sample++;
//			}else{
//				sonar_corrected_debug= sonar_raw_filter/20;
//				sonar_corrected = (sonar_corrected_debug)*cos(rpy[PV_IMU_ROLL])*cos(rpy[PV_IMU_PITCH]);//the altitude must be in meters
//				sample=0;
//				sonar_raw_filter=0;
//			}
//			#endif

//			sonar_corrected = (sonar_raw/100)*cos(rpy[PV_IMU_ROLL])*cos(rpy[PV_IMU_PITCH]);//the altitude must be in meters
//			sonar_raw = (c_io_sonar_read()/100)*cos(rpy[PV_IMU_ROLL])*cos(rpy[PV_IMU_PITCH]);//the altitude must be in meters

//			#ifdef LIMIT_SONAR_VAR
//				if (init)
//					last_valid_sonar_raw = sonar_corrected;
//
//				if ( ( (last_valid_sonar_raw-SONAR_MAX_VAR)<sonar_corrected && (last_valid_sonar_raw+SONAR_MAX_VAR)>sonar_corrected ))
//					last_valid_sonar_raw = sonar_corrected;
//				else
//					sonar_corrected = last_valid_sonar_raw;
//			#endif

			#ifdef SONAR_FILTER_1_ORDER_10HZ
				//1st order filter with fc=10Hz
				sonar_filtered = k1_1o_10Hz*sonar_filtered_k_minus_1 + k2_1o_10Hz*sonar_corrected + k3_1o_10Hz*sonar_raw_k_minus_1;
				// Filter memory
				sonar_raw_k_minus_1 = sonar_corrected;
				sonar_filtered_k_minus_1 = sonar_filtered;
			#elif defined SONAR_FILTER_2_ORDER_10HZ
				//1st order filter with fc=10Hz
				sonar_filtered = k1_2o_10Hz*sonar_filtered_k_minus_1 + k2_2o_10Hz*sonar_filtered_k_minus_2 + k3_2o_10Hz*sonar_corrected + k4_2o_10Hz*sonar_raw_k_minus_1 + k5_2o_10Hz*sonar_raw_k_minus_2;
				// Filter memory
				sonar_raw_k_minus_2 = sonar_raw_k_minus_1;
				sonar_raw_k_minus_1 = sonar_corrected;
				sonar_filtered_k_minus_2 = sonar_filtered_k_minus_1;
				sonar_filtered_k_minus_1 = sonar_filtered;
			#else //If no filter is active, the result is the measurement
				sonar_filtered = sonar_corrected;
			#endif

			// Derivada = (dado_atual-dado_anterior )/(tempo entre medicoes) - fiz a derivada do sinal filtrado, REVER
			dotZ = (sonar_filtered - position.z)/0.005;
			// 1st order filter with fc=10Hz
			dotZ_filtered = k1_1o_10Hz*dotZ_filtered_k_minus_1 + k2_1o_10Hz*dotZ + k3_1o_10Hz*dotZ_k_minus_1;
			// Filter memory
			dotZ_filtered_k_minus_1 = dotZ_filtered;
			dotZ_k_minus_1 = dotZ;

			//Filtered measurements
//			position.z = sonar_filtered*cos(rpy[PV_IMU_ROLL])*cos(rpy[PV_IMU_PITCH]);
			position.z = sonar_filtered;
			position.dotZ = dotZ_filtered;

//			if (count>=200){
//				reset_height_estimation(position.z, position.dotZ);
//				count=0;
//			}
//			else
//				count++;


			// Change to automatic height control if channel_B switch is 1
			if (channel_flight_mode){
				// Execute this condition only one time
				if (get_manual_height_control()){
//					position_reference_initial = sonar_filtered;
//					channel_THROTTLE_initial=channel_THROTTLE/100;
//					position_reference.z = sonar_filtered;
					set_manual_height_control(false);}
			}
			else{
				set_manual_height_control(true);
				position_reference.z = sonar_filtered;
			}

		#endif

			if (channel_B)
				enable_integration = true;
			else
				enable_integration = false;

		/// DADOS OUT
		if (abs2(rpy[PV_IMU_ROLL  ]-attitude.roll)>ATTITUDE_MINIMUM_STEP)
			attitude.roll     = rpy[PV_IMU_ROLL  ];
		if (abs2(rpy[PV_IMU_PITCH ]-attitude.pitch)>ATTITUDE_MINIMUM_STEP) attitude.pitch    = rpy[PV_IMU_PITCH ];
		if (abs2(rpy[PV_IMU_YAW   ]-attitude.yaw)>ATTITUDE_MINIMUM_STEP)  attitude.yaw      = rpy[PV_IMU_YAW   ];
		attitude.dotRoll  = rpy[PV_IMU_DROLL ];
		attitude.dotPitch = rpy[PV_IMU_DPITCH];
		attitude.dotYaw   = rpy[PV_IMU_DYAW  ];

		// Referencias
#ifdef ATTITUDE_REF_INCREMENTAL
		if (channel_ROLL>=90 && !lock_increment_roll){
			attitude_reference.roll += REF_ROLL_INCREMENT;
			lock_increment_roll = true;}
		else if (channel_ROLL <= -90){
			attitude_reference.roll -= REF_ROLL_INCREMENT;
			lock_increment_roll = true;}
		else
			lock_increment_roll = false;

		if (channel_PITCH>=90 && !lock_increment_pitch){
			attitude_reference.pitch += REF_PITCH_INCREMENT;
			lock_increment_pitch = true;}
		else if (channel_PITCH <= -90){
			attitude_reference.pitch -= REF_PITCH_INCREMENT;
			lock_increment_pitch = true;}
		else
			lock_increment_pitch = false;

		if (channel_YAW>=90 && !lock_increment_yaw){
			attitude_reference.yaw += REF_YAW_INCREMENT;
			lock_increment_yaw = true;}
		else if (channel_YAW <= -90){
			attitude_reference.yaw -= REF_YAW_INCREMENT;
			lock_increment_yaw = true;}
		else
			lock_increment_yaw = false;

#elif defined ATTITUDE_REF_CONTINOUS
		attitude_reference.roll     = (REF_ROLL_MAX*channel_ROLL/100)+REF_ROLL_BIAS;
		attitude_reference.pitch    = REF_PITCH_MAX*channel_PITCH/100+REF_PITCH_BIAS;
		attitude_reference.yaw      = attitude_yaw_initial;// + REF_YAW_MAX*channel_YAW/100;

//		if (!get_manual_height_control()){
//			throttle_moduler=(channel_THROTTLE/100)-channel_THROTTLE_initial;
//			position_reference.z = position_reference_initial + REF_Z_MAX*throttle_moduler;}
//			if (channel_THROTTLE>=90 && !lock_increment_z){
//				position_reference.z += REF_Z_INCREMENT;
//				lock_increment_z = true;}
//			else if (channel_THROTTLE <= -90 && !lock_increment_z){
//				position_reference.z -= REF_Z_INCREMENT;
//				lock_increment_z = true;}
//			else if (channel_THROTTLE>=-90 && channel_THROTTLE<=90)
//				lock_increment_z = false;

#endif

//		 A referencia é a orientacao que o UAV é iniciado
		if (init)
			attitude_yaw_initial	 = rpy[PV_IMU_YAW];

		if (channel_THROTTLE < 0) channel_THROTTLE = 0;

		// CONTROLE
		#if 1
			if (!init){
				#ifdef LQR_ATTITUDE_HEIGHT_CONTROL
					iActuation = c_rc_LQR_AH_controller(attitude,attitude_reference,position,position_reference,get_manual_height_control());
				#elif defined BACKSTEPPING_ATTITUDE_HEIGHT_CONTROL
					iActuation = c_rc_BS_AH_controller(attitude,attitude_reference,position,position_reference,channel_THROTTLE/100,get_manual_height_control(),enable_integration);
				#endif
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
			sp_right = setPointESC_Forca(iActuation.escRightSpeed);
			sp_left = setPointESC_Forca(iActuation.escLeftSpeed );
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
					c_io_blctrl_setSpeed(1, ESC_MINIMUM_VELOCITY );
					c_common_utils_delayus(10);
					c_io_blctrl_setSpeed(0, ESC_MINIMUM_VELOCITY );
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

			if (init){
				c_common_datapr_multwii_bicopter_identifier();
	    		c_common_datapr_multwii_motor_pins();}
			c_common_datapr_multwii_motor((int)iActuation.escLeftSpeed,(int)iActuation.escRightSpeed);
//			c_common_datapr_multwii_motor((int)(1),(int)(10));
			c_common_datapr_multwii_attitude(rpy[PV_IMU_ROLL  ]*RAD_TO_DEG*10, rpy[PV_IMU_PITCH  ]*RAD_TO_DEG*10, rpy[PV_IMU_YAW  ]*RAD_TO_DEG*10 );
//	    	c_common_datapr_multwii_raw_imu(accRaw,gyrRaw,magRaw);
	    	c_common_datapr_multwii_servos((iActuation.servoLeft*RAD_TO_DEG),(iActuation.servoRight*RAD_TO_DEG));
//	    	c_common_datapr_multwii_servos((int)(altitude_sonar*100),(int)(position.dotZ*100));
//	    	c_common_datapr_multwii_debug(channel_A, channel_B, channel_VR, channel_THROTTLE);
//	    	c_common_datapr_multwii_debug((dotZ_filtered*1000),(iActuation.servoRight*RAD_TO_DEG*10),(sonar_filtered*100), 1);
//	    	c_common_datapr_multwii_debug((int)((dotZ_filtered*1000)+100),(int)((iActuation.servoRight*RAD_TO_DEG*10)+100),(int)((sonar_filtered*100)+100),get_manual_height_control()+10);

			c_common_datapr_multwii_debug((int)(sonar_raw_real),(int)(sonar_filtered*100),(int)(attitude_reference.roll*RAD_TO_DEG*10),(int)(attitude_reference.pitch*RAD_TO_DEG*10));

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




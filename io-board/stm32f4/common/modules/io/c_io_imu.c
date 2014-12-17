/**
  ******************************************************************************
  * @file    modules/io/c_io_imu.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    11-February-2014
  * @brief   Funções para IMUs (incialmente baseadas no CIs ITG3205 e ADXL345).
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_io_imu.h"

/** @addtogroup Module_IO
  * @{
  */

/** @addtogroup Module_IO_Component_IMU
  *	\brief Componente para IMU.
  *
  *	Este componente é projetado para implementar funções da IMU da aeronave - leitura
  *	e pré-processamento. A IMU suportada é a baseada nos CIs ITG3205 e ADXL345, mas
  *	outros modelos podem ser incorporados via #define.
  *
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//#define I2Cx_imu      I2C2 // i2c of imu

#ifdef C_IO_IMU_USE_ITG_ADXL_HMC
	#define GYRO_ADDR   0x68 // The address of ITG3205
	#define ACCL_ADDR   0x53 // The address of ADXL345
	#define MAGN_ADDR   0x1E // The address of HMC5883
	#define GYRO_X_ADDR 0x1D // Start address for x-axis
	#define ACCL_X_ADDR 0x32 // Start address for x-axis
	#define MAGN_X_ADDR 0x03 // Start address for x-axis
#elif defined C_IO_IMU_USE_MPU6050_HMC5883
	#include "c_io_imu_MPU6050.h"

	#define HMC58X3_ADDR 0x1E // 7 bit address of the HMC58X3
	#define HMC_POS_BIAS 1
	#define HMC_NEG_BIAS 2

	// HMC58X3 register map. For details see HMC58X3 datasheet
	#define HMC58X3_R_CONFA 0
	#define HMC58X3_R_CONFB 1
	#define HMC58X3_R_MODE 2
	#define HMC58X3_R_XM 3
	#define HMC58X3_R_XL 4
	#define HMC58X3_R_STATUS 9
	#define HMC58X3_R_IDA 10
	#define HMC58X3_R_IDB 11
	#define HMC58X3_R_IDC 12
#else
	#error "Define an IMU type in `c_io_imu.h`! C_IO_USE_ITG_ADXL, or other!"
#endif


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
I2C_TypeDef* I2Cx_imu;
uint8_t imuBuffer[16];
unsigned char ACCL_ID = 0;
unsigned char GYRO_ID = 0;
unsigned char MAGN_ID = 0;

const float mag_ellipsoid_center[3] = {79.8977, -113.117, -136.064};
const float mag_ellipsoid_transform[3][3] = {{0.792428, -0.00418974, 0.00504922}, {-0.00418974, 0.841005, -0.0430735}, {0.00504922, -0.0430735, 0.988147}};

//calibration_matrix[3][3] is the transformation matrix
 double mag_calibration_matrix[3][3] = {{M11, M12, M13},{M21, M22, M23},{M31, M32, M33} };
 //bias[3] is the bias
 double mag_bias[3] = {Bx,By,Bz};

/* Private function prototypes -----------------------------------------------*/

//void c_io_imu_Euler2Quaternion(float * rpy, float * q);

/* Private functions ---------------------------------------------------------*/

/* Exported functions definitions --------------------------------------------*/

/** \brief Funcão que retorna o modulo do Valor. Implementada porque por motivos desconhecidos a funcão abs da lib do C não está
 * funcionando.
 * NÃO É GENËRICA! Aceita somente float.
 */
float abs2(float num){
	if (num < 0)
		return -num;
	else
		return num;
}

/** \brief Inicializa a IMU.
 *
 * Seta sensibilidade do acelerômetro e liga o girscópio.
 */
void c_io_imu_init(I2C_TypeDef* I2Cx)
{

  I2Cx_imu=I2Cx;

#ifdef C_IO_IMU_USE_ITG_ADXL_HMC // Inicialização para a IMU selecionada
  /*-----------------------Acelerometer------------------------------*/
	// Get Accelerometer ID
	c_common_i2c_readBytes(I2Cx_imu, ACCL_ADDR, 0x00, 1, &ACCL_ID);

	//  ADXL345 (Accel) pow_CTL - Set to measurement mode
	c_common_i2c_writeByte(I2Cx_imu, ACCL_ADDR, 0x2D, 8);
	// Accelerometer range and set full scale (4mG/LSB)
//	c_common_i2c_writeByte(I2Cx_imu, ACCL_ADDR, 0x31, 0x0B); //(+/- 16G)
//	c_common_i2c_writeByte(I2Cx_imu, ACCL_ADDR, 0x31, 8); //(+/- 2G)
	c_common_i2c_writeByte(I2Cx_imu, ACCL_ADDR, 0x31, 9); //(+/- 4G)

	// Accelerometer output data rate(Hz)
	c_common_i2c_writeByte(I2Cx_imu, ACCL_ADDR, 0x2C, 11); //200Hz
//	c_common_i2c_writeByte(I2Cx_imu, ACCL_ADDR, 0x2C, 12); //400Hz



  /*-----------------------Gyroscope------------------------------*/
  // Gyro ID and setup
	c_common_i2c_readBytes(I2Cx_imu, GYRO_ADDR, 0x00, 1, &GYRO_ID);
//	c_common_i2c_writeByte(I2Cx_imu, GYRO_ADDR, 0x16, 24); //24 = 0b0001 1000
	// Low Pass Filter
//	c_common_i2c_writeByte(I2Cx_imu, GYRO_ADDR, 0X16, 0x1E); //fc=5Hz
	c_common_i2c_writeByte(I2Cx_imu, GYRO_ADDR, 0X16, 0x1B); //fc=42Hz
//	c_common_i2c_writeByte(I2Cx_imu, GYRO_ADDR, 0X16, 0x1A); //fc=98Hz
	c_common_i2c_writeByte(I2Cx_imu, GYRO_ADDR, 0X16, 25); //fc=188Hz - estava usando este
//	c_common_i2c_writeByte(I2Cx_imu, GYRO_ADDR, 0X16, 0x03); //fc=256Hz

	// Output data frequency (must change if the value in register 0x16 changes to fc=256Hz)
//	c_common_i2c_writeByte(I2Cx_imu, GYRO_ADDR, 0X15, 4); //f=200Hz
	c_common_i2c_writeByte(I2Cx_imu, GYRO_ADDR, 0X15, 1); //f=500Hz
//	c_common_i2c_writeByte(I2Cx_imu, GYRO_ADDR, 0x15, 9); //f=800Hz
	// Clock selection
	c_common_i2c_writeByte(I2Cx_imu, GYRO_ADDR, 0x3E, 3); //clock to PLL gyro z axis


/*-----------------------Magnetometer------------------------------*/
  // HMC5883 (Magn) Run in continuous mode
  c_common_i2c_writeByte(I2Cx_imu, MAGN_ADDR, 0x02, 0x00);
  // configure the B register to default value of Sensor Input Field Range: 1.2Ga
  // +/- 1.2Ga <-> +/- 2047
  c_common_i2c_writeByte(I2Cx_imu, MAGN_ADDR, 0x01, 0x20);

  //75Hz (maximum)
  c_common_i2c_writeByte(I2Cx_imu, MAGN_ADDR, 0x00, 0x18);

//	#ifdef CALIBRATION__MAGN_USE_EXTENDED
//		arm_mat_init_f32(&magn_ellipsoid_transform, 3, 1, (float32_t *)magn_ellipsoid_transform_f32);
//	#endif

#endif

#ifdef C_IO_IMU_USE_MPU6050_HMC5883 //Inicialização para a IMU baseada na MPU6050
  // Clear the 'sleep' bit to start the sensor.
  c_common_i2c_writeByte(I2Cx_imu, MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0);

  // Alocar o sub i2c -> desligar o I2C Master da MPU, habilitar I2C bypass
  c_common_i2c_writeBit(I2Cx_imu, MPU6050_I2C_ADDRESS, MPU6050_USER_CTRL, MPU6050_I2C_MST_EN, 0);
  c_common_i2c_writeBit(I2Cx_imu, MPU6050_I2C_ADDRESS, MPU6050_INT_PIN_CFG, MPU6050_I2C_BYPASS_EN, 1);

  /** \todo Implementar e testar o enabling do bus secundário da MPU, para leitura do HMC.*/
  //c_common_i2c_writeByte(0x1E, 0x02, 0x00);
  //uint8_t hmcid[3];
  //c_common_i2c_readBytes(HMC58X3_ADDR, HMC58X3_R_IDA, 3, hmcid);
#endif
}

/** \brief Obtem as leituras raw do acelerômetro, giro e magnetômetro.
 *
 * O output de cada um dos sensores é escrito nos buffers passados para a função. Os buffers passados
 * devem ter tamanho mínimo 3 (tipo float), e serão escritos sempre com valores dos eixos X, Y e Z
 * de cada sensor lido.
 * Os valores retornados para cada eixo de cada sensor estão em  \f$ g \f$ para o \b acelerômetro,
 * \f$ rad/s \f$ para o \b giroscópio, e \f$ rad \f$ para o \b magnetômetro.
 *
 * @param accRaw Buffer onde serão escritos os dados do acelerômetro.
 * @param gyrRaw Buffer onde serão escritos os dados do giroscópio.
 * @param magRaw Buffer onde serão escritos os dados do magnetômetro.
 * @param sample_time Periodo de amostragem da IMU. Tempo entre a última aquisicão e a atual.
 */
void c_io_imu_getRaw(float  * accRaw, float * gyrRaw, float * magRaw) {

//#ifdef CALIBRATION__MAGN_USE_EXTENDED
//	float32_t mag_tmp_f32[3]={0}; // used as intermediate variable for matrices operations
//	arm_matrix_instance_f32 mag_tmp_matrix;
//	arm_matrix_instance_f32 magRaw_matrix;
//
//	arm_mat_init_f32(&mag_tmp_matrix, 3, 1, (float32_t *)mag_tmp_f32);
//	arm_mat_init_f32(&magRaw_matrix, 3, 1, (float32_t *)magRaw);
//
//#endif

float mag_tmp[3]={0};

#ifdef C_IO_IMU_USE_ITG_ADXL_HMC
    // Read x, y, z acceleration, pack the data.
    uint8_t  buffer[14]={};
    float accScale =0.00390625f; // 1/256
    /*All g-ranges, full resolution - typical sensitivity = 256 LSB/g*/

#ifdef ENABLE_PREEMPT
	taskENTER_CRITICAL();
#endif
  	c_common_i2c_readBytes(I2Cx_imu, ACCL_ADDR, ACCL_X_ADDR, 6, imuBuffer);
#ifdef ENABLE_PREEMPT
	taskEXIT_CRITICAL();
#endif
  	// Para transformar em valores no SI -> acc/256 *G m/s^2
    accRaw[0] = (int16_t)(imuBuffer[0] | (imuBuffer[1] << 8))*accScale;
    accRaw[1] = (int16_t)(imuBuffer[2] | (imuBuffer[3] << 8))*accScale;
    accRaw[2] = (int16_t)(imuBuffer[4] | (imuBuffer[5] << 8))*accScale;
//	accRaw[1] = ((((int16_t) imuBuffer[3]) << 8) | imuBuffer[2])*accScale; // X axis (internal sensor y axis)
//	accRaw[0] = ((((int16_t) imuBuffer[1]) << 8) | imuBuffer[0])*accScale; // Y axis (internal sensor x axis)
//	accRaw[2] = ((((int16_t) imuBuffer[5]) << 8) | imuBuffer[4])*accScale; // Z axis (internal sensor z axis)

    // Read x, y, z from gyro, pack the data

    /** A sensitividade do giroscopio da ITG3200 é dada pela tabela (extraída do datasheet):
     FS_SEL | Full Scale Range | LSB Sensitivity
    --------|------------------|----------------
    0       | Reservado        | Reservado
    1       | Reservado        | Reservado
    2       | Reservado        | Reservado
    3       | 2,000°/seg       | 14.375 LSBs °/S
    ***********************************************/
    
    float gyrScale =14.375f;
    gyrScale = 0.0174532925f/gyrScale;//0.0174532925 = PI/180

#ifdef ENABLE_PREEMPT
	taskENTER_CRITICAL();
#endif
  	c_common_i2c_readBytes(I2Cx_imu, GYRO_ADDR, GYRO_X_ADDR, 6, imuBuffer);
#ifdef ENABLE_PREEMPT
	taskEXIT_CRITICAL();
#endif
  	gyrRaw[0] =  (int16_t)((imuBuffer[1] | (imuBuffer[0] << 8)))*gyrScale;
  	gyrRaw[1] =  (int16_t)((imuBuffer[3] | (imuBuffer[2] << 8)))*gyrScale;
  	gyrRaw[2] =  (int16_t)((imuBuffer[5] | (imuBuffer[4] << 8)))*gyrScale;


    // Read x, y, z from magnetometer;
#ifdef ENABLE_PREEMPT
	taskENTER_CRITICAL();
#endif
    c_common_i2c_readBytes(I2Cx_imu, MAGN_ADDR, MAGN_X_ADDR, 6, imuBuffer);
#ifdef ENABLE_PREEMPT
	taskEXIT_CRITICAL();
#endif

    magRaw[0] =  (int16_t)((imuBuffer[1] | (imuBuffer[0] << 8)));// X
    magRaw[1] =  (int16_t)((imuBuffer[5] | (imuBuffer[4] << 8)));// Y
    magRaw[2] =  (int16_t)((imuBuffer[3] | (imuBuffer[2] << 8)));// Z
    
    /** Como dito no link:  http://www.multiwii.com/forum/viewtopic.php?f=8&t=1387&p=10658
    * temosque encontrar os zeros do mag

         X   |     Y    |     Z
    ---------|----------|----------------
    -196/607 | -488/250 | -422/263
    ***********************************************/
    
//    magRaw[1] = (magRaw[1]-(250-488)/2)/3.69;
//    magRaw[0] = (magRaw[0]-(607-196)/2)/4.015;
//    magRaw[2] = (magRaw[2]-(263-422)/2)/3.425;

  #ifdef CALIBRATE
    // Compensate accelerometer error
    accRaw[0] = (accRaw[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
    accRaw[1] = (accRaw[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
    accRaw[2] = (accRaw[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;
    // Compensate magnetometer error
    #ifdef CALIBRATION__MAGN_USE_EXTENDED
		for (int i = 0; i < 3; i++)
			mag_tmp[i] = magRaw[i] - mag_ellipsoid_center[i];
//		Matrix_Vector_Multiply(magn_ellipsoid_transform, mag_tmp, magRaw);
		magRaw[0] = mag_ellipsoid_transform[0][0]*mag_tmp[0] + mag_ellipsoid_transform[0][1]*mag_tmp[1] + mag_ellipsoid_transform[0][2]*mag_tmp[2];
		magRaw[1] = mag_ellipsoid_transform[1][0]*mag_tmp[0] + mag_ellipsoid_transform[1][1]*mag_tmp[1] + mag_ellipsoid_transform[1][2]*mag_tmp[2];
		magRaw[2] = mag_ellipsoid_transform[2][0]*mag_tmp[0] + mag_ellipsoid_transform[2][1]*mag_tmp[1] + mag_ellipsoid_transform[2][2]*mag_tmp[2];
    #else
	 for (int i=0; i<3; ++i) magRaw[i] = magRaw[i] - mag_bias[i];

	 float result[3] = {0, 0, 0};
	 	 for (int i=0; i<3; ++i)
	 		 for (int j=0; j<3; ++j)
	 			 result[i] += mag_calibration_matrix[i][j] * magRaw[j];

	 //calibrated values
	 for (int i=0; i<3; ++i) magRaw[i] = result[i];
    #endif
    // Compensate gyroscope error
    gyrRaw[0] -= OFFSET_GYRO_X;
    gyrRaw[1] -= OFFSET_GYRO_Y;
    gyrRaw[2] -= OFFSET_GYRO_Z;
  #endif

#endif

#ifdef C_IO_IMU_USE_MPU6050_HMC5883
    uint8_t  buffer[14];
    c_common_i2c_readBytes(I2Cx_imu, MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_H, 14, buffer);

    /** A sensitividade do acelerômetro da MPU6050 é dada pela tabela (extraída do datasheet):
    AFS_SEL | Full Scale Range | LSB Sensitivity
    --------|------------------|----------------
    0       | ±2g              | 16384 LSB/g
    1       | ±4g              | 8192 LSB/g
    2       | ±8g              | 4096 LSB/g
    3       | ±16g             | 2048 LSB/g
    ***********************************************/
    float accScale = 16384.0f;

    accRaw[0] = -1.0f*(float)((((signed char)buffer[0]) << 8) | ((uint8_t)buffer[1] & 0xFF))/accScale;
    accRaw[1] = -1.0f*(float)((((signed char)buffer[2]) << 8) | ((uint8_t)buffer[3] & 0xFF))/accScale;
    accRaw[2] =       (float)((((signed char)buffer[4]) << 8) | ((uint8_t)buffer[5] & 0xFF))/accScale;

    /** A sensitividade do giroscópio da MPU6050 é dada pela tabela (extraída do datasheet):
    FS_SEL | Full Scale Range | LSB Sensitivity
    -------|------------------|----------------
    0      | ± 250 °/s        | 131 LSB/°/s
    1      | ± 500 °/s        | 65.5 LSB/°/s
    2      | ± 1000 °/s       | 32.8 LSB/°/s
    3      | ± 2000 °/s       | 16.4 LSB/°/s
    ***********************************************/
    float gyrScale = 131.0f;

    gyrRaw[0] = (float)((((signed char)buffer[8])  << 8) | ((uint8_t)buffer[9]  & 0xFF))/gyrScale;
    gyrRaw[1] = (float)((((signed char)buffer[10]) << 8) | ((uint8_t)buffer[11] & 0xFF))/gyrScale;
    gyrRaw[2] = (float)((((signed char)buffer[12]) << 8) | ((uint8_t)buffer[13] & 0xFF))/gyrScale;
#endif

}





/** \brief Calibra a IMU considerando o veículo em repouso.
 *
 *
 */
void c_io_imu_calibrate() {

}

/** \brief Calcula a taxa de variacao dos angulos de Euler a partir da velocidade angular representada no frame do corpo
 *
 */
void c_io_imu_EulerMatrix(float * rpy, float * velAngular){

// Calculando a taxa de variacao dos angulos de Euler a partir da medicao dos gyros
  rpy[PV_IMU_DROLL ] = velAngular[PV_IMU_ROLL] +
		  	  	  	  	  	  	  velAngular[PV_IMU_PITCH ]*sin(rpy[PV_IMU_ROLL ])*tan(rpy[PV_IMU_PITCH ]) +
		  	  	  	  	  	  	  velAngular[PV_IMU_YAW ]*cos(rpy[PV_IMU_ROLL ])*tan(rpy[PV_IMU_PITCH ]);

  rpy[PV_IMU_DPITCH ] = velAngular[PV_IMU_PITCH ]*cos(rpy[PV_IMU_ROLL ]) - velAngular[PV_IMU_YAW ]*sin(rpy[PV_IMU_ROLL ]);

  rpy[PV_IMU_DYAW ] = velAngular[PV_IMU_PITCH ]*sin(rpy[PV_IMU_ROLL ])/cos(rpy[PV_IMU_PITCH ]) +
		  	  	  	  	  	  	  velAngular[PV_IMU_YAW ]*cos(rpy[PV_IMU_ROLL ])/cos(rpy[PV_IMU_PITCH ]);
}

/**\brief Quaternion para Euler
 * Solucao adaptada de http://www.sedris.org/wg8home/Documents/WG80485.pdf, "Technical Concepts Orientation,
 * Rotation, Velocity and Acceleration and the SRM" escrito por Paul Berner, página 39
 *
 * @param quaternion
 * @param angulo euler convencao ZYX
 */
void c_io_imu_Quaternion2Euler(float * q, float * rpy){

	float sqx = q[1]*q[1];
    float sqy = q[2]*q[2];
    float sqz = q[3]*q[3];
	float singularity_check = (q[1]*q[3] - q[0]*q[2]);

	if (singularity_check > 0.49) { // singularity at north pole
		rpy[0] = 0;
		rpy[1] = - PI/2;
		rpy[2] = 2 * atan2(q[1],q[0]);
		return;
	}
	if (singularity_check < -0.49) { // singularity at south pole
		rpy[0] = 0;
		rpy[1] =  PI/2;
		rpy[2] = -2 * atan2(q[1],q[0]);
		return;
	}

	rpy[0] = atan2(q[2]*q[3]+q[0]*q[1] , 0.5 - sqx - sqy);
	rpy[1] = asin(-2*singularity_check);
	rpy[2] = atan2(q[1]*q[2]+q[0]*q[3] , 0.5  - sqy - sqz);
}

//void c_io_imu_Euler2Quaternion(float * rpy, float * q){
//	q[0] = cos(rpy[0]/2)*cos(rpy[1]/2)*cos(rpy[2]/2) + sin(rpy[0]/2)*sin(rpy[1]/2)*sin(rpy[2]/2);
//	q[1] = sin(rpy[0]/2)*cos(rpy[1]/2)*cos(rpy[2]/2) - cos(rpy[0]/2)*sin(rpy[1]/2)*sin(rpy[2]/2);
//	q[2] = cos(rpy[0]/2)*sin(rpy[1]/2)*cos(rpy[2]/2) + sin(rpy[0]/2)*cos(rpy[1]/2)*sin(rpy[2]/2);
//	q[3] = cos(rpy[0]/2)*cos(rpy[1]/2)*sin(rpy[2]/2) - sin(rpy[0]/2)*sin(rpy[1]/2)*cos(rpy[2]/2);
//}


/**\brief Quaternion para Euler
 * Solucao adaptada de http://www.x-io.co.uk/res/doc/madgwick_internal_report.pdf, "An efficient orientation filter for inertial and
inertial/magnetic sensor arrays" escrito por Sebastian O.H. Madgwick, página 6, equacoes (7)(8)(9).
 *
 * @param quaternion
 * @param angulo euler
 */
void c_io_imu_Quaternion2EulerMadgwick(float * q, float * rpy){

	float singularity_check = 2*(q[1]*q[3] + q[0]*q[2]);

	if (singularity_check > 0.499) { // singularity at north pole
		rpy[0] = 0;
		rpy[1] = - PI/2;
		rpy[2] = 2 * atan2(q[1],q[0]);
		return;
	}
	if (singularity_check < -0.499) { // singularity at south pole
		rpy[0] = 0;
		rpy[1] =  PI/2;
		rpy[2] = -2 * atan2(q[1],q[0]);
		return;
	}

	rpy[0]=atan2(2*(q[2]*q[3] - q[0]*q[1]) , 2*(q[0]*q[0] + q[3]*q[3] -0.5));
	rpy[1]=-asin(singularity_check);
	rpy[2]=atan2(2*(q[1]*q[2] -q[0]*q[3]) , 2*(q[0]*q[0] +q[1]*q[1] -0.5));
}
/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */


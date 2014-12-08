/**
  ******************************************************************************
  * @file    modules/datapr/c_datapr_filter.c
  * @author  Rodrigo Donadel
  * @version V1.0.0
  * @date    06-August-2014
  * @brief   Filtros discretos, a principio para a filtragem de sinais dos sensores.
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_datapr_filter.h"

/** @addtogroup Module_IO
  * @{
  */

/** @addtogroup Module_IO_Filter
  *	\brief Filtragem discreta para os perifericos.
  *
  *	Este componente é projetado para implementar funções de filtragem para os perifericos do VANT.
  * Nem todos componentes serão filtrados.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//************** Memoria para o filtro do Giroscopio ************************
float filtered_gyro_memory_X[2]={0};
float filtered_gyro_memory_Y[2]={0};
float filtered_gyro_memory_Z[2]={0};

float raw_gyro_memory_X[2]={0};
float raw_gyro_memory_Y[2]={0};
float raw_gyro_memory_Z[2]={0};
//***************************************************************************

//************** Memoria para o filtro do acelerometro ************************
float filtered_acc_memory_X[2]={0};
float filtered_acc_memory_Y[2]={0};
float filtered_acc_memory_Z[2]={0};

float raw_acc_memory_X[2]={0};
float raw_acc_memory_Y[2]={0};
float raw_acc_memory_Z[2]={0};
//***************************************************************************

//************** Memoria para o filtro do Sonar ************************
float filtered_sonar_memory[2]={0};
float raw_sonar_memory[2]={0};
//***************************************************************************

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
//num = 0.006412193744950		0	-0.012824387489899	0	0.006412193744950
//
//den = 1.000000000000000  -3.754353653929880   5.296176152097898  -3.328509382430800  0.786698128410561
/* Exported functions definitions --------------------------------------------*/
//        0.003363 z^8 - 0.01345 z^6 + 0.02018 z^4 - 0.01345 z^2 + 0.003363
//
//  ------------------------------------------------------------------------------
//
//  z^8 - 6.399 z^7 + 18.06 z^6 - 29.4 z^5 + 30.25 z^4 - 20.15 z^3 + 8.488 z^2
//
//                                                              - 2.067 z + 0.2228
int c_datapr_filter_gyro(float *raw_gyro, float *sinal_filtrado){

//	float sinal_filtrado[3]={0}; //resultado do filtro


	#if defined FILTER_GYRO_LOW_BUTTER_1OD
		//Eixo X
		sinal_filtrado[0] = 0.50954*filtered_gyro_memory_X[0] +0.24523*raw_gyro_memory_X[0] +0.24523*raw_gyro[0];
		filtered_gyro_memory_X[0]=sinal_filtrado[0];
		raw_gyro_memory_X[0]=raw_gyro[0];

		//Eixo Y
		sinal_filtrado[1] = 0.50954*filtered_gyro_memory_X[1] +0.24523*raw_gyro_memory_X[1] +0.24523*raw_gyro[1];
		filtered_gyro_memory_X[1]=sinal_filtrado[1];
		raw_gyro_memory_X[1]=raw_gyro[1];

		//Eixo Z
		sinal_filtrado[2] = 0.50954*filtered_gyro_memory_X[2] +0.24523*raw_gyro_memory_X[2] +0.24523*raw_gyro[2];
		filtered_gyro_memory_X[2]=sinal_filtrado[2];
		raw_gyro_memory_X[2]=raw_gyro[2];

	#elif defined FILTER_GYRO_LOW_BUTTER_1OD_5HZ
		//Eixo X
		sinal_filtrado[0] = 0.854080685463467*filtered_gyro_memory_X[0] +0.072959657268267*raw_gyro_memory_X[0] +0.072959657268267*raw_gyro[0];
		filtered_gyro_memory_X[0]=sinal_filtrado[0];
		raw_gyro_memory_X[0]=raw_gyro[0];

		//Eixo Y
		sinal_filtrado[1] = 0.854080685463467*filtered_gyro_memory_Y[0] +0.072959657268267*raw_gyro_memory_Y[0] +0.072959657268267*raw_gyro[1];
		filtered_gyro_memory_Y[0]=sinal_filtrado[1];
		raw_gyro_memory_Y[0]=raw_gyro[1];

		//Eixo Z
		sinal_filtrado[2] = 0.854080685463467*filtered_gyro_memory_Z[0] +0.072959657268267*raw_gyro_memory_Z[0] +0.072959657268267*raw_gyro[2];
		filtered_gyro_memory_Z[0]=sinal_filtrado[2];
		raw_gyro_memory_Z[0]=raw_gyro[2];

	#elif defined FILTER_GYRO_LOW_BUTTER_1OD_10HZ
		//Eixo X
		sinal_filtrado[0] = 0.726542528005361*filtered_gyro_memory_X[0] +0.136728735997319*raw_gyro_memory_X[0] +0.136728735997319*raw_gyro[0];
		filtered_gyro_memory_X[0]=sinal_filtrado[0];
		raw_gyro_memory_X[0]=raw_gyro[0];

		//Eixo Y
		sinal_filtrado[1] = 0.726542528005361*filtered_gyro_memory_X[1] +0.136728735997319*raw_gyro_memory_X[1] +0.136728735997319*raw_gyro[1];
		filtered_gyro_memory_X[1]=sinal_filtrado[1];
		raw_gyro_memory_X[1]=raw_gyro[1];

		//Eixo Z
		sinal_filtrado[2] = 0.726542528005361*filtered_gyro_memory_X[2] +0.136728735997319*raw_gyro_memory_X[2] +0.136728735997319*raw_gyro[2];
		filtered_gyro_memory_X[2]=sinal_filtrado[2];
		raw_gyro_memory_X[2]=raw_gyro[2];

	#elif defined FILTER_GYRO_BAND_BUTTER_6OD

//		y = 6.398795086770444*filtered_gyro_memory[0] -18.056234987997165*filtered_gyro_memory[1] +29.398420877873452*filtered_gyro_memory[2]
//		    -30.245469352207913*filtered_gyro_memory[3] +20.147981514639788*filtered_gyro_memory[4] -8.487696789614214*filtered_gyro_memory[5]
//            +2.067013962137406*filtered_gyro_memory[6] -0.222811572920135*filtered_gyro_memory[7] -0.013451260515926*raw_gyro_memory[1]
//            +0.020176890773888*raw_gyro_memory[3] -0.013451260515926*raw_gyro_memory[5] +0.003362815128981*raw_gyro_memory[7] +0.003362815128981*raw_gyro;

		//Eixo X
sinal_filtrado[0] = 6.398795086770444*filtered_gyro_memory_X[0] -18.056234987997165*filtered_gyro_memory_X[1] +29.398420877873452*filtered_gyro_memory_X[2]
			-30.245469352207913*filtered_gyro_memory_X[3] +20.147981514639788*filtered_gyro_memory_X[4] -8.487696789614214*filtered_gyro_memory_X[5]
			+2.067013962137406*filtered_gyro_memory_X[6] -0.222811572920135*filtered_gyro_memory_X[7] -0.013451260515926*raw_gyro_memory_X[1]
			+0.020176890773888*raw_gyro_memory_X[3] -0.013451260515926*raw_gyro_memory_X[5] +0.003362815128981*raw_gyro_memory_X[7]
            +0.003362815128981*raw_gyro[0];

		filtered_gyro_memory_X[7]=filtered_gyro_memory_X[6];
		filtered_gyro_memory_X[6]=filtered_gyro_memory_X[5];
		filtered_gyro_memory_X[5]=filtered_gyro_memory_X[4];
		filtered_gyro_memory_X[4]=filtered_gyro_memory_X[3];
		filtered_gyro_memory_X[3]=filtered_gyro_memory_X[2];
		filtered_gyro_memory_X[2]=filtered_gyro_memory_X[1];
		filtered_gyro_memory_X[1]=filtered_gyro_memory_X[0];
		filtered_gyro_memory_X[0]=sinal_filtrado[0];

		raw_gyro_memory_X[7]=raw_gyro_memory_X[6];
		raw_gyro_memory_X[6]=raw_gyro_memory_X[5];
		raw_gyro_memory_X[5]=raw_gyro_memory_X[4];
		raw_gyro_memory_X[4]=raw_gyro_memory_X[3];
		raw_gyro_memory_X[3]=raw_gyro_memory_X[2];
		raw_gyro_memory_X[2]=raw_gyro_memory_X[1];
		raw_gyro_memory_X[1]=raw_gyro_memory_X[0];
		raw_gyro_memory_X[0]=raw_gyro[0];


		//Eixo Y
sinal_filtrado[1] = 6.398795086770444*filtered_gyro_memory_Y[0] -18.056234987997165*filtered_gyro_memory_Y[1] +29.398420877873452*filtered_gyro_memory_Y[2]
			-30.245469352207913*filtered_gyro_memory_Y[3] +20.147981514639788*filtered_gyro_memory_Y[4] -8.487696789614214*filtered_gyro_memory_Y[5]
			+2.067013962137406*filtered_gyro_memory_Y[6] -0.222811572920135*filtered_gyro_memory_Y[7] -0.013451260515926*raw_gyro_memory_Y[1]
			+0.020176890773888*raw_gyro_memory_Y[3] -0.013451260515926*raw_gyro_memory_Y[5] +0.003362815128981*raw_gyro_memory_Y[7]
            +0.003362815128981*raw_gyro[1];

		filtered_gyro_memory_Y[7]=filtered_gyro_memory_Y[6];
		filtered_gyro_memory_Y[6]=filtered_gyro_memory_Y[5];
		filtered_gyro_memory_Y[5]=filtered_gyro_memory_Y[4];
		filtered_gyro_memory_Y[4]=filtered_gyro_memory_Y[3];
		filtered_gyro_memory_Y[3]=filtered_gyro_memory_Y[2];
		filtered_gyro_memory_Y[2]=filtered_gyro_memory_Y[1];
		filtered_gyro_memory_Y[1]=filtered_gyro_memory_Y[0];
		filtered_gyro_memory_Y[0]=sinal_filtrado[1];

		raw_gyro_memory_Y[7]=raw_gyro_memory_Y[6];
		raw_gyro_memory_Y[6]=raw_gyro_memory_Y[5];
		raw_gyro_memory_Y[5]=raw_gyro_memory_Y[4];
		raw_gyro_memory_Y[4]=raw_gyro_memory_Y[3];
		raw_gyro_memory_Y[3]=raw_gyro_memory_Y[2];
		raw_gyro_memory_Y[2]=raw_gyro_memory_Y[1];
		raw_gyro_memory_Y[1]=raw_gyro_memory_Y[0];
		raw_gyro_memory_Y[0]=raw_gyro[1];


		//Eixo Z
sinal_filtrado[2] = 6.398795086770444*filtered_gyro_memory_Z[0] -18.056234987997165*filtered_gyro_memory_Z[1] +29.398420877873452*filtered_gyro_memory_Z[2]
					-30.245469352207913*filtered_gyro_memory_Z[3] +20.147981514639788*filtered_gyro_memory_Z[4] -8.487696789614214*filtered_gyro_memory_Z[5]
					+2.067013962137406*filtered_gyro_memory_Z[6] -0.222811572920135*filtered_gyro_memory_Z[7] -0.013451260515926*raw_gyro_memory_Z[1]
					+0.020176890773888*raw_gyro_memory_Z[3] -0.013451260515926*raw_gyro_memory_Z[5] +0.003362815128981*raw_gyro_memory_Z[7]
		            +0.003362815128981*raw_gyro[2];

				filtered_gyro_memory_Z[7]=filtered_gyro_memory_Z[6];
				filtered_gyro_memory_Z[6]=filtered_gyro_memory_Z[5];
				filtered_gyro_memory_Z[5]=filtered_gyro_memory_Z[4];
				filtered_gyro_memory_Z[4]=filtered_gyro_memory_Z[3];
				filtered_gyro_memory_Z[3]=filtered_gyro_memory_Z[2];
				filtered_gyro_memory_Z[2]=filtered_gyro_memory_Z[1];
				filtered_gyro_memory_Z[1]=filtered_gyro_memory_Z[0];
				filtered_gyro_memory_Z[0]=sinal_filtrado[2];

				raw_gyro_memory_Z[7]=raw_gyro_memory_Z[6];
				raw_gyro_memory_Z[6]=raw_gyro_memory_Z[5];
				raw_gyro_memory_Z[5]=raw_gyro_memory_Z[4];
				raw_gyro_memory_Z[4]=raw_gyro_memory_Z[3];
				raw_gyro_memory_Z[3]=raw_gyro_memory_Z[2];
				raw_gyro_memory_Z[2]=raw_gyro_memory_Z[1];
				raw_gyro_memory_Z[1]=raw_gyro_memory_Z[0];
				raw_gyro_memory_Z[0]=raw_gyro[2];

	#else
		y=raw_gyro;

	#endif

//	return y;
	return 0;
}


int c_datapr_filter_acc(float *raw_acc, float *sinal_filtrado){

//	float sinal_filtrado[3]={0}; //resultado do filtro

//	 0.005543 s^2 + 0.01109 s + 0.005543
//	  -----------------------------------
//	        s^2 - 1.779 s + 0.8008
	#if defined FILTER_ACC_LOW_BUTTER_2OD_5HZ
	//Eixo X
	sinal_filtrado[0] = 1.778631777824585*filtered_acc_memory_X[0] - 0.800802646665708*filtered_acc_memory_X[1] + 0.005542717210281*raw_acc_memory_X[1]+
			+  0.011085434420561*raw_acc_memory_X[0] + 0.005542717210281*raw_acc[0];

	filtered_acc_memory_X[1]=filtered_acc_memory_X[0];
	filtered_acc_memory_X[0]=sinal_filtrado[0];

	raw_acc_memory_X[1]=raw_acc_memory_X[0];
	raw_acc_memory_X[0]=raw_acc[0];

//	//Eixo Y
//	sinal_filtrado[1] = 1.561018075800718*filtered_acc_memory_Y[0] - 0.641351538057563*filtered_acc_memory_Y[1] + 0.020083365564211*raw_acc_memory_Y[1]+
//			+ 0.040166731128423*raw_acc_memory_Y[0] + 0.020083365564211*raw_acc[1];
//	filtered_acc_memory_Y[1]=filtered_acc_memory_Y[0];
//	filtered_acc_memory_Y[0]=sinal_filtrado[1];
//
//	raw_acc_memory_Y[1]=raw_acc_memory_Y[0];
//	raw_acc_memory_Y[0]=raw_acc[1];
//
//	//Eixo Z
//	sinal_filtrado[2] = 1.561018075800718*filtered_acc_memory_Z[0] - 0.641351538057563*filtered_acc_memory_Z[1] + 0.020083365564211*raw_acc_memory_Z[1]+
//			+ 0.040166731128423*raw_acc_memory_Z[0] + 0.020083365564211*raw_acc[2];
//	filtered_acc_memory_Z[1]=filtered_acc_memory_Z[0];
//	filtered_acc_memory_Z[0]=sinal_filtrado[2];
//
//	raw_acc_memory_Z[1]=raw_acc_memory_Z[0];
//	raw_acc_memory_Z[0]=raw_acc[2];
	sinal_filtrado[1]=raw_acc[1];
	sinal_filtrado[2]=raw_acc[2];

	#elif defined FILTER_ACC_BAND_BUTTER_2OD_5HZ
		return 0;

	#else
		sinal_filtrado[0]=raw_acc[0];
		sinal_filtrado[1]=raw_acc[1];
		sinal_filtrado[2]=raw_acc[2];

	#endif

	//	return y;
	return 0;
}


float c_datapr_filter_sonar(float raw_sonar){
	return 0;
}


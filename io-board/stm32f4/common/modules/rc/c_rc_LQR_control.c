/**
  ******************************************************************************
  * @file    modules/rc/c_rc_SFC.c
  * @author  Rodrigo Donadel
  * @version V1.0.0
  * @date    08-December-2014
  * @brief   Controle de estabilizacao por realimentacao de estados.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_rc_LQR_control.h"

/** @addtogroup Module_RC
  * @{
  */

/** @addtogroup Module_RC_Component_State_Feedback_Control
  * \brief Controle de estabilização para o modo de operação RC por realimentacao de estados.
  *
   * @{
  */

		//---------------------------------------------------------------------------------------------

/* Exported functions definitions --------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

c_rc_stability_error LQR_AH_integrated_error={0};
c_rc_stability_error LQR_AH_last_error={0};

// Matrizes de ganho
//float32_t LQR_AH_Ke_f32[4][8]={
//{21.207062455586751, -24.707408251121784,   0.069346468761083,   2.452130918218125,  12.216788626480916,  -4.011929550716840,  0.004475317798479,   0.557055453783041},
//{21.233444539785776,  24.686653404459395,  -0.003353700697260,  -2.445515972875630,  12.233162353241495,   4.011111755550325,  0.005166788138636,  -0.555396333501935},
//{-0.000719994314509,   0.171143370645070,   0.344766866536217,   2.043259478401853,  -0.000386473018984,   0.031030138533026,  0.348169765415673,   0.506637304021608},
//{-0.000523964029942,  -0.167138107269209,   0.375284925986611,  -2.029180987056317,  -0.000281018829228,  -0.030178402624864,  0.345138150637528,  -0.506235008031713}
//};
//float32_t LQR_AH_Ki_f32[4][8]={
//{-18.360708418541652,  74.747291643686452,  -0.082091356993229,  -5.151947191333180},
//{-18.381627935970801, -74.663218767292690,   0.029590624926112,   5.137988576748046},
//{0.000669511614153,   -0.467900642908488,   -4.063031942899512,  -4.032256152392106},
//{0.000487812904700,   0.459532421289280,    -4.048507830710655,   4.047414146624359}
//};
//float32_t LQR_AH_Ke_f32[4][8]={
//    {7.2736,   -7.8155,    0.0283,   -0.3765,    5.7900,   -1.6030,    0.0056,   -0.1355},
//    {7.2715,    7.7557,    0.0077,    0.3739,    5.7898,    1.5958,    0.0041,    0.1348},
//    {-0.0047,   -0.4258,    4.5543,    3.4167,   -0.0033,   -0.0408,    0.9586,    1.0033},
//    {-0.0049,    0.4375,    4.5827,   -3.3797,   -0.0035,    0.0430,    0.9544,   -1.0018}
//};
float32_t LQR_AH_Ki_f32[4][4]={
   {-4.5685,   10.2205,   -0.0282,    0.5649},
   {-4.5660,  -10.1532,   -0.0040,   -0.5610},
   {0.0034,    0.4810,   -9.6865,   -5.2854},
   {0.0035,   -0.4932,   -9.6756,    5.2928}
};


// LQR devagar e menos ruidos
//float32_t LQR_AH_Ke_f32[4][8]={
//{4.6045,-1.7371,0.043385,-0.02557,3.7475,-1.0197,0.0058769,-0.031419},
//{4.5783,1.6911,-0.029769,0.024721,3.7285,1.0063,0.0011543,0.031007},
//{-0.00046913,-0.0063996,0.69711,0.78315,-0.00026476,-0.0017255,0.34916,0.42652},
//{-0.00049931,0.02299,0.90799,-1.4934,-0.000279,0.005537,0.35102,-0.8221} };

//// LQR +-1s e mais ruidos
float32_t LQR_AH_Ke_f32[4][8]={
{4.5946,-7.9015,0.10525,-0.05663,3.7402,-1.5115,0.0093342,-0.054388},
{4.588,7.7454,-0.090913,0.055196,3.7356,1.4899,-0.002182,0.053597},
{-0.00029445,-0.036064,1.9302,0.59573,-0.00016939,-0.0016446,0.61088,0.32161},
{-0.00026628,0.14971,2.7227,-1.5509,-0.00014443,0.012367,0.64269,-0.85586} };
//
//// LQR balanceado
//float32_t LQR_AH_Ke_f32[4][8]={
//{4.5946,-7.9017,0.15527,-0.054149,3.7402,-1.5115,0.011316,-0.052972},
//{4.588,7.7456,-0.13993,0.052754,3.7356,1.4899,-0.0041599,0.052203},
//{-0.00022268,-0.016759,1.3091,0.40231,-0.00012909,9.3277e-07,0.50978,0.21664},
//{-0.00019948,0.15371,2.4372,-1.6355,-0.00010648,0.012418,0.55003,-0.90126} };


float32_t equilibrium_point_f32[8]={0, 0, -0.079091, 0, 0, 0, 0, 0};
//float32_t equilibrium_point_f32[8]={0, 0.8913f,-0.0791f,-1.123453f, 0, 0, 0, 0};
float32_t equilibrium_control_f32[8]={8.5294, 8.5524, 0.079189, 0.078994};
float32_t state_vector_f32[8]={0};
float32_t error_state_vector_f32[8]={0};
float32_t PD_control_output_f32[4]={0};
float32_t I_control_output_f32[4]={0};
float32_t PID_control_output_f32[4]={0};
float32_t control_output_f32[4]={0};
float32_t  integrated_error_states_f32[4]={0};

arm_matrix_instance_f32 equilibrium_control;
arm_matrix_instance_f32 LQR_AH_Ke;
arm_matrix_instance_f32 LQR_AH_Ki;


/* Private function prototypes -----------------------------------------------*/
arm_matrix_instance_f32 c_rc_LQR_AH_errorStateVector(pv_msg_datapr_attitude attitude, pv_msg_datapr_position position, pv_msg_datapr_position position_reference);
arm_matrix_instance_f32 c_rc_LQR_AH_PD(arm_matrix_instance_f32 error_state_vector);
arm_matrix_instance_f32 c_rc_LQR_AH_I(c_rc_stability_error error);
void c_rc_LQR_AH_integrate_error(c_rc_stability_error current_error, float sample_time);

/* Private functions ---------------------------------------------------------*/

void c_rc_LQR_AH_integrate_error(c_rc_stability_error error, float sample_time){

	#ifdef ENABLE_INT_ROLL
		LQR_AH_integrated_error.roll = c_rc_saturation( c_rc_integrate_trapezoidal(LQR_AH_integrated_error.roll, error.roll,
				LQR_AH_last_error.roll, sample_time), INT_ROLL_LOWER_ER_LIMIT, INT_ROLL_UPPER_ER_LIMIT);

		LQR_AH_last_error.roll  = error.roll;
	#endif
	#ifdef ENABLE_INT_PITCH
		LQR_AH_integrated_error.pitch = c_rc_saturation( c_rc_integrate_trapezoidal(LQR_AH_integrated_error.pitch, error.pitch,
				LQR_AH_last_error.pitch, sample_time), INT_PITCH_LOWER_ER_LIMIT, INT_PITCH_UPPER_ER_LIMIT);

		LQR_AH_last_error.pitch = error.pitch;
	#endif
	#ifdef ENABLE_INT_YAW
		LQR_AH_integrated_error.yaw = c_rc_saturation( c_rc_integrate_trapezoidal(LQR_AH_integrated_error.yaw, error.yaw,
				LQR_AH_last_error.yaw, sample_time), INT_YAW_LOWER_ER_LIMIT, INT_YAW_UPPER_ER_LIMIT);

		LQR_AH_last_error.yaw   = error.yaw;
	#endif
	#ifdef ENABLE_INT_Z
		LQR_AH_integrated_error.z = c_rc_saturation( c_rc_integrate_trapezoidal(LQR_AH_integrated_error.z, error.z,
				LQR_AH_last_error.z, sample_time), INT_Z_LOWER_ER_LIMIT, INT_Z_UPPER_ER_LIMIT);

		LQR_AH_last_error.z   = error.z;
	#endif
}



arm_matrix_instance_f32 c_rc_LQR_AH_errorStateVector(pv_msg_datapr_attitude attitude, pv_msg_datapr_position position,
													pv_msg_datapr_position position_reference){

	arm_matrix_instance_f32 error_state_vector, state_vector, equilibrium_point;

	//State Vector
	state_vector_f32[STATE_Z]=position.z;
	state_vector_f32[STATE_ROLL]=attitude.roll;
	state_vector_f32[STATE_PITCH]=attitude.pitch;
	state_vector_f32[STATE_YAW]=attitude.yaw;
	state_vector_f32[STATE_DZ]=position.dotZ;
	state_vector_f32[STATE_DROLL]=attitude.dotRoll;
	state_vector_f32[STATE_DPITCH]=attitude.dotPitch;
	state_vector_f32[STATE_DYAW]=attitude.dotYaw;
	//Updates the height equilibrium point according to the reference
	equilibrium_point_f32[STATE_Z]= position_reference.z;
	equilibrium_point_f32[STATE_DZ]= position_reference.dotZ;
	//Initializes the matrices
	arm_mat_init_f32(&equilibrium_point, 8, 1, (float32_t *)equilibrium_point_f32);
	arm_mat_init_f32(&state_vector, 8, 1, (float32_t *)state_vector_f32);
	arm_mat_init_f32(&error_state_vector, 8, 1, (float32_t *)error_state_vector_f32);
	//e(t)=x(t)- equilibrium_point
	arm_mat_sub_f32(&state_vector, &equilibrium_point, &error_state_vector);

	return error_state_vector;
}



arm_matrix_instance_f32 c_rc_LQR_AH_PD(arm_matrix_instance_f32 error_state_vector){

	arm_matrix_instance_f32 PD_control_output;


	//Initializing result matrices
	arm_mat_init_f32(&PD_control_output, 4, 1, (float32_t *)PD_control_output_f32);
	//u=-Ke*e(t)
	arm_mat_mult_f32(&LQR_AH_Ke, &error_state_vector, &PD_control_output);

	return PD_control_output;
}



arm_matrix_instance_f32 c_rc_LQR_AH_I(c_rc_stability_error error){

	arm_matrix_instance_f32 I_control_output, integrated_error_states;


	//Initializing result matrice
	arm_mat_init_f32(&I_control_output, 4, 1, (float32_t *)I_control_output_f32);
	// Integrate the error
	c_rc_LQR_AH_integrate_error(error, CONTROL_SAMPLE_TIME);
	//Create the integrated error vector
	integrated_error_states_f32[STATE_ROLL] =	LQR_AH_integrated_error.roll;
	integrated_error_states_f32[STATE_PITCH]= 	LQR_AH_integrated_error.pitch;
	integrated_error_states_f32[STATE_YAW]=	 	LQR_AH_integrated_error.yaw;
	integrated_error_states_f32[STATE_Z]=	 	LQR_AH_integrated_error.z;
	arm_mat_init_f32(&integrated_error_states, 4, 1, (float32_t *)integrated_error_states_f32);
	//u=Ki*e(t)
	arm_mat_mult_f32(&LQR_AH_Ki, &integrated_error_states, &I_control_output);

	return I_control_output;
}



/* Exported functions definitions --------------------------------------------*/

/** \brief Inicilização do controle de estabilidade.
 *
 * O controlador utiliza a API de DSP da CMSIS, e portanto se baseia fortemente no uso do
 * tipo arm_matrix_instance_f32. Esta \b struct contêm os valores de número de linhas e
 * colunas de matriz, além de um ponteiro para seus elementos (na forma de array).
 * Estes arrays são prealocados globalmente (ver código fonte), para evitar overhead
 * de alocação dinâmica em cada chamada e para evitar que, a cada alocação em uma função, a memória para
 * a qual o ponteiro aponta saia de escopo e seja deletada. Uma vez que as funções são privadas e chamadas
 * em ordem determinística, mutexes não são implementadas (por simplicidade apenas)
 */
void c_rc_LQR_control_init() {

	// Inicializa as matrizes estaticas
	arm_mat_init_f32(&equilibrium_control, 4, 1, (float32_t *)equilibrium_control_f32);
	arm_mat_init_f32(&LQR_AH_Ke, 4, 8, (float32_t *)LQR_AH_Ke_f32);
	arm_mat_init_f32(&LQR_AH_Ki, 4, 4, (float32_t *)LQR_AH_Ki_f32);
}



/* \brief LQR Controller.
 *
 * Implemented based on the article "Back-stepping Control Strategy for Stabilization of a Tilt-rotor UAV" by Chowdhury, A. B., with some modifications.
 * It implements an estabilization controller and an altitude controller. It is meant to be used with the radio controller.
 * The struct pv_msg_io_attitude includes the angular velocity.
 */
pv_msg_io_actuation c_rc_LQR_AH_controller(pv_msg_datapr_attitude attitude,
				  pv_msg_datapr_attitude attitude_reference,
				  pv_msg_datapr_position position,
				  pv_msg_datapr_position position_reference){

	pv_msg_io_actuation actuation_signals;
	arm_matrix_instance_f32 error_state_vector, PD_control, I_control, PID_control, control_output;
	c_rc_stability_error error;


	//Initialize result matrices
	arm_mat_init_f32(&PID_control, 4, 1, (float32_t *)PID_control_output_f32);
	arm_mat_init_f32(&control_output, 4, 1, (float32_t *)control_output_f32);

	//e(t)=x(t)- equilibrium_point
	error_state_vector = c_rc_LQR_AH_errorStateVector(attitude, position, position_reference);
	//u_p+u_d=-Ke*e(t)
	PD_control = c_rc_LQR_AH_PD(error_state_vector);
	//u_i=-Ki*integral(e(t))
	//Select variables to integrate( same as ref - C*x(t) )
	error.roll=	 attitude_reference.roll-attitude.roll;
	error.pitch= attitude_reference.pitch-attitude.pitch;
	error.yaw=	 attitude_reference.yaw-attitude.yaw;
	error.z=	 position_reference.z-position.z;
	I_control = c_rc_LQR_AH_I(error);
	//u_pid=u_pd+u_i;
	arm_mat_add_f32(&PD_control, &I_control, &PID_control);
	//u(t)=u_pid+u_r
	arm_mat_sub_f32(&equilibrium_control, &PID_control, &control_output);
	//The result must be in a struct pv_msg_io_actuation
	actuation_signals.escRightSpeed= control_output.pData[0];
	actuation_signals.escLeftSpeed=	 control_output.pData[1];
	actuation_signals.servoRight=	 control_output.pData[2];
	actuation_signals.servoLeft=	 control_output.pData[3];
    //Declares that the servos will use angle control, rather than torque control
	actuation_signals.servoTorqueControlEnable = 0;

	return actuation_signals;
}

/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */


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


/* FUNCIONA
//float32_t LQR_AH_Ke_f32[4][8]={
//{7.040976297565402,-5.529229324124633,0.04702986610932462,0.00561489028574513,4.784116494421191,-1.332157888195032,0.005292439859508952,-0.02523119114941602},
//{7.023281552066317,5.419188201888098,-0.02459662937025143,-0.006002093781574897,4.773689056743583,1.315620257865842,0.003355595972752019,0.0250416504329464},
//{-0.0004314539919496955,-0.0007248419157975509,0.3780011800473317,0.3253239953618016,-0.0001687353920980406,0.001400164361242898,0.2610936031770633,0.2357439212021502},
//{-0.0002960525605633278,0.003329929148075135,0.4435165326275957,-0.3155488988119409,-7.79051206386452e-05,-0.0008715492959148382,0.2602342879604763,-0.2349755509548509} };

// Nao cai em z
float32_t LQR_AH_Ke_f32[4][8]={
{19.82173770584356,-5.523651353446152,0.05866688264124353,0.00564133999333163,7.234964837881615,-1.329843868740137,0.007169008173166387,-0.02516009206056138},
{19.74520768684976,5.424748073263217,-0.01298617337269339,-0.005975614604587052,7.218179044500586,1.31792983805772,0.005230615335537397,0.02511266610930479},
{-0.002392520325043178,-0.0007256210578553606,0.3780000314265149,0.3253239974809817,-0.000385735662679856,0.001400013475887998,0.2610936017512977,0.2357439188514519},
{-0.001861239041877276,0.003329361430359374,0.4435157807019197,-0.3155488964100273,-0.0002299143300308964,-0.0008716420190671401,0.2602343237067323,-0.2349755517349952} };
*/

//  Melhor ate agora, roll um pouco ruim
//float32_t LQR_AH_Ke_f32[4][8]={
//{14.04186696524969,-5.526354564328385,0.05345624408392692,0.005631053538878648,6.246768625543134,-1.330754976936697,0.006474139444955942,-0.02518685650946107},
//{13.99683161418621,5.422054591114527,-0.01818362967759337,-0.005985920959936275,6.233051661681607,1.317020713427478,0.004536139811159716,0.02508593104815383},
//{-0.001322618019066036,-0.0007251946868122222,0.3780006388234145,0.3253239961985965,-0.0002734198612438831,0.001400088459640222,0.2610935934506699,0.2357439198381478},
//{-0.0009901608028203023,0.003329678424405977,0.4435161892474985,-0.3155488977741001,-0.0001482115700462263,-0.00087159380209768,0.2602342999753556,-0.2349755514330892} };

// Mentira, esse é o melhor LQR :)
//float32_t LQR_AH_Ke_f32[4][8]={
//{14.02577777219334,-10.67302771101574,0.04284178809200095,-0.03196142272696709,6.240562907855309,-2.155834514035699,0.005686313649390949,-0.0557366891268529},
//{14.01269005282396,10.47231597547145,-0.007743097229952396,0.03092362736657574,6.239177445701783,2.127268613636207,0.005316980068840059,0.05510842988328495},
//{-0.001304310740079771,-0.03812349237943729,0.378024293325187,0.3252064219514601,-0.0002635211919329557,-0.003339767758877333,0.2610977588171128,0.2356684011756946},
//{-0.001002386412604174,0.0406471375369083,0.4434956274126712,-0.3154304780452504,-0.0001553768338996878,0.003904593443683988,0.2602312053407402,-0.2348975490079081} };

//Pitch mais rapido

//float32_t LQR_AH_Ke_f32[4][8]={
//{14.02577960829001,-10.67235785134619,0.04140413150853862,-0.03609575737717224,6.240563543696757,-2.155736950344561,0.00569853040903995,-0.0586601386219219},
//{14.0126878348504,10.47165953261245,-0.006538947132632808,0.03498612398151896,6.239176713476921,2.127173041225789,0.005324660838075028,0.05797841739632231},
//{-0.001447528409069795,-0.04409541969486132,0.4366453474895148,0.3454360475989844,-0.0002903797150479569,-0.004106867829529327,0.278202256262893,0.249722204656236},
//{-0.001138132529510704,0.04676186202243832,0.5009775313447895,-0.3348393035068429,-0.0001798923456837326,0.004704545829124088,0.2771796991427579,-0.2488896535076065} };

//Pitch mais rapido
//float32_t LQR_AH_Ke_f32[4][8]={
//{14.02578151441314,-10.6715488685709,0.03980074835493128,-0.04076637843986088,6.240564205359655,-2.155620730815042,0.005711674819979066,-0.06196411029100388},
//{14.01268544992879,10.4708666536548,-0.005203226499064257,0.03957556343381151,6.239175930378262,2.127059180057068,0.005336199822425629,0.06122172912068639},
//{-0.001630971820575172,-0.05159927931418744,0.5138083452481149,0.3701478040858032,-0.0003246821785532389,-0.005068605411505696,0.2991788667387266,0.2668837437589616},
//{-0.001313581856101325,0.05443508145386049,0.5766955504415486,-0.3585353300708185,-0.0002117142676134747,0.005705423143078923,0.2979872768257196,-0.2659773374157722} };

//Pitch mais rapido
float32_t LQR_AH_Ke_f32[4][8]={
{14.02578348210831,-10.67055128744823,0.03798522260330547,-0.04613057429481957,6.240564890336407,-2.15547939574062,0.005725911859277205,-0.06576022205311377},
{14.01268286509862,10.46988882049192,-0.003698524523851303,0.04484654574488396,6.239175088196358,2.126920697964934,0.005353550136059286,0.06494785408264969},
{-0.001875333430842356,-0.06135044159459858,0.6199376760635573,0.4013241338134781,-0.000370218793769218,-0.006315329185710952,0.3257670608329968,0.2885273599412381},
{-0.001549424750277614,0.06439176425174957,0.6809460466756787,-0.3884144222228286,-0.0002546475363179105,0.006999929604126315,0.3243909941599943,-0.2875297642940329} };

float32_t equilibrium_point_f32[8]={0, 0, -0.079091, 0, 0, 0, 0, 0};
//float32_t equilibrium_point_f32[8]={0, 0.8913f,-0.0791f,-1.123453f, 0, 0, 0, 0};
//float32_t equilibrium_control_f32[8]={8.5294, 8.5524, 0.079189, 0.078994};
float32_t equilibrium_control_f32[8]={0.113152E2,0.113459E2,0.79091E-1,0.79091E-1};
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
arm_matrix_instance_f32 c_rc_LQR_AH_errorStateVector(pv_msg_datapr_attitude attitude, pv_msg_datapr_attitude attitude_reference, pv_msg_datapr_position position, pv_msg_datapr_position position_reference);
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



arm_matrix_instance_f32 c_rc_LQR_AH_errorStateVector(pv_msg_datapr_attitude attitude, pv_msg_datapr_attitude attitude_reference,
														pv_msg_datapr_position position, pv_msg_datapr_position position_reference){

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
	equilibrium_point_f32[STATE_ROLL]= attitude_reference.roll;
	equilibrium_point_f32[STATE_PITCH]= attitude_reference.pitch;

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
const float height_takeoff_a=-1.2;
const float height_takeoff_b=-0.5;
const float height_takeoff_c=1.2;
float total_time_height_control=0;
pv_msg_io_actuation c_rc_LQR_AH_controller(pv_msg_datapr_attitude attitude,
				  pv_msg_datapr_attitude attitude_reference,
				  pv_msg_datapr_position position,
				  pv_msg_datapr_position position_reference,
				  float throttle_control,
				  bool manual_height_control){

	pv_msg_io_actuation actuation_signals;
	arm_matrix_instance_f32 error_state_vector, PD_control, I_control, PID_control, control_output;
	c_rc_stability_error error;
	float temp_height_takeoff=0;


	//Initialize result matrices
	arm_mat_init_f32(&PID_control, 4, 1, (float32_t *)PID_control_output_f32);
	arm_mat_init_f32(&control_output, 4, 1, (float32_t *)control_output_f32);

	//Updates the height equilibrium point according to the reference
	#ifdef ENABLE_TAKEOFF_PROCEDURE
		if(!manual_height_control){
			temp_height_takeoff=height_takeoff_a*exp(height_takeoff_b*total_time_height_control);
			position_reference.z=temp_height_takeoff+height_takeoff_c;
			position_reference.dotZ=height_takeoff_b*temp_height_takeoff;

			total_time_height_control += CONTROL_SAMPLE_TIME;}
		else
			total_time_height_control=0;
	#endif

	//e(t)=x(t)- equilibrium_point
	#ifdef ENABLE_RC_HEIGHT_REFERENCE
		position_reference.z=throttle_control*HEIGHT_REFERENCE_MAX;
	#endif


	error_state_vector = c_rc_LQR_AH_errorStateVector(attitude, attitude_reference, position, position_reference);

	//u_p+u_d=-Ke*e(t)
	PD_control = c_rc_LQR_AH_PD(error_state_vector);
	//u_i=-Ki*integral(e(t))
	//Select variables to integrate( same as ref - C*x(t) )
//	error.roll=	 attitude_reference.roll-attitude.roll;
//	error.pitch= attitude_reference.pitch-attitude.pitch;
//	error.yaw=	 attitude_reference.yaw-attitude.yaw;
//	error.z=	 position_reference.z-position.z;
//	I_control = c_rc_LQR_AH_I(error);
	//u_pid=u_pd+u_i;
//	arm_mat_add_f32(&PD_control, &I_control, &PID_control);
	//u(t)=u_pid+u_r
//	arm_mat_sub_f32(&equilibrium_control, &PID_control, &control_output);
	arm_mat_sub_f32(&equilibrium_control, &PD_control, &control_output);
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


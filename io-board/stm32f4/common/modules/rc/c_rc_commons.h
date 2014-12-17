/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_RC_COMMONS_H
#define C_RC_COMMONS_H

//#include "arm_math.h"
#include "pv_typedefs.h"

#ifdef __cplusplus
 extern "C" {
#endif
/* Exported macros ------------------------------------------------------------*/

// Choose the controller
//#define LQR_ATTITUDE_HEIGHT_CONTROL
#define BACKSTEPPING_ATTITUDE_HEIGHT_CONTROL //Based on Chowdhurry's article
//#define LQR_PATHTRACK_CONTROL  //To be implemented
//#define HINF_PATHTRACK_CONTROL //To be implemented
//#define HMIX_PATHTRACK_CONTROL //To be implemented


#if defined LQR_ATTITUDE_HEIGHT_CONTROL || defined BACKSTEPPING_ATTITUDE_HEIGHT_CONTROL
 #define STABILITY_CONTROL
#elif defined LQR_PATHTRACK_CONTROL || defined HINF_PATHTRACK_CONTROL || defined HMIX_PATHTRACK_CONTROL
 #define PATH_TRACKING_CONTROL
#endif


// PID parameters for the backstepping control
//#ifdef BACKSTEPPING_ATTITUDE_HEIGHT_CONTROL
#if 1
	 #define KPPHI    	20.0f
	 #define KVPHI    	8.0f
	 #define KIPHI	 	0.0f
	 #define KPTHETA  	25.0f
	 #define KVTHETA  	8.0f
	 #define KITHETA	0.0f
	 #define KPPSI    	18.0f
	 #define KVPSI    	6.0f
	 #define KIPSI	 	0.0f
	 #define KVZ		6.0f
	 #define KPZ		18.0f
  #else
	#define KPPHI    	86.0f
	#define KVPHI    	10.0f
	#define KIPHI	 	3.0f
	#define KPTHETA  	86.0f
	#define KVTHETA  	10.0f
	#define KITHETA		3.0f
	#define KPPSI    	18.0f
	#define KVPSI    	6.0f
	#define KIPSI	 	0.0f
	#define KVZ			6.0f
	#define KPZ			18.0f
#endif
//#endif

//Maximum value for the thrust when in manual height mode
#define THRUST_ROTOR_MAX			14.2f   // Newtons. It is the maximum thrust that can be appliesd to a rotor
#define THRUST_FLIGHT_THRESHOLD		(M*G-14.0f)	// Newtons. The thrust needed for the aircraft to almost take flight
#define THRUST_MAX_MANUAL			(2.0f*THRUST_ROTOR_MAX-THRUST_FLIGHT_THRESHOLD)	// Newtons. It is the total thrust applied by the brushless motors combined

 //keeping here to keep it practical, move it to sensors after test
#define ESC_MINIMUM_VELOCITY	10//esc set point value (0-255)
#define ATTITUDE_MINIMUM_STEP	0.01// Radians. Minimum change in angle that is passed to the controller

 // Fixed Sample Time
 #define CONTROL_SAMPLE_TIME 	0.005f

 // Saturation limits for the anti-windup
#define INT_ROLL_LOWER_ER_LIMIT		-0.01
#define INT_ROLL_UPPER_ER_LIMIT		 0.01
#define INT_PITCH_LOWER_ER_LIMIT	-0.01
#define INT_PITCH_UPPER_ER_LIMIT	 0.01
#define INT_YAW_LOWER_ER_LIMIT		-0.01
#define INT_YAW_UPPER_ER_LIMIT		 0.01
#define INT_Z_LOWER_ER_LIMIT		-0.03
#define INT_Z_UPPER_ER_LIMIT		 0.03

 // Reference limits for the radio controller
#define REF_ROLL_MAX		0.1 //radians
#define REF_PITCH_MAX		0.1 //radians
#define REF_YAW_MAX			0.1 //radians
#define REF_Z_MAX			0.5 //meters

 //Enable the integration in relation to time
//#define ENABLE_INT_Z
#define ENABLE_INT_ROLL
#define ENABLE_INT_PITCH
//#define ENABLE_INT_YAW


#ifdef STABILITY_CONTROL
 #define STATE_Z		0
 #define STATE_ROLL		1
 #define STATE_PITCH	2
 #define STATE_YAW		3
 #define STATE_DZ		4
 #define STATE_DROLL	5
 #define STATE_DPITCH	6
 #define STATE_DYAW		7
#elif defined PATH_TRACKING_CONTROL
 #define STATE_X			0
 #define STATE_Y			1
 #define STATE_Z			2
 #define STATE_ROLL			3
 #define STATE_PITCH		4
 #define STATE_YAW			5
 #define STATE_ALPHA_R		6
 #define STATE_ALPHA_L		7
 #define STATE_DX			8
 #define STATE_DY			9
 #define STATE_DZ			10
 #define STATE_DROLL		11
 #define STATE_DPITCH		12
 #define STATE_DYAW			13
 #define STATE_DALPHA_R		14
 #define STATE_DALPHA_L		15
#endif




//Environment parameters
#define G	9.81f
// Aircraft parameters - B coordinate system origin in IMU center
#define M    2.3f   // mass inf kg of entire aircraft (including the fiber-glass protection
#define L    0.24737f // aircraft's arm length. Y-axis distance between center of rotation(B) and rotor center of mass.
#define H    0.05394f // center of mass displacement in Z-axis
// Aircraft's Moments of Inertia km*m²
#define IXX  0.04794375f
#define IYY  0.01872182f
#define IZZ  0.03666388f
//antigo
//#define IXX  0.00982678f
//#define IYY  0.01008018f
//#define IZZ  0.00979193f


/* Exported types ------------------------------------------------------------*/

/** \brief Integral do erro dos angulos de orientacao VANT.*/
typedef struct {
	float z, roll, pitch, yaw;
} c_rc_stability_error;

typedef struct {
	float x, y, z, yaw;
} c_rc_pathtrack_error;

/* Exported variables ------------------------------------------------------------*/
bool c_rc_commons_manual_height_control; //If 1 the thrust is controlled with the remote control

/* Exported functions ------------------------------------------------------- */
void c_rc_commons_init(bool manual_height_control);
void set_manual_height_control(bool manual_height_control);
bool get_manual_height_control();

float c_rc_integrate_trapezoidal(float last_integration, float current_value, float last_value, float sample_time);
float c_rc_saturation(float value, float lower_limit, float upper_limit);



#ifdef __cplusplus
}
#endif

#endif

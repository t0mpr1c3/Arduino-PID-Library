#ifndef pid_v1_h

#define pid_v1_h
#define pid_LIBRARY_VERSION	1.0.0

#include <stdlib.h>
#include <avr/io.h>
#include "millis.h"		// API for millisecond counter uint32_t millis (roll your own)

#define MAX_KD_MIN	0.01	// Greater than zero to avoid division by zero in calculation of Kd term

//  public:

enum mode
{
	MANUAL = 0,
	AUTOMATIC = 1
};

enum direction
{
	DIRECT = 1,
	REVERSE = -1
};

enum boolean
{
	FALSE = 0,
	TRUE = 1
};

typedef struct
{
// public
	double* my_input;					// * Pointers to the Input, Output, and set_point variables
	double* my_output;					//   This creates a hard link between the variables and the 
	double* my_set_point;					//   pid_t, freeing the user from having to constantly tell us
								//   what these values are.  with pointers we'll just know.
		  
// private
	double Disp_Kp;						// * we'll hold on to the tuning parameters in user-entered 
	double Disp_Ki;						//   format for display purposes
	double Disp_Kd;						//

	double Kp;						// * (P)roportional Tuning Parameter
	double Ki;						// * (I)ntegral Tuning Parameter
	double Kd;						// * (D)erivative Tuning Parameter

	enum direction Controller_direction;

	uint32_t Last_time;
	double I_term; 
	double D_term; 
	double Last_input;

	uint32_t Sample_time;
	double Out_min; 
	double Out_max;
	enum boolean In_auto;

	double Dither;						// * Noise added to input to smooth quantization errors.
								//   set to smallest step value in input range.

	double Max_Kd;						// * Maximum derivative gain. 
								//   set to 10-20 to avoid applying derivation to high frequency measurement noise.

} pid_t;


//commonly used functions **************************************************************************
	extern pid_t* pid_new( double*, double*, double*,	// * constructor.  links the pid_t to the Input, Output, and 
	double, double, double, enum direction );		//   set_point.  Initial tuning parameters are also set_ here

	extern void pid_set_mode( pid_t* p, enum mode );	// * set PID to either Manual (0) or Auto (non-0)

	extern void pid_compute( pid_t* p );			// * performs the PID calculation.  it should be
																//   called every time loop() cycles. ON/OFF and
																//   calculation frequency can be set using set_mode
																//   set_sample_time respectively

	extern void pid_set_output_limits( pid_t* p, double, 	// clamps the output to a specific range. 0-255 by default, but
	double );													// it's likely the user will want to change this depending on
																// the application



//available but not commonly used functions ********************************************************
	extern void pid_set_tunings( pid_t* p, double, double,	// * While most users will set the tunings once in the 
				double );										//   constructor, this function gives the user the option
																//   of changing tunings during runtime for Adaptive control
	extern void pid_set_controller_direction( pid_t* p, 	// * set the direction, or "Action" of the controller. DIRECT
				enum direction );		//   means the output will increase when error is positive. REVERSE
																//   means the opposite.  it's very unlikely that this will be needed
																//   once it is set in the constructor.
	extern void pid_set_sample_time( pid_t* p, uint32_t );	// * set the frequency, in Milliseconds, with which 
																//   the PID calculation is performed.  default is 100
	extern void pid_set_dither( pid_t* p, double );		// * set the range of the dither 										  
	extern void pid_set_max_Kd( pid_t* p, double );		// * set the maximum derivative gain 										  
									  
									  
//Display functions ****************************************************************
	extern double pid_get_Kp( pid_t* p );			// These functions query the PID for interal values.
	extern double pid_get_Ki( pid_t* p );			// they were created mainly for the PID front-end,
	extern double pid_get_Kd( pid_t* p );			// where it's important to know what is actually 
	extern enum mode pid_get_mode( pid_t* p );		// inside the pPID
	extern enum direction pid_get_direction( pid_t* p )	//
	extern double pid_get_dither( pid_t* p );		// 
	extern double pid_get_maxKd( pid_t* p );		// 
	
/*
private:
	void Pid_init( pid_t* p );
*/

#endif
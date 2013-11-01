#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION 1.0.0

#include "PID_types.h"


class PID
{

public:

  // constants ****************************************************************************************

  // manual or automatic control
  const byte MANUAL    = 0;
  const byte AUTOMATIC = 1;

  // direction
  const byte DIRECT  =  1;
  const byte REVERSE = -1;

  // commonly used methods ****************************************************************************
  PID(double*, double*, double*,// * constructor.  links the PID to the Input, Output, and 
      double, double, double,   //   Setpoint.  Initial tuning parameters are also set here 
      byte);                    //      

  void SetMode(enum Mode);      // * sets PID to either Manual (0) or Auto (non-0)

  void Compute();               // * performs the PID calculation.  it should be        
                                //   called every time loop() cycles. ON/OFF and
                                //   calculation frequency can be set using SetMode
                                //   SetSampleTime respectively

  void SetOutputLimits(double,  // * clamps the output to a specific range. 
      double);                  //   0-255 by default, but it's likely the user will want 
                                //   to change this depending on the application
  bool Compute();               // * performs the PID calculation.  it should be
                                //   called every time loop() cycles. ON/OFF and
                                //   calculation frequency can be set using SetMode
                                //   SetSampleTime respectively



  // less commonly used methods **********************************************************************
  void SetTunings(double,       // * While most users will set the tunings once in the 
    double, double);            //   constructor, this function gives the user the option
                                //   of changing tunings during runtime for Adaptive control
  void SetControllerDirection(  // * Sets the Direction, or "Action" of the controller. DIRECT
    byte);                      //   means the output will increase when error is positive. REVERSE
                                //   means the opposite.  it's very unlikely that this will be needed
                                //   once it is set in the constructor.
  void SetSampleTime(int);      // * sets the frequency, in Milliseconds, with which 
                                //   the PID calculation is performed.  default is 100
  void SetMaxKd(double);        // * set the maximum derivative gain.  default is 10.



  // display functions ******************************************************************************
  double GetKp();               // These functions query the pid for interal values.
  double GetKi();               //  they were created mainly for the pid front-end,
  double GetKd();               // where it's important to know what is actually 
  byte GetMode();               //  inside the PID.
  byte GetDirection();          //
  double GetPTerm();            // 
  double GetITerm();            //  
  double GetDTerm();            // 


private:

  void Initialize();
  void Limit(*double);

  double dispKp;                // * we'll hold on to the tuning parameters in user-entered 
  double dispKi;                //   format for display purposes
  double dispKd;                //

  double kp;                    // * (P)roportional Tuning Parameter
  double ki;                    // * (I)ntegral Tuning Parameter
  double kd;                    // * (D)erivative Tuning Parameter

  double PTerm;                 // * (P)roportional term contributing to the output
  double ITerm;                 // * (I)ntegral term contributing to the output
  double DTerm;                 // * (D)erivative term contributing to the output

  double MaxKd;                 // * Maximum derivative gain. 
                                //   set to 10-20 to avoid applying derivation 
                                //   to high frequency measurement noise.

  const double MAX_KD_MIN = 0.01;    // To avoid division by 0 in calculation of Kd term

  double Dither;                // * Noise added to input to smooth quantization errors.
                                //   set to smallest step value in input range.

  byte controllerDirection;

  double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
  double *myOutput;             //   This creates a hard link between the variables and the 
  double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                //   what these values are.  with pointers we'll just know.

  unsigned long lastTime;
  double lastInput;

  unsigned long SampleTime;
  double outMin, outMax;
};
#endif


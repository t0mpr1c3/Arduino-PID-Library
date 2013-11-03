#ifndef PID_v1_h
#define PID_v1_h
#define PID_LIBRARY_VERSION 1.0.0

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class PID
{
public:

  // constants used in some of the functions below

  enum 
  {
    MANUAL    = 0,
    AUTOMATIC = 1
  };

  enum 
  {
    DIRECT   = 0,
    REVERSE  = 1
  };

  static const double MAX_KD_MIN = 0.01;    // Greater than 0 to avoid division by 0 in calculation of Kd term

  // commonly used functions **************************************************************************
  
  PID(double*, double*, double*,// * constructor.  links the PID to the Input, Output, and 
      double, double, double,   //   Setpoint.  Initial tuning parameters are also set here 
      byte);                    //      

  void SetMode(byte);           // * sets PID to either Manual (0) or Auto (non-0)\

  void SetOutputLimits(double,  // * clamps the output to a specific range. 
      double);                  //   0-255 by default, but it's likely the user will want 
                                //   to change this depending on the application
  bool Compute();               // * performs the PID calculation.  it should be
                                //   called every time loop() cycles. ON/OFF and
                                //   calculation frequency can be set using SetMode
                                //   SetSampleTime respectively

  // available but not commonly used functions ********************************************************
  
  void SetTunings(double,       // * While most users will set the tunings once in the 
      double, double);          //   constructor, this function gives the user the option
                                //   of changing tunings during runtime for Adaptive control
  void SetControllerDirection(  // * Sets the Direction, or "Action" of the controller. DIRECT
      byte);                    //   means the output will increase when error is positive. REVERSE
                                //   means the opposite.  it's very unlikely that this will be needed
                                //   once it is set in the constructor.
  void SetSampleTime(int);      // * sets the frequency, in Milliseconds, with which 
                                //   the PID calculation is performed.  default is 100
  void SetMaxKd(double);        // * set the maximum derivative gain.  default is 10.

  // display functions ****************************************************************
  
  double GetKp();               // These functions query the PID for interal values.
  double GetKi();               //  they were created mainly for the PID front-end,
  double GetKd();               //  where it's important to know what is actually 
  byte GetMode();               //  inside the PID.
  byte GetDirection();          //
  double GetPTerm();            // 
  double GetITerm();            //  
  double GetDTerm();            // 

private:
  void Initialize();

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

  byte mode;
  byte controllerDirection;

  double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
  double *myOutput;             //   This creates a hard link between the variables and the 
  double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                //   what these values are.  with pointers we'll just know.

  unsigned long lastTime;
  double lastInput;

  unsigned long SampleTime;
  
  void Limit(double*);          // * apply output limits to supplied variable
  double outMin, outMax;        // * output limits
};

#endif



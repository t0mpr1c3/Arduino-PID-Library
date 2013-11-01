/**********************************************************************************************
 * Arduino PID Library - Version 1.0.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "PID_v1.h"


/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint,
  double Kp, double Ki, double Kd, byte ControllerDirection)
{
  // default output limit corresponds to 
  // the arduino pwm limits
  PID::SetOutputLimits(0, 255);			

  // default Controller Sample Time is 0.1 seconds
  SampleTime = 100;							

  PID::SetControllerDirection(ControllerDirection);
  PID::SetTunings(Kp, Ki, Kd);

  lastTime = millis()-SampleTime;				
  inAuto = false;
  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
  MaxKd = 10.0;
}

/* Compute() **********************************************************************
 *   This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/ 
bool PID::Compute()
{
  if (!inAuto) 
  {
    return false;
  }
  unsigned long now = millis();
  unsigned long timeChange = (now - lastTime);
  if (timeChange >= SampleTime)
  {
    // compute all the working error variables
    double input = *myInput;

    // compute pid_t Output
    double error = *mySetpoint - input;
    double dInput = (input - lastInput);

    PTerm = kp * error; 
    ITerm += ( ki * error );
    ITerm = Limit(&ITerm);
    DTerm = - kd * dInput / ((MaxKd < MAX_KD_MIN) ? 1.0 : 1.0 + kd / MaxKd);	

    // compute PID Output
    double output = PTerm + ITerm + DTerm;
    Limit(&output);
    *myOutput = output;

    // remember some variables for next time
    lastInput = input;
    lastTime = now;
    return true;
  }
  else 
  {
    return false;
  }
}

/* Limit(...)******************************************************************
 *  Applies outMin and outMax limits to the supplied variable
 ******************************************************************************/
void PID::Limit(double *var)
{
  if (*var > outMax)
  {
    *var = outMax;
  }
  else if (*var < outMin)
  {
    *var = outMin;
  }
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd)
{
  if ((Kp < 0.0) || (Ki < 0.0) || (Kd < 0.0)) 
  {
    return;
  }

  dispKp = Kp; 
  dispKi = Ki; 
  dispKd = Kd;

  double SampleTimeInSec = ((double) SampleTime) / 1000.0;  
  kp = Kp;
  ki = Ki * SampleTimeInSec;
  kd = Kd / SampleTimeInSec;

  if (controllerDirection == REVERSE)
  {
    kp = (0.0 - kp);
    ki = (0.0 - ki);
    kd = (0.0 - kd);
  }
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed	
 ******************************************************************************/
void PID::SetSampleTime(int NewSampleTime)
{
  if (NewSampleTime > 0)
  {
    double ratio = (double) NewSampleTime / (double) SampleTime;
    ki *= ratio;
    kd /= ratio;
    SampleTime = (unsigned long) NewSampleTime;
  }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(double Min, double Max)
{
  if (Min >= Max) 
  {
    return;
  }
  outMin = Min;
  outMax = Max;

  if (inAuto)
  {
    Limit(myOutput);
    Limit(&ITerm);
  }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID::SetMode(byte Mode)
{
  bool newAuto = (Mode == AUTOMATIC);
  if (newAuto == !inAuto)
  {  /*we just went from manual to auto*/
    PID::Initialize();
  }
  inAuto = newAuto;
}

/* Initialize()****************************************************************
 *  does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID::Initialize()
{
  ITerm = *myOutput;
  lastInput = *myInput;
  ITerm = Limit(&Iterm);
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID::SetControllerDirection(byte Direction)
{
  if (inAuto && (Direction != controllerDirection))
  {
    kp = (0.0 - kp);
    ki = (0.0 - ki);
    kd = (0.0 - kd);
  }   
  controllerDirection = Direction;
}

/* pid_set_max_Kd(...)*********************************************************
 * set maximum derivative gain
 * Default 0 means no maximum.
 ******************************************************************************/
void PID::SetMaxKd( double newMaxKd )
{
  if ( newMaxKd < MAX_KD_MIN ) 
  {
    // no limit to Kd
    newMaxKd = 0.0;	
  }
  MaxKd = newMaxKd;
}	

/* Status Functions************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::GetKp()
{ 
  return  dispKp; 
}

double PID::GetKi()
{ 
  return  dispKi; 
}

double PID::GetKd()
{ 
  return  dispKd; 
}

byte PID::GetMode()
{ 
  return  inAuto ? AUTOMATIC : MANUAL; 
}

byte PID::GetDirection()
{ 
  return controllerDirection; 
}

double PID::GetPTerm()
{ 
  return  PTerm; 
}

double PID::GetITerm()
{ 
  return  ITerm; 
}

double PID::GetDTerm()
{ 
  return  DTerm; 
}

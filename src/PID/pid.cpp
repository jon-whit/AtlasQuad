#include "mbed.h"
#include "pid.h"

PID roll_controller(&pid_roll_in, &pid_roll_out, &pid_roll_setpoint, 5.0, 0.0, 0.0, REVERSE);

PID::PID(float* Input, float* Output, float* Setpoint,
         float Kp, float Ki, float Kd, int ControllerDirection)
{
    
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;
    
    PID::SetOutputLimits(PID_OUT_MIN, PID_OUT_MAX);  //default output limit corresponds to PWM limits
                                                

    SampleTime = 100;                                //default Controller Sample Time is 0.1 seconds

    PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd);

    lastTime = time(NULL)-SampleTime;
}

bool PID::Compute()
{
   if(!inAuto) return false;
   unsigned long now = time(NULL);
   unsigned long timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      float input = *myInput;
      float error = *mySetpoint - input;
      ITerm+= (ki * error);
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
      float dInput = (input - lastInput);
 
      /*Compute PID Output*/
      float output = kp * error + ITerm- kd * dInput;
      
      if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
      *myOutput = output;
      
      /*Remember some variables for next time*/
      lastInput = input;
      lastTime = now;
      return true;
   }
   else return false;
}

void PID::SetTunings(float Kp, float Ki, float Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;
 
   dispKp = Kp; dispKi = Ki; dispKd = Kd;
   
   float SampleTimeInSec = ((float)SampleTime)/1000;  
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
 
  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}

void PID::SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      float ratio  = (float)NewSampleTime
                      / (float)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}

void PID::SetOutputLimits(float Min, float Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;
 
   if(inAuto)
   {
       if(*myOutput > outMax) *myOutput = outMax;
       else if(*myOutput < outMin) *myOutput = outMin;
     
       if(ITerm > outMax) ITerm= outMax;
       else if(ITerm < outMin) ITerm= outMin;
   }
}

void PID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
        PID::Initialize();
    }
    inAuto = newAuto;
}

void PID::Initialize()
{
   ITerm = *myOutput;
   lastInput = *myInput;
   if(ITerm > outMax) ITerm = outMax;
   else if(ITerm < outMin) ITerm = outMin;
}

void PID::SetControllerDirection(int Direction)
{
   if(inAuto && Direction != controllerDirection)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }   
   controllerDirection = Direction;
}

float PID::GetKp(){ return  dispKp; }
float PID::GetKi(){ return  dispKi;}
float PID::GetKd(){ return  dispKd;}
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PID::GetDirection(){ return controllerDirection;}

void PIDInit(void)
{
    
}

void PIDUpdate(void)
{
    pid_roll_in  = roll;
    pid_pitch_in = pitch;
    pid_yaw_in   = yaw;
}

void PIDCompute(void)
{

}
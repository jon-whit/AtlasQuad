#include "mbed.h"
#include "pid.h"

extern PID roll_controller;
extern PID pitch_controller;
extern PID yaw_controller;

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

    lastTime = t.read_ms()-SampleTime;
}

bool PID::Compute()
{
   if(!inAuto) return false;
   unsigned long now = t.read_ms();
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
void PID::SetKp(float Kp) { kp = Kp; }
void PID::SetKi(float Ki) { ki = Ki; }
void PID::SetKd(float Kd) { kd = Kd; }
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PID::GetDirection(){ return controllerDirection;}

void PIDInit(int SampleTime)
{
    // Initialize the roll controller
    roll_controller.SetOutputLimits(PID_OUT_MIN, PID_OUT_MAX);
    roll_controller.SetMode(AUTOMATIC);
    roll_controller.SetSampleTime(SampleTime); // in ms
    
    // Initialize the pitch controller
    pitch_controller.SetOutputLimits(PID_OUT_MIN, PID_OUT_MAX);
    pitch_controller.SetMode(AUTOMATIC);
    pitch_controller.SetSampleTime(SampleTime); // in ms
    
    // Initialize the yaw controller
    yaw_controller.SetOutputLimits(PID_OUT_MIN, PID_OUT_MAX);
    yaw_controller.SetMode(AUTOMATIC);
    yaw_controller.SetSampleTime(SampleTime); // in ms
}

void PIDUpdate(void)
{
    pid_roll_in  = roll;
    pid_pitch_in = pitch;
    pid_yaw_in   = yaw;
}

void PIDCompute(void)
{
    roll_controller.Compute();
    pitch_controller.Compute();
    yaw_controller.Compute();
}

void PIDSetConstants(float kp, float ki, float kd)
{
    roll_controller.SetTunings(kp, ki, kd);
    pitch_controller.SetTunings(kp, ki, kd);
    yaw_controller.SetTunings(kp, ki, kd);
}

void PIDSetKp(float kp)
{
    roll_controller.SetKp(kp);
    pitch_controller.SetKp(kp);
    yaw_controller.SetKp(kp);
}

void PIDSetKi(float ki)
{
    roll_controller.SetKi(ki);
    pitch_controller.SetKi(ki);
    yaw_controller.SetKi(ki);
}

void PIDSetKd(float kd)
{
    roll_controller.SetKd(kd);
    pitch_controller.SetKd(kd);
    yaw_controller.SetKd(kd);
}

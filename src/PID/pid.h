#ifndef PID_H
#define PID_H

extern float pid_roll_in, pid_pitch_in, pid_yaw_in;
extern float pid_roll_out, pid_pitch_out, pid_yaw_out;
extern float pid_roll_setpoint, pid_pitch_setpoint, pid_yaw_setpoint;
extern float roll, pitch, yaw;

class PID
{
private:

    void Initialize(void);
    
    float dispKp;              // * we'll hold on to the tuning parameters in user-entered 
    float dispKi;              //   format for display purposes
    float dispKd;              //
    
    float kp;                  // * (P)roportional Tuning Parameter
    float ki;                  // * (I)ntegral Tuning Parameter
    float kd;                  // * (D)erivative Tuning Parameter

    int controllerDirection;

    float *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    float *myOutput;             //   This creates a hard link between the variables and the 
    float *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
              
    unsigned long lastTime;
    float ITerm, lastInput;

    unsigned long SampleTime;
    float outMin, outMax;
    bool inAuto;
    
public:

    #define AUTOMATIC 1
    #define MANUAL    0
    #define DIRECT    0
    #define REVERSE   1
    #define PID_OUT_MIN 1000
    #define PID_OUT_MAX 1900
      
    PID(float* Input, float* Output, float* Setpoint,
        float Kp, float Ki, float Kd, int ControllerDirection);
        
    bool Compute(void);
    
    void SetTunings(float Kp, float Ki, float Kd);
    void SetSampleTime(int NewSampleTime);
    void SetOutputLimits(float Min, float Max);
    void SetMode(int Mode);
    void SetControllerDirection(int Direction);
    
    float GetKp();
    float GetKi();
    float GetKd();
    int GetMode();
    int GetDirection();
        
};

void PIDInit(void);
void PIDUpdate(void);
void PIDCompute(void);

#endif
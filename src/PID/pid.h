#ifndef PID_H
#define PID_H

class PID
{
private:

    void Initialize(void);
    
    double dispKp;              // * we'll hold on to the tuning parameters in user-entered 
    double dispKi;              //   format for display purposes
    double dispKd;              //
    
    double kp;                  // * (P)roportional Tuning Parameter
    double ki;                  // * (I)ntegral Tuning Parameter
    double kd;                  // * (D)erivative Tuning Parameter

    int controllerDirection;

    double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    double *myOutput;             //   This creates a hard link between the variables and the 
    double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
              
    unsigned long lastTime;
    double ITerm, lastInput;

    unsigned long SampleTime;
    double outMin, outMax;
    bool inAuto;
    
public:

    #define AUTOMATIC 1
    #define MANUAL    0
    #define DIRECT    0
    #define REVERSE   1
      
    PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int ControllerDirection);
        
    bool Compute(void);
    
    void SetTunings(double Kp, double Ki, double Kd);
    void SetSampleTime(int NewSampleTime);
    void SetOutputLimits(double Min, double Max);
    void SetMode(int Mode);
    void SetControllerDirection(int Direction);
    
    double GetKp();
    double GetKi();
    double GetKd();
    int GetMode();
    int GetDirection();
        
};

#endif
#ifndef PID_H
#define PID_H

extern float pid_roll_in, pid_pitch_in, pid_yaw_in;
extern float pid_roll_out, pid_pitch_out, pid_yaw_out;
extern float pid_roll_setpoint, pid_pitch_setpoint, pid_yaw_setpoint;
extern float roll, pitch, yaw;
extern Timer t;

#define KP 0.2
#define KI 0.2
#define KD 0.006

/*
 * PID - A PID controller implementation.
 *
 * Code used with permission from Brett Beauregard. For more information,
 * please see the link below:
 *
 * http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
 */
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
    #define PID_OUT_MIN -200
    #define PID_OUT_MAX 200
      
    PID(float* Input, float* Output, float* Setpoint,
        float Kp, float Ki, float Kd, int ControllerDirection);
        
    bool Compute(void);
    
    /*
     * Sets the PID tuning parameters on the fly.
     */
    void SetTunings(float Kp, float Ki, float Kd);
    
    /*
     * Set the Kp parameter on the fly.
     */
    void SetKp(float Kp);
    
    /*
     * Set the Ki parameter on the fly.
     */
    void SetKi(float Ki);
    
    /*
     * Set the Kd parameter on the fly.
     */
    void SetKd(float Kd);
    
    /*
     * Sets the sample time of the PID controller, maintaining a constant ratio, 
     * thus making a smooth transition between different sample rates.
     */
    void SetSampleTime(int NewSampleTime);
    
    /*
     * Sets the minimum and maximum output limits of this PID controller.
     */
    void SetOutputLimits(float Min, float Max);
    
    /*
     * Sets the mode of this PID controller (either AUTOMATIC or MANUAL).
     */
    void SetMode(int Mode);
    
    /*
     * Sets the direction of this PID controller (either DIRECT or REVERSE).
     */
    void SetControllerDirection(int Direction);
    
    float GetKp();
    float GetKi();
    float GetKd();
    int GetMode();
    int GetDirection();
        
};

/*
 * Initializes all of the PIDs of the AtlasQuad system.
 */
void PIDInit(int SampleTime);

/*
 * Updates all of the inputs to each PID controller for the AtlasQuad system.
 */
void PIDUpdate(void);

/*
 * Computes the output for each PID controller for the AtlasQuad system.
 */
void PIDCompute(void);

/*
 * Sets the PID constants across each PID controller for the AtlasQuad system.
 */
void PIDSetConstants(float kp, float ki, float kd);

/*
 * Sets the Kp parameter across each PID controller for the AtlasQuad system.
 */
void PIDSetKp(float kp);

/*
 * Sets the Ki parameter across each PID controller for the AtlasQuad system.
 */
void PIDSetKi(float ki);

/*
 * Sets the Kd parameter across each PID controller for the AtlasQuad system.
 */
void PIDSetKd(float kd);

#endif

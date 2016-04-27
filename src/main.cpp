#include "mbed.h"
#include "motors.h"
#include "config.h"
#include "ITG3200.h"
#include "ADXL345.h"
#include "xbeeuart.h"
#include "pid.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Serial pc(SERIAL_TX, SERIAL_RX); // TX, RX
ITG3200 gyro(IMU_SDA, IMU_SCL);
ADXL345 accl(IMU_SDA, IMU_SCL);
XBeeUART comm((uint16_t) 0); // 16-bit remote address of 1
PID roll_controller(&pid_roll_in, &pid_roll_out, &pid_roll_setpoint, KP, KI, KD, REVERSE);
PID pitch_controller(&pid_pitch_in, &pid_pitch_out, &pid_pitch_setpoint, KP, KI, KD, REVERSE);
PID yaw_controller(&pid_yaw_in, &pid_yaw_out, &pid_yaw_setpoint, KP, KI, KD, DIRECT);

const float acclAlpha = 0.5;
const float gyroAlpha = 0.95;

float gyroOffsetX = 0.0;
float gyroOffsetY = 0.0;
float gyroOffsetZ = 0.0;
float acclOffsetX = 0.0;
float acclOffsetY = 0.0;
float acclOffsetZ = 0.0;

Ticker commticker;
Timer t; // global timer
uint32_t tcurr, tprev = 0;

uint16_t throttle = 0;

AccelG fG;
float acclroll, acclpitch; // (deg)
float fXg, fYg, fZg = 0.0;

float pitch, roll, yaw = 0.0;

// PID variables
float pid_roll_in,   pid_roll_out,   pid_roll_setpoint = 0.0;
float pid_pitch_in,  pid_pitch_out,  pid_pitch_setpoint = 0.0;
float pid_yaw_in,  pid_yaw_out,  pid_yaw_setpoint = 0.0;

// Motor control - automatic (PID) or manual (comm)
bool motormode = MANUAL;

int mspeed1, mspeed3 = MOTOR_MIN; // Motors along the x-axis
//=> Changing their speeds will affect the pitch

int mspeed2, mspeed4 = MOTOR_MIN; // Motors along the y-axis
//=> Changing their speeds will affect the roll


/*
* Function to initialize the Gyro sample rate and give it the base offset value to give
* better calculations in the GetAngleMeasurements function.
*/
void InitGyro()
{
    // Set the internal sample rate to 1kHz and set the sample
    // time to 1kHz (1ms)
    gyro.setLpBandwidth(LPFBW_188HZ);
    gyro.setSampleRateDivider(0);
    wait_ms(50); // Wait for ZRO settling
    
    // Calculate an average offset for the gyro
    for (int i = 0; i < OFFSET_AVG_SAMPLES; i++)
    {
      gyroOffsetX += gyro.getGyroX();
      gyroOffsetY += gyro.getGyroY();
    }
    //use the average calculated while getting the samples to give a more accurate offset value
    gyroOffsetX /= OFFSET_AVG_SAMPLES;
    gyroOffsetY /= OFFSET_AVG_SAMPLES;
    
    #ifdef DEBUG
    pc.printf("Gyro Offset (X, Y, Z): (%f, %f, %f)\r\n", gyroOffsetX, gyroOffsetY, gyroOffsetZ);
    #endif
}


/*
* initialize the accelerometer and set the sample rate frequency.
* Also get the base offset of the accelerometer at startup to give better
* measurements in the GetAnglemeasurements function.
*/
void InitAccelerometer()
{
    // Put the ADXL345 into standby mode to configure the device
    accl.setPowerControl(0x00);
    
    // Configure the ADXL345 in 10-bit resolution, +/-16g range, 1.6kHz data rate
    accl.setDataFormatControl(0x03);
    accl.setDataRate(ADXL345_1600HZ);

    // Start ADXL345 measurement mode
    accl.setPowerControl(0x08);

    // Calculate an average offset for the accelerometer
    AccelG fg;
    for (int i = 0; i < OFFSET_AVG_SAMPLES; i++)
    {
        fg = accl.getAccelG();
        
        acclOffsetX += fg.x;
        acclOffsetY += fg.y;
        acclOffsetZ += (fg.z - 1);
    }
    //Using the offsets sames get the average value to give us a more accurate value
    acclOffsetX /= OFFSET_AVG_SAMPLES;
    acclOffsetY /= OFFSET_AVG_SAMPLES;
    acclOffsetZ /= OFFSET_AVG_SAMPLES;
    
    #ifdef DEBUG
    pc.printf("Accel Offset (X, Y, Z): (%f, %f, %f)\r\n", acclOffsetX, acclOffsetY, acclOffsetZ);
    #endif
}


/*
*  Initializes the Gyro and the accelerometer at startup.
*/
void InitIMU(void)
{
    InitGyro();  //initualize the gyro
    InitAccelerometer(); //initialize the accelerometer
}


/*
* function using a sample time to calculate the current angle over time.
* the main features of this function are the filter for the accelerometer
* the measurements gathered from the gyro to get the current angle 
* and the complimentary filter we used to get our accurate measurement of our
* angle at any given point in time.
*/
void GetAngleMeasurements()
{
    if (((tcurr = t.read_ms()) - tprev) >= IMU_SAMPLE_TIME)
    {    
        // Get the forces in X, Y, and Z
        fG = accl.getAccelG();
        
        // Low pass filter
        fXg = (fG.x - acclOffsetX) * acclAlpha + (fXg * (1.0 - acclAlpha));
        fYg = (fG.y - acclOffsetY) * acclAlpha + (fYg * (1.0 - acclAlpha));
        fZg = (fG.z - acclOffsetZ) * acclAlpha + (fZg * (1.0 - acclAlpha));
        
        acclroll  = (atan2(-fYg, fZg)*180.0)/M_PI;
        acclpitch = (atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0)/M_PI;

        // Calculate the roll and pitch angles from the gyro measurements (in deg/s)
        float grollRate  = (gyro.getGyroX() - gyroOffsetX) / 14.375;
        float gpitchRate = (gyro.getGyroY() - gyroOffsetY) / 14.375;
        float dt = (tcurr - tprev) / 1000.0;

        // Complementary filter used to obtain roll and pitch (in deg)
        roll  = gyroAlpha*(roll + grollRate * dt)  + (1-gyroAlpha)*acclroll;
        pitch = gyroAlpha*(pitch + gpitchRate * dt) + (1-gyroAlpha)*acclpitch;
        
        #ifdef DEBUG
        pc.printf("(Roll, Pitch): (%f, %f)\r\n", roll, pitch);
        #endif
        
        tprev = tcurr;
    }
}

void ControlUpdate()
{
    // Update the PID control values, and issue a PID computation
    PIDUpdate();
    PIDCompute();
    
    // Calculate the new motor speeds
    if(motormode == AUTOMATIC) {
        //mspeed1 = throttle + pid_pitch_out;
        mspeed2 = throttle + pid_roll_out;
        //mspeed3 = throttle - pid_pitch_out;
        mspeed4 = throttle - pid_roll_out;
    }
    
    // Update the motor speeds
    UpdateMotors(&mspeed1, &mspeed2, &mspeed3, &mspeed4);
}

/** Dummy callback functions - fill these out with correct commands/function calls later **/
void StopMotors() {
    
    #ifdef DEBUG
    pc.printf("stop quadcopter\r\n");
    #endif

    // add code to stop motors in an emergency
    motormode = MANUAL;
    mspeed1 = MOTOR_MIN;
    mspeed2 = MOTOR_MIN;
    mspeed3 = MOTOR_MIN;
    mspeed4 = MOTOR_MIN;
    UpdateMotors(&mspeed1, &mspeed2, &mspeed3, &mspeed4);

    // also stop PID computes
    roll_controller.SetMode(MANUAL);
    pitch_controller.SetMode(MANUAL);
    yaw_controller.SetMode(MANUAL);
}

void ResetAll() {
    NVIC_SystemReset();
}

void Heartbeat() {
    comm.send_data((uint8_t *) "OK");
}

void SetMotor(uint8_t motor, uint16_t value) {
    
    #ifdef DEBUG
    pc.printf("set motor %d to %d\r\n", motor, value);
    #endif

    // Change each individual motor/ESC in manual mode or set to automatic, or update throttle
    switch(motor) {
        case ESC_1:
            motormode = MANUAL;
            mspeed1 = (int) value;
            break;
        case ESC_2:
            motormode = MANUAL;
            mspeed2 = (int) value;
            break;
        case ESC_3:
            motormode = MANUAL;
            mspeed3 = (int) value;
            break;
        case ESC_4:
            motormode = MANUAL;
            mspeed4 = (int) value;
            break;
        case ESC_AUTO:
            motormode = AUTOMATIC;
            roll_controller.SetMode(AUTOMATIC);
            pitch_controller.SetMode(AUTOMATIC);
            yaw_controller.SetMode(AUTOMATIC);
            break;
        case THROTTLE:
            throttle = value;
            break;
    }
}

void SetRotation(uint8_t mode, float value) {
    #ifdef DEBUG
    pc.printf("move %d %d\r\n", mode, value);
    #endif

    char buffer[12];
    switch(mode) {
        case ROT_X:
            pid_roll_setpoint = value;
            break;
        case ROT_Y:
            pid_pitch_setpoint = value;
            break;
        case ROT_Z:
            pid_yaw_setpoint = value;
            break;
        case GET_ROT_X:
            snprintf(buffer, sizeof(buffer), "%f", roll);
            comm.send_data((uint8_t *) buffer);
            break;
        case GET_ROT_Y:
            snprintf(buffer, sizeof(buffer), "%f", pitch);
            comm.send_data((uint8_t *) buffer);
            break;
        case GET_ROT_Z:
            snprintf(buffer, sizeof(buffer), "%f", yaw);
            comm.send_data((uint8_t *) buffer);
            break;
    }
}

void SetPID(uint8_t mode, float value) 
{
    
    switch (mode)
    {
        case UART_KP:
            PIDSetKp(value);
            break;
        case UART_KI:
            PIDSetKi(value);
            break;
        case UART_KD:
            PIDSetKd(value);
            break;
        default:
            break;
    }
}

int main()
{
    // Start the global timer
    t.start();
    
    #ifdef DEBUG 
    pc.printf("Initializing Motors...\r\n");
    #endif
    InitMotors();
    
    #ifdef DEBUG 
    pc.printf("Initializing IMU...\r\n");
    #endif
    InitIMU();
    
    #ifdef DEBUG
    pc.printf("Initializing PID Controllers...\r\n");
    #endif
    PIDInit(10); // Initialize the PID controllers with a 100Hz sampling rate
    
    #ifdef DEBUG
    pc.printf("Initializing Communications...\r\n");
    #endif
    comm.init();
    comm.register_callbacks(&StopMotors, &ResetAll, &Heartbeat, &SetRotation, &SetMotor, &SetPID);
    commticker.attach(&comm, &XBeeUART::process_frames, 0.1); // process received frames at 10 Hz

    #ifdef DEBUG
    pc.printf("Arming Motors...\r\n");
    #endif
    ArmMotors();
    
    while (1)
    {   
        // Runs at ~2.8kHz
        GetAngleMeasurements();
        ControlUpdate();  
    }
}

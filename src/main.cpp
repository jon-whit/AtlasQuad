#include "mbed.h"
#include "motors.h"
#include "ITG3200.h"
#include "ADXL345.h"
#include "pid.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SAMPLE_TIME 0.5 // (ms) --> 2kHz

Serial pc(SERIAL_TX, SERIAL_RX); // TX, RX
ITG3200 gyro(PB_9, PB_8);
ADXL345 accl(PB_9, PB_8);

const float acclAlpha = 0.5;
const float gyroAlpha = 0.98;
const int gyroOffsetX = 0;
const int gyroOffsetY = 0;
const int gyroOffsetZ = 0;

Timer t; // global timer
uint32_t tprev;

uint16_t throttle = 0;

float fXg = 0;
float fYg = 0;
float fZg = 0;

AccelG fG;
float acclPitch, acclRoll; // (deg)

float gyroPitch, gyroRoll, gyroYaw; // (deg)

float pitch, roll, yaw = 0.0;

// PID variables
float pid_roll_in,   pid_roll_out,   pid_roll_setpoint = 0;
float pid_pitch_in,  pid_pitch_out,  pid_pitch_setpoint = 0;
float pid_yaw_in,  pid_yaw_out,  pid_yaw_setpoint = 0;

int mspeed1 = MOTOR_MIN, mspeed3 = MOTOR_MIN; // Motors along the x-axis
//=> Changing their speeds will affect the pitch

int mspeed2 = MOTOR_MIN, mspeed4 = MOTOR_MIN; // Motors along the y-axis
//=> Changing their speeds will affect the roll

void InitIMU(void)
{
    // Set the internal sample rate to 1kHz and set the sample
    // time to 200Hz (5ms)
    gyro.setLpBandwidth(LPFBW_188HZ);
    gyro.setSampleRateDivider(4);
    
    // Put the ADXL345 into standby mode to configure the device
    accl.setPowerControl(0x00);
    
    // Configure the ADXL345 in 10-bit resolution, +/-16g range, 200 Hz data rate
    accl.setDataFormatControl(0x03);
    accl.setDataRate(ADXL345_200HZ);
    
    // Start ADXL345 measurement mode
    accl.setPowerControl(0x08);
}

void GetAngleMeasurements()
{
    if (t.read_ms() - tprev >= SAMPLE_TIME)
    {
        // Get the forces in X, Y, and Z
        fG = accl.getAccelG();

        // Low Pass Filter
        fXg = fG.x * acclAlpha + (fXg * (1.0 - acclAlpha));
        fYg = fG.y * acclAlpha + (fYg * (1.0 - acclAlpha));
        fZg = fG.z * acclAlpha + (fZg * (1.0 - acclAlpha));

        // Roll & Pitch Equations
        acclRoll  = (atan2(-fYg, fZg)*180.0)/M_PI;
        acclPitch = (atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0)/M_PI;

        // Calculate the roll and pitch angles from the gyro measurements
        float groll = (float) gyro.getGyroX() / 14.375;
        float gpitch = (float) gyro.getGyroY() / 14.375;
        uint32_t dt = t.read_ms() - tprev;
        gyroRoll += groll * dt;
        gyroPitch += gpitch * dt;

        // Complementary filter
        roll  = gyroAlpha*gyroRoll  + (1-gyroAlpha)*acclRoll;
        pitch = gyroAlpha*gyroPitch + (1-gyroAlpha)*acclPitch;
    }
}

void ControlUpdate()
{
    // Update the PID control values, and issue a PID computation
    PIDUpdate();
    PIDCompute();
    
    // Calculate the new motor speeds
    //mspeed1 = throttle + pid_pitch_out;
    mspeed2 = throttle + pid_roll_out;
    //mspeed3 = throttle - pid_pitch_out;
    mspeed4 = throttle - pid_roll_out;
    
    // Update the motor speeds
    UpdateMotors(&mspeed1, &mspeed2, &mspeed3, &mspeed4);
}

int main()
{
    // Start the global timer
    t.start();
    
    pc.printf("Initializing Motors...\r\n");
    InitMotors();
    
    pc.printf("Initializing IMU...\r\n");
    InitIMU();
    
    pc.printf("Initializing PID Controllers...\r\n");
    PIDInit(1); // Initialize the PID controllers with a 1kHz sampling rate
    
    pc.printf("Arming Motors...\r\n");
    ArmMotors();
    
    while (1)
    {   
        GetAngleMeasurements();
        ControlUpdate();
        
        tprev = t.read_ms();
    }
}
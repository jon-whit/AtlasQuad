#include "mbed.h"
#include "motors.h"
#include "ITG3200.h"
#include "ADXL345.h"
#include "xbeeuart.h"
#include "pid.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Serial pc(SERIAL_TX, SERIAL_RX); // TX, RX
ITG3200 gyro(PB_9, PB_8);
ADXL345 accl(PB_9, PB_8);
XBeeUART comm((uint16_t) 0); // 16-bit remote address of 1

Timer t;
uint16_t currentTime;
uint16_t previousTime;

const float acclAlpha = 0.5;
const float gyroAlpha = 0.98;
const int gyroOffsetX = 0;
const int gyroOffsetY = 0;
const int gyroOffsetZ = 0;

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
    // Get the forces in X, Y, and Z
    fG = accl.getAccelG();
    
    // Low Pass Filter
    fXg = fG.x * acclAlpha + (fXg * (1.0 - acclAlpha));
    fYg = fG.y * acclAlpha + (fYg * (1.0 - acclAlpha));
    fZg = fG.z * acclAlpha + (fZg * (1.0 - acclAlpha));
    
    // Roll & Pitch Equations
    acclRoll  = (atan2(-fYg, fZg)*180.0)/M_PI;
    acclPitch = (atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0)/M_PI;
    currentTime = t.read_us();
    gyroRoll += (currentTime - previousTime) * (gyroRoll/14.375);
    gyroPitch += (currentTime - previousTime) * (gyroPitch/14.375);
    previousTime = currentTime;

    // Complementary filter
    roll  = gyroAlpha*gyroRoll  + (1-gyroAlpha)*acclRoll;
    pitch = gyroAlpha*gyroPitch + (1-gyroAlpha)*acclPitch;
}

void ControlUpdate()
{
    // Update the PID control values, and issue a PID computation
    PIDUpdate();
    PIDCompute();
    
    // Calculate the new motor speeds
    mspeed1 = throttle + pid_pitch_out;
    mspeed2 = throttle + pid_roll_out;
    mspeed3 = throttle - pid_pitch_out;
    mspeed4 = throttle - pid_roll_out;
    
    // Update the motor speeds
    UpdateMotors(&mspeed1, &mspeed2, &mspeed3, &mspeed4);
}

int main()
{
    t.start();
    previousTime = 0;
    pc.printf("Initializing Motors...\r\n");
    InitMotors();
    
    pc.printf("Initializing IMU...\r\n");
    InitIMU();
    
    pc.printf("Initializing PID Controllers...\r\n");
    PIDInit();
    
    pc.printf("Initializing Communications...\r\n");
    comm.init();

    pc.printf("Arming Motors...\r\n");
    ''();
    
    while (1)
    {   
        comm.process_frames();
        GetAngleMeasurements();
        ControlUpdate();  

        // maybe place this in its own thread/interrupt-driven callback
        uint8_t message = comm.get_message_byte();
        if(message != 0) {
            pc.printf("Received byte %x\r\n", message);
        }
    }
}

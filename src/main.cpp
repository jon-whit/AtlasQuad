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

const float acclAlpha = 0.5;
const float gyroAlpha = 0.98;
const int gyroOffsetX;
const int gyroOffsetY;
const int gyroOffsetZ;

Timer t; // global timer
uint32_t tprev = 0;

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
    // time to 1kHz (1ms)
    gyro.setLpBandwidth(LPFBW_188HZ);
    gyro.setSampleRateDivider(0);
    
    // Set offset for gyro
    gyroOffsetX = gyro.getGyroX();
    gyroOffsetY = gyro.getGyroY();
    gyroOffsetZ = gyro.getGyroZ();
    
    // Put the ADXL345 into standby mode to configure the device
    accl.setPowerControl(0x00);
    
    // Configure the ADXL345 in 10-bit resolution, +/-16g range, 1.6kHz data rate
    accl.setDataFormatControl(0x03);
    accl.setDataRate(ADXL345_1600HZ);

    // Start ADXL345 measurement mode
    accl.setPowerControl(0x08);

    // Set offset for accl
    int16_t readings[3] = {-1, -1, -1};
    getRawOutput(readings);
    setOffset(ADXL345_X, (uint8_t)readings[0]);
    setOffset(ADXL345_Y, (uint8_t)readings[1]);
    setOffset(ADXL345_Z, (uint8_t)readings[2]);
}

void GetAngleMeasurements()
{
    if (t.read_us() - tprev >= IMU_SAMPLE_TIME)
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
        float groll = (float) (gyro.getGyroX() - gyroOffsetX) / 14.375;
        float gpitch = (float) (gyro.getGyroY() - gyroOffsetY) / 14.375;
        uint32_t dt = t.read_us() - tprev;
        gyroRoll += groll * dt;
        gyroPitch += gpitch * dt;

        // Complementary filter
        roll  = gyroAlpha*gyroRoll  + (1-gyroAlpha)*acclRoll;
        pitch = gyroAlpha*gyroPitch + (1-gyroAlpha)*acclPitch;
        
        tprev = t.read_us();
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

/** Dummy callback functions - fill these out with correct commands/function calls later **/
void StopMotors() {
    
    #ifdef DEBUG
        pc.printf("stop quadcopter\r\n");
    #endif

    // add code to stop motors in an emergency
}

void Heartbeat() {
    comm.send_data((uint8_t *) "OK");
}

void SetMotor(uint8_t motor, uint8_t value) {
    
    #ifdef DEBUG
        pc.printf("set motor %d to %d\r\n", motor, value);
    #endif

    // add code to change individual motor/ESC
}

uint32_t SetPositionRotation(uint8_t mode, uint32_t value) {
    
    #ifdef DEBUG
        pc.printf("move %d %d\r\n", mode, value);
    #endif

    // parse mode (X/Y/Z position/rotation set/get)
    return 0;
}

uint32_t SetIMU(uint8_t mode) {
    
    #ifdef DEBUG
        pc.printf("imu %d\r\n", mode);
    #endif

    // get/set IMU values
    return 0;
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
    comm.register_callbacks(&StopMotors, &Heartbeat, &SetPositionRotation, &SetMotor, &SetIMU);

    #ifdef DEBUG
        pc.printf("Arming Motors...\r\n");
    #endif
    ArmMotors();
    
    while (1)
    {   
        comm.process_frames();
        GetAngleMeasurements();
        ControlUpdate();  
    }
}

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
const float gyroAlpha = 0.95;
int gyroOffsetX = 0;
int gyroOffsetY = 0;
int gyroOffsetZ = 0;
int acclOffsetX = 0;
int acclOffsetY = 0;
int acclOffsetZ = 0;

Ticker commticker;
Timer t; // global timer
uint32_t tcurr, tprev = 0;

uint16_t throttle = 0;

float fXg = 0.0;
float fYg = 0.0;
float fZg = 0.0;

AccelG fG;
float acclPitch, acclRoll; // (deg)

float gyroPitch, gyroRoll, gyroYaw; // (deg)

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

void InitIMU(void)
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
    	gyroOffsetZ += gyro.getGyroZ();
    }
    
    gyroOffsetX /= OFFSET_AVG_SAMPLES;
    gyroOffsetY /= OFFSET_AVG_SAMPLES;
    gyroOffsetZ /= OFFSET_AVG_SAMPLES;
	
    // Put the ADXL345 into standby mode to configure the device
    accl.setPowerControl(0x00);
    
    // Configure the ADXL345 in 10-bit resolution, +/-16g range, 1.6kHz data rate
    accl.setDataFormatControl(0x03);
    accl.setDataRate(ADXL345_1600HZ);

    // Start ADXL345 measurement mode
    accl.setPowerControl(0x08);

    // Calculate an average offset for the accelerometer    
    for (int i = 0; i < OFFSET_AVG_SAMPLES; i++)
    {
        int16_t readings[3] = {-1, -1, -1};
        accl.getRawOutput(readings);
        
        acclOffsetX += readings[0];
        acclOffsetY += readings[1];
        acclOffsetZ += readings[2];
    }
    
    acclOffsetX /= OFFSET_AVG_SAMPLES;
    acclOffsetY /= OFFSET_AVG_SAMPLES;
    acclOffsetZ /= OFFSET_AVG_SAMPLES;
    
    accl.setOffset(ADXL345_X, acclOffsetX);
    accl.setOffset(ADXL345_Y, acclOffsetY);
    accl.setOffset(ADXL345_Z, acclOffsetZ);
    
    #ifdef DEBUG
    pc.printf("Gyro Offset (%d, %d, %d)\r\n", gyroOffsetX, gyroOffsetY, gyroOffsetZ);
    pc.printf("Accel Offset (%d, %d, %d)\r\n", acclOffsetX, acclOffsetY, acclOffsetZ);
    #endif
}

void GetAngleMeasurements()
{
    if (((tcurr = t.read_ms()) - tprev) >= IMU_SAMPLE_TIME)
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
        float dt = (tcurr - tprev) / 1000.0;

        // Complementary filter
        roll  = gyroAlpha*(roll + groll*dt)  + (1-gyroAlpha)*acclRoll;
        pitch = gyroAlpha*(pitch + gpitch*dt) + (1-gyroAlpha)*acclPitch;
        
        #ifdef DEBUG
        //pc.printf("(%f, %f, %f)", roll, pitch, yaw);
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
        mspeed1 = throttle - pid_pitch_out;
        mspeed2 = throttle - pid_roll_out;
        mspeed3 = throttle + pid_pitch_out;
        mspeed4 = throttle + pid_roll_out;
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
}

void Heartbeat() {
    comm.send_data((uint8_t *) "OK");
}

void SetMotor(uint8_t motor, uint16_t value) {
    
    #ifdef DEBUG
    pc.printf("set motor %d to %d\r\n", motor, value);
    #endif

    // Change each individual motor/ESC, or update throttle
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
            break;
        case THROTTLE:
            throttle = value;
            break;
    }
}

uint32_t SetPositionRotation(uint8_t mode, uint32_t value) {
    
    #ifdef DEBUG
    pc.printf("move %d %d\r\n", mode, value);
    #endif

    // parse mode (X/Y/Z position/rotation set/get)
    return 0;
}

void SetPID(uint8_t mode, float value) {
    // if values are zero, leave them alone
    float kp = KP;
    float ki = KI;
    float kd = KD;
    if(mode == UART_KP)
        kp = value;
    else if(mode == UART_KI)
        ki = value;
    else if(mode == UART_KD)
        kd = value;

    #ifdef DEBUG
    pc.printf("setpid %f %f %f\r\n", kp, ki, kd);
    #endif

    PIDSetConstants(kp, ki, kd);
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
    comm.register_callbacks(&StopMotors, &Heartbeat, &SetPositionRotation, &SetMotor, &SetIMU, &SetPID);
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

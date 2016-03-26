#include "mbed.h"
#include "motors.h"
#include "ITG3200.h"
#include "pid.h"

Serial pc(SERIAL_TX, SERIAL_RX); // TX, RX
//ITG3200 gyro(PB_9, PB_8);

int setpoint = 0;

int mspeed1 = MOTOR_MIN, mspeed2 = MOTOR_MIN;
int mspeed3 = MOTOR_MIN, mspeed4 = MOTOR_MIN;

int main()
{
    pc.printf("Initializing Motors...\r\n");
    InitMotors();
    
    pc.printf("Initializing IMU...\r\n");
    
    pc.printf("Arming Motors...\r\n");
    ArmMotors();
    
    while (1)
    {
        char c = pc.getc();
        if(c == 'u') {
            mspeed1 += 10;
            mspeed2 += 10;
            mspeed3 += 10;
            mspeed4 += 10;
            UpdateMotors(&mspeed1, &mspeed2, &mspeed3, &mspeed4);
            pc.printf("%i us\r\n", mspeed1);
        }
        
        if(c == 'd') {
            mspeed1 -= 10;
            mspeed2 -= 10;
            mspeed3 -= 10;
            mspeed4 -= 10;
            UpdateMotors(&mspeed1, &mspeed2, &mspeed3, &mspeed4);
            pc.printf("%i us\r\n", mspeed1);
        } 
    }
}
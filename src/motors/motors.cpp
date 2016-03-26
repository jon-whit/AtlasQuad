#include "motors.h"

PwmOut esc1(D7); // PWM 1/1
PwmOut esc2(D8); // PWM 1/2
PwmOut esc3(D5); // PWM 3/1
PwmOut esc4(D4); // PWM 3/2

void InitMotors(void)
{
    // Set the ESC update rate to 490 Hz
    esc1.period((float) 1/490);
    esc2.period((float) 1/490);
    esc3.period((float) 1/490);
    esc4.period((float) 1/490);
    
    // Set the output to the minimum
    esc1.pulsewidth_us(MOTOR_MIN);
    esc2.pulsewidth_us(MOTOR_MIN);
    esc3.pulsewidth_us(MOTOR_MIN);
    esc4.pulsewidth_us(MOTOR_MIN);
}

void ArmMotors(void)
{
    esc1.pulsewidth_us(MOTOR_MIN);
    esc2.pulsewidth_us(MOTOR_MIN);
    esc3.pulsewidth_us(MOTOR_MIN);
    esc4.pulsewidth_us(MOTOR_MIN);
}

void UpdateMotors(int* mspeed1, int* mspeed2, int* mspeed3, int* mspeed4)
{
    if (*mspeed1 < MOTOR_MIN)
        *mspeed1 = MOTOR_MIN;
    if (*mspeed1 > MOTOR_MAX)
        *mspeed1 = MOTOR_MAX;
    esc1.pulsewidth_us(*mspeed1);
    
    if (*mspeed2 < MOTOR_MIN)
        *mspeed2 = MOTOR_MIN;
    if (*mspeed2 > MOTOR_MAX)
        *mspeed2 = MOTOR_MAX;
    esc2.pulsewidth_us(*mspeed2);
    
    if (*mspeed3 < MOTOR_MIN)
        *mspeed3 = MOTOR_MIN;
    if (*mspeed3 > MOTOR_MAX)
        *mspeed3 = MOTOR_MAX;
    esc3.pulsewidth_us(*mspeed3);
    
    if (*mspeed4 < MOTOR_MIN)
        *mspeed4 = MOTOR_MIN;
    if (*mspeed4 > MOTOR_MAX)
        *mspeed4 = MOTOR_MAX;
    esc4.pulsewidth_us(*mspeed4);
}
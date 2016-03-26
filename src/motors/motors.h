#ifndef MOTORS_H
#define MOTORS_H
#include "mbed.h"

#define MOTOR_MIN 1000
#define MOTOR_MID 1450
#define MOTOR_MAX 1900

void InitMotors(void);
void ArmMotors(void);
void UpdateMotors(int*, int*, int*, int*);

#endif
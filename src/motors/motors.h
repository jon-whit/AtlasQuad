#ifndef MOTORS_H
#define MOTORS_H
#include "mbed.h"

#define MOTOR_MIN 1000
#define MOTOR_MID 1450
#define MOTOR_MAX 1900

/*
 * Initializes the ESCs onboard the quadcopter.
 */
void InitMotors(void);

/*
 * Reset the ESCs to arm them and prepare them for flight mode.
 */
void ArmMotors(void);

/*
 * Set the pulse width of each ESC to change the motor speeds.
 */
void UpdateMotors(int*, int*, int*, int*);

#endif
#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "MK64F12.h"
#include "uart.h"
#include "stdio.h"

#define CLOCK						20485760u
#define DC_FREQUENCY		10000
#define SERVO_FREQUENCY 50.0f
#define FTM0_MOD_VALUE	(CLOCK / DC_FREQUENCY)
#define FTM3_MOD_VALUE	(CLOCK/SERVO_FREQUENCY)

#define RIGHT		8.1f
#define LEFT 		3.7f
#define STRAIGHT 	6.1f

#define CENTER 64.0f

#define FORWARD 	0
#define BACKWARD 	1

// PID Stuff
#define K_P 	0.0800f
#define K_I		0.0000f
#define K_D		0.1500f // .2

#define LEFT_BOUND 	4.7f
#define RIGHT_BOUND	7.8f	

// Tolerance
#define STRAIGHT_TOL 	0.70f
#define TURN_TOL			0.80f

#define SENSE_LEFT		4.80f
#define SENSE_RIGHT		5.10f

// Speed Boost
#define ACC				2	
#define VEL				47
#define VEL_CAP		53

#define SLOW_DOWN	40

void AdjustWheels(int dL, int dR);

void ApplyPID(float desired, int L, int R, float last, float e1, float e2, float* e0, float* servo_pos);

void SetDutyCycleLeft(unsigned int dc, int dir);
void SetDutyCycleRight(unsigned int dc, int dir);
void SetDutyCycleDC(unsigned int DutyCycle, int dir);
void SetDutyCycleServo(float DutyCycle);

void InitPWM(void);
void FTM0_IRQHandler(void);
void FTM3_IRQHandler(void);

#endif

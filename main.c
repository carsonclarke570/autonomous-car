/*
 * Main Method for testing the PWM Code for the K64F
 * PWM signal can be connected to output pins are PC3 and PC4
 * 
 * Author:  
 * Created:  
 * Modified:  
 */

#include "MK64F12.h"
#include "uart.h"
#include "motor.h"
#include "camera.h"
#include "stdio.h"
#include "string.h"
#include "math.h"

#define CALIBRATE 0

void Initialize(void);
void delay(int del);

void mode_1(void);
void mode_2(void);

extern int capcnt;
extern uint16_t line[128];

char strx[100];

int main(void) {
	// Initialize UART and PWM
	Initialize();
	
	// Print welcome over serial
	BTSend("Running... \n\r");
	
	int mode = 0;
	
	while ((GPIOC_PDIR & (1UL << 6)) != 0 && (GPIOA_PDIR & (1UL << 4)) != 0) {
		if ((GPIOC_PDIR & (1UL << 6)) == 0) {
			mode = 0;
		}
		if ((GPIOA_PDIR & (1UL << 4)) == 0) {
			mode = 1;
		}
	}
	
	if (mode == 0) {
		mode_1();
	} else if (mode == 1) {
		mode_2();
	}

	return 0;
}

// Safety mode
void mode_1(void) {
	// Signal buffers
	int16_t buffer[LINE_SIZE];
	uint16_t signal[LINE_SIZE];
	
	int dL, dR;
	float last;
	float e1;
	float e2;
	float e0;
	float servo_pos;
	float mean, std_dev; 
	int l_cutoff, r_cutoff, control, dc;
	char x;
	
	delay(150);
	
	while (CALIBRATE) {
		SetDutyCycleServo(STRAIGHT);
		delay(100);
		SetDutyCycleServo(LEFT_BOUND);
		delay(100);
		SetDutyCycleServo(RIGHT_BOUND);
		delay(100);
		
		SetDutyCycleLeft(SLOW_DOWN + 35, FORWARD);
		SetDutyCycleRight(0, FORWARD);
		delay(100);
		
		SetDutyCycleLeft(0, FORWARD);
		SetDutyCycleRight(SLOW_DOWN + 35, FORWARD);
		delay(100);
		
		SetDutyCycleDC(0, FORWARD);
	}
		
	while (!CALIBRATE) {
		last = 0.0f;
		e1 = 0.0f;
		e2 = 0.0f;
		e0 = 0.0f;
		servo_pos = 0.0f;
		
		// Velocity control
		dc = VEL;
		
		control = 1;
		while(control) {
			if (capcnt > 0) {
				
				// Copy signal to work with
				memcpy(signal, line, sizeof(uint16_t) * LINE_SIZE);
				
				// Condition signal and extract DL and DR
				Condition(signal, buffer, LINE_SIZE);
				GetStats(buffer, LINE_SIZE, &dL, &dR, &mean, &std_dev);
				
				l_cutoff = (int) fabs(mean + (SENSE_LEFT * std_dev));
				r_cutoff = (int) fabs(mean + (SENSE_RIGHT * std_dev));
				
				if (0) {
					sprintf(strx, "%d - %d - %d - %d\r\n", l_cutoff, r_cutoff, buffer[dL], buffer[dR]);
					BTSend(strx);
				}
				
				if (-l_cutoff < buffer[dL] && r_cutoff > buffer[dR]) { // NO EDGES DETECTED!
					ApplyPID(64, 64, 64, last, e1, e2, &e0, &servo_pos);
					e2 = e1;
					e1 = e0;
					last = servo_pos;
				} else { // Apply PID control */
					ApplyPID(64, dL, dR, last, e1, e2, &e0, &servo_pos);
					e2 = e1;
					e1 = e0;
					last = servo_pos;
				}
				
				// Straight - Accelerate
				if (fabs(servo_pos) < STRAIGHT_TOL) {
					dc += ACC;
					if (dc > VEL_CAP) {
						dc = VEL_CAP;
					}
					SetDutyCycleDC(dc, FORWARD);
				} else {
					if (servo_pos > 0) {
							SetDutyCycleLeft(SLOW_DOWN + 30, FORWARD);
							SetDutyCycleRight(0, FORWARD);
					} else {
							SetDutyCycleLeft(0, FORWARD);
							SetDutyCycleRight(SLOW_DOWN + 30, FORWARD);
					}
				} 
				
				// Restart loop
				capcnt = 0;
			}
			
			if (UART3_S1 & UART_S1_RDRF_MASK) {
				x = UART3_D;
				if (x == 's') {
					SetDutyCycleDC(0, FORWARD);
				}
				control = 0;
			}
		}
	}
}

void mode_2(void) {
	// Signal buffers
	int16_t buffer[LINE_SIZE];
	uint16_t signal[LINE_SIZE];
	
	int dL, dR;
	float last;
	float e1;
	float e2;
	float e0;
	float servo_pos;
	float mean, std_dev; 
	int l_cutoff, r_cutoff, control, dc;
	char x;
	
	delay(150);
	
	while (CALIBRATE) {
		SetDutyCycleServo(STRAIGHT);
		delay(100);
		SetDutyCycleServo(LEFT_BOUND);
		delay(100);
		SetDutyCycleServo(RIGHT_BOUND);
		delay(100);
		
		SetDutyCycleLeft(SLOW_DOWN + 35, FORWARD);
		SetDutyCycleRight(0, FORWARD);
		delay(100);
		
		SetDutyCycleLeft(0, FORWARD);
		SetDutyCycleRight(SLOW_DOWN + 35, FORWARD);
		delay(100);
		
		SetDutyCycleDC(0, FORWARD);
	}
		
	while (!CALIBRATE) {
		last = 0.0f;
		e1 = 0.0f;
		e2 = 0.0f;
		e0 = 0.0f;
		servo_pos = 0.0f;
		
		// Velocity control
		
		
		control = 1;
		while(control) {
			if (capcnt > 0) {
				dc = VEL + 15;
				
				// Copy signal to work with
				memcpy(signal, line, sizeof(uint16_t) * LINE_SIZE);
				
				// Condition signal and extract DL and DR
				Condition(signal, buffer, LINE_SIZE);
				GetStats(buffer, LINE_SIZE, &dL, &dR, &mean, &std_dev);
				
				l_cutoff = (int) fabs(mean + (SENSE_LEFT * std_dev));
				r_cutoff = (int) fabs(mean + (SENSE_RIGHT * std_dev));
				
				if (0) {
					sprintf(strx, "%d - %d - %d - %d\r\n", l_cutoff, r_cutoff, buffer[dL], buffer[dR]);
					BTSend(strx);
				}
				
				if (-l_cutoff < buffer[dL] && r_cutoff > buffer[dR]) { // NO EDGES DETECTED!
					ApplyPID(64, 64, 64, last, e1, e2, &e0, &servo_pos);
					e2 = e1;
					e1 = e0;
					last = servo_pos;
				} else { // Apply PID control */
					ApplyPID(64, dL, dR, last, e1, e2, &e0, &servo_pos);
					e2 = e1;
					e1 = e0;
					last = servo_pos;
				}
				
				// Straight - Accelerate
				if (fabs(servo_pos) < STRAIGHT_TOL) {
					dc += ACC;
					if (dc > (VEL_CAP + 3)) {
						dc = (VEL_CAP + 3);
					}
					SetDutyCycleDC(dc, FORWARD);
				} else {
					if (servo_pos > 0) {
							SetDutyCycleLeft(SLOW_DOWN + 35, FORWARD);
							SetDutyCycleRight(0, FORWARD);
					} else {
							SetDutyCycleLeft(0, FORWARD);
							SetDutyCycleRight(SLOW_DOWN + 35, FORWARD);
					}
				} 
				
				// Restart loop
				capcnt = 0;
			}
			
			if (UART3_S1 & UART_S1_RDRF_MASK) {
				x = UART3_D;
				if (x == 's') {
					SetDutyCycleDC(0, FORWARD);
				}
				control = 0;
			}
		}
	}
}

void delay(int del) {
	int i;
	for (i=0; i<del*50000; i++){
		// Do nothing
	}
}

void Initialize() {
	InitUART();	
	InitPWM();
	InitGPIO();
	InitFTM2();
	InitADC0();
	InitPIT();
}

#ifndef _CAMERA_H_
#define _CAMERA_H_

#include "MK64F12.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
#include "uart.h"

#define DEFAULT_SYSTEM_CLOCK 20485760u
#define PERIOD (1 / DEFAULT_SYSTEM_CLOCK)
#define INTEGRATION_TIME .0075f

#define LINE_SIZE 128

#define TOL_L 3000
#define TOL_R	-TOL_L

void ConvolveUS(uint16_t* x, double* h, int16_t* y, int xsize, int hsize);
void ConvolveUU(uint16_t* x, double* h, uint16_t* y, int xsize, int hsize);
void ConvolveSS(int16_t* x, double* h, int16_t* y, int xsize, int hsize);
void Condition(uint16_t* signal, int16_t* buffer, int size);
void GetStats(int16_t* signal, int size, int* dl, int* dr, float* mean, float* std_dev);

void InitFTM2(void);
void InitGPIO(void);
void InitPIT(void);
void InitADC0(void);

void FTM2_IRQHandler(void);
void PIT0_IRQHandler(void);
void ADC0_IRQHandler(void);

#endif

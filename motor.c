#include "motor.h"

void ApplyPID(float desired, int L, int R, float last, float e1, float e2, float* e0, float* servo_pos) {
	
	// Calculate error
	float e = desired - ((float)(L + R) / 2.0f);
	
	// Calculate new servo position
	float correction = last +
		(K_P * (e - e1)) +							// Porprotional term
		(K_I * ((e + e1) / 2.0f)) +			// Integral term
		(K_D * (e - (2.0f * e1) + e2));	// Derivative term
	
	float dc = STRAIGHT + correction;
	if (dc > RIGHT_BOUND) {
		dc = RIGHT_BOUND;
	} else if (dc < LEFT_BOUND) {
		dc = LEFT_BOUND;
	}
	
	SetDutyCycleServo(dc);
	
	*e0 = e;
	*servo_pos = correction;
}

void SetDutyCycleDC(unsigned int DutyCycle, int dir) {
	uint16_t mod = (uint16_t) ((FTM0_MOD_VALUE * DutyCycle) / 100);
  
	if (dir==1) {
		FTM0_C3V = mod; FTM0_C2V=0;
		FTM0_C0V = mod; FTM0_C1V=0;
	} else {
		FTM0_C2V = mod; FTM0_C3V=0;
		FTM0_C1V = mod; FTM0_C0V=0;
  }

	FTM0_MOD = FTM0_MOD_VALUE;
}

void SetDutyCycleLeft(unsigned int dc, int dir) {
	uint16_t mod = (uint16_t) ((FTM0_MOD_VALUE * dc) / 100);
  
	if (dir==1) {
		FTM0_C0V = mod; FTM0_C1V=0;
	} else {
		FTM0_C1V = mod; FTM0_C0V=0;
  }

	FTM0_MOD = FTM0_MOD_VALUE;
}

void SetDutyCycleRight(unsigned int dc, int dir) {
	uint16_t mod = (uint16_t) ((FTM0_MOD_VALUE * dc) / 100);
  
	if (dir==1) {
		FTM0_C3V = mod; FTM0_C2V=0;
	} else {
		FTM0_C2V = mod; FTM0_C3V=0;
  }

	FTM0_MOD = FTM0_MOD_VALUE;
}

void SetDutyCycleServo(float DutyCycle) {
	uint16_t mod = (uint16_t) (((FTM3_MOD_VALUE / 8) * DutyCycle) / 100.0f);
  
	FTM3_C4V = mod;

	FTM3_MOD = FTM3_MOD_VALUE / 8;
}

void InitPWM() {
	SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;
	SIM_SCGC3 |= SIM_SCGC3_FTM3_MASK;
	
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK;
	
	PORTC_PCR3  = PORT_PCR_MUX(4)  | PORT_PCR_DSE_MASK; //Ch2
	PORTC_PCR4  = PORT_PCR_MUX(4)  | PORT_PCR_DSE_MASK;//Ch3
	
	PORTC_PCR1  = PORT_PCR_MUX(4)  | PORT_PCR_DSE_MASK;//Ch0
	PORTC_PCR2  = PORT_PCR_MUX(4)  | PORT_PCR_DSE_MASK;//Ch1
	
	PORTC_PCR8 = PORT_PCR_MUX(3) | PORT_PCR_DSE_MASK;
	
	FTM0_MODE |= FTM_MODE_WPDIS_MASK;
	FTM3_MODE |= FTM_MODE_WPDIS_MASK;
	
	FTM0_CNT = 0;
	FTM3_CNT = 0;
	
	FTM0_CNTIN = 0;
	FTM3_CNTIN = 0;
	
	FTM0_MOD = FTM0_MOD_VALUE;
	FTM3_MOD = FTM3_MOD_VALUE / 8;
	
	FTM0_C3SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM0_C3SC &= ~FTM_CnSC_ELSA_MASK;
   
	FTM0_C2SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM0_C2SC &= ~FTM_CnSC_ELSA_MASK;

	FTM0_C0SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM0_C0SC &= ~FTM_CnSC_ELSA_MASK;
 
	FTM0_C1SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM0_C1SC &= ~FTM_CnSC_ELSA_MASK;
	
	FTM3_C4SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM3_C4SC &= ~FTM_CnSC_ELSA_MASK;

	FTM0_SC = FTM_SC_PS(0) | FTM_SC_CLKS(1);
	FTM0_SC |= FTM_SC_TOIE_MASK;
	
	FTM3_SC = FTM_SC_PS(3) | FTM_SC_CLKS(1);
	FTM3_SC |= FTM_SC_TOIE_MASK;

  NVIC_EnableIRQ(FTM0_IRQn);
	NVIC_EnableIRQ(FTM3_IRQn);
}

void FTM0_IRQHandler(void) { 
  FTM0_SC &= ~FTM_SC_TOF_MASK;
}

void FTM3_IRQHandler(void) { 
  FTM3_SC &= ~FTM_SC_TOF_MASK;
}

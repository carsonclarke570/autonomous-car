#include "camera.h"

int pixcnt = -2;
int clkval = 0;
uint16_t line[LINE_SIZE];
uint16_t ADC0VAL;

int debugcamdata = 1;
int capcnt = 0;
char str[100];

void ConvolveUS(uint16_t* x, double* h, int16_t* y, int xsize, int hsize) {
	int half = hsize / 2;
	for (int i = half; i < xsize - half; i++) {
		double sum = 0.0f;
		for (int j = 0; j < hsize; j++) {
			sum += h[j] * (double) x[i-half+j];
		}
		y[i] = (int16_t) sum;
	}
}

void ConvolveUU(uint16_t* x, double* h, uint16_t* y, int xsize, int hsize) {
	int half = hsize / 2;
	for (int i = half; i < xsize - half; i++) {
		double sum = 0.0f;
		for (int j = 0; j < hsize; j++) {
			sum += h[j] * (double) x[i-half+j];
		}
		y[i] = (uint16_t) sum;
	}
}

void ConvolveSS(int16_t* x, double* h, int16_t* y, int xsize, int hsize) {
	int half = hsize / 2;
	for (int i = half; i < xsize - half; i++) {
		double sum = 0.0f;
		for (int j = 0; j < hsize; j++) {
			sum += h[j] * (double) x[i-half+j];
		}
		y[i] = (int16_t) sum;
	}
}

void Condition(uint16_t* signal, int16_t* buffer, int size) {
	// Smooth
	
	double smooth_kernel[7] = { 1.0f / 7.0f, 1.0f / 7.0f, 
								1.0f / 7.0f, 1.0f / 7.0f,
								1.0f / 7.0f, 1.0f / 7.0f,
								1.0f / 7.0f };
	{
		uint16_t buf[128];
		ConvolveUU(signal, smooth_kernel, buf, size, 7);
						
		// Derivative
		double deriv_kernel[3] = { 1.0f, 0, -1.0f };
		ConvolveUS(buf, deriv_kernel, buffer, size, 3);
	}
	
	for (int i = 0; i < 5; i++) {
		buffer[i] = 0;
		buffer[LINE_SIZE - i - 1] = 0;
	}
}

void GetStats(int16_t* signal, int size, int* dl, int* dr, float* mean, float* std_dev) {
	int16_t min = 30000;
	int16_t max = -30000;
	int16_t x;
	
	int sum = 0;
	
	for (int i = 5; i < size - 5; i++) {
		x = signal[i];
		sum += x;
		
		if (x < min) {
			min = x;
			*dl = i;
		}
		
		if (x > max) {
			max = x;
			*dr = i;
		}
	}
	
	float m = (float)sum / (float)(size - 10);
	
	float s = 0.0f;
	for (int i = 5; i < size - 5; i++) {
		s += fabs(signal[i] - m);
	}
	
	*mean = m;
	*std_dev = s / (float) (size - 10);
}

/* ADC0 Conversion Complete ISR  */
void ADC0_IRQHandler(void) {
	// Reading ADC0_RA clears the conversion complete flag
	ADC0VAL = ADC0_RA;
}

/*
* FTM2 handles the camera driving logic
*	This ISR gets called once every integration period
*		by the periodic interrupt timer 0 (PIT0)
*	When it is triggered it gives the SI pulse,
*		toggles clk for 128 cycles, and stores the line
*		data from the ADC into the line variable
*/
void FTM2_IRQHandler(void){ //For FTM timer
	// Clear interrupt
  FTM2_SC &= ~(FTM_SC_TOF_MASK);

	// Toggle clk
	clkval = !clkval;
	if (clkval) 
		GPIOB_PSOR |= (1 << 9); // CLK = 1
	else
		GPIOB_PCOR |= (1 << 9); // CLK = 0

	// Line capture logic
	if ((pixcnt >= 2) && (pixcnt < 256)) {
		if (!clkval) {	// check for falling edge
			// ADC read (note that integer division is
			//  occurring here for indexing the array)
			line[pixcnt/2] = ADC0VAL;
		}
		pixcnt += 1;
	} else if (pixcnt < 2) {
		if (pixcnt == -1) {
			GPIOB_PSOR |= (1 << 23); // SI = 1
		} else if (pixcnt == 1) {
			GPIOB_PCOR |= (1 << 23); // SI = 0
			// ADC read
			line[0] = ADC0VAL;
		}
		pixcnt += 1;
	} else {
		GPIOB_PCOR |= (1 << 9); // CLK = 0
		clkval = 0; // make sure clock variable = 0
		pixcnt = -2; // reset counter
		// Disable FTM2 interrupts (until PIT0 overflows
		//   again and triggers another line capture)
		FTM2_SC &= ~(FTM_SC_TOIE_MASK);
	}
	return;
}

/* PIT0 determines the integration period
*		When it overflows, it triggers the clock logic from
*		FTM2. Note the requirement to set the MOD register
* 	to reset the FTM counter because the FTM counter is
*		always counting, I am just enabling/disabling FTM2
*		interrupts to control when the line capture occurs
*/
void PIT0_IRQHandler(void){
	if (debugcamdata) {
		// Increment capture counter so that we can only
		//	send line data once every ~2 seconds
		capcnt += 1;
	}
	
	// Clear interrupt
	PIT_TFLG0 |= (PIT_TFLG_TIF_MASK);

	// Setting mod resets the FTM counter
	FTM2->MOD = DEFAULT_SYSTEM_CLOCK / 100000;

	// Enable FTM2 interrupts (camera)
	FTM2_SC |= FTM_SC_TOIE_MASK;

	return;
}


/* Initialization of FTM2 for camera */
void InitFTM2(){
	// Enable clock
	SIM_SCGC6 |= SIM_SCGC6_FTM2_MASK;

	// Disable Write Protection
	FTM2_MODE |= FTM_MODE_WPDIS_MASK;

	// Set output to '1' on init
	FTM2_OUTINIT |= FTM_OUTINIT_CH0OI_MASK;

	// Initialize the CNT to 0 before writing to MOD
	FTM2_CNT &= ~(FTM_CNT_COUNT_MASK);

	// Set the Counter Initial Value to 0
	FTM2_CNTIN &= ~(FTM_CNTIN_INIT_MASK);

	// Set the period (~10us)
	FTM2->MOD = (DEFAULT_SYSTEM_CLOCK)/ 100000;

	// 50% duty
	FTM2_C0V = (DEFAULT_SYSTEM_CLOCK)/ 200000;

	// Set edge-aligned mode
	FTM2_QDCTRL &= ~FTM_QDCTRL_QUADEN_MASK;
	FTM2_COMBINE &= ~(FTM_COMBINE_DECAPEN0_MASK | FTM_COMBINE_COMBINE0_MASK);
	FTM2_SC &= ~FTM_SC_CPWMS_MASK;
	FTM2_C0SC |= FTM_CnSC_MSB_MASK;

	// Enable High-true pulses
	// ELSB = 1, ELSA = 0
	FTM2_C0SC |= FTM_CnSC_ELSB_MASK;
	FTM2_C0SC &= ~(FTM_CnSC_ELSA_MASK);

	// Enable hardware trigger from FTM2
	FTM2_EXTTRIG |= FTM_EXTTRIG_INITTRIGEN_MASK;

	// Don't enable interrupts yet (disable)
	FTM2_SC &= ~(FTM_SC_TOIE_MASK);

	// No prescalar, system clock
	FTM2_SC &= ~(FTM_SC_PS_MASK); // Set prescaler 1
	FTM2_SC |= FTM_SC_CLKS(01); // Select system clock

	// Set up interrupt
	FTM2_SC |= FTM_SC_TOIE_MASK;
	
	NVIC_EnableIRQ(FTM2_IRQn);

	return;
}

/* Initialization of PIT timer to control
* 		integration period
*/
void InitPIT(void){
	// Setup periodic interrupt timer (PIT)
	// Do we need PIT_MCR &= ~(PIT_MCR_MDIS_MASK);????
	
	// Enable clock for timers
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;

	// Enable timers to continue in debug mode
	PIT_MCR &= ~(PIT_MCR_FRZ_MASK | PIT_MCR_MDIS_MASK); // In case you need to debug

	// PIT clock frequency is the system clock
	// Load the value that the timer will count down from
	PIT_LDVAL0 = DEFAULT_SYSTEM_CLOCK * INTEGRATION_TIME;

	// Enable timer interrupts
	PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;

	// Enable the timer
	PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;

	// Clear interrupt flag
	PIT_TFLG0 |= (PIT_TFLG_TIF_MASK);

	// Enable PIT interrupt in the interrupt controller
	NVIC_EnableIRQ(PIT0_IRQn);
	
	return;
}


/* Set up pins for GPIO
* 		PTB9 		- camera clk
*		PTB23		- camera SI
*		PTB22		- red LED
*/
void InitGPIO(void){
	// Enable LED and GPIO so we can see results
	
	// Enable clocks on Ports B for camera clk, SI and LED timing
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
	
	// Enable motors
	PORTB_PCR3 |= PORT_PCR_MUX(1);
	PORTB_PCR2 |= PORT_PCR_MUX(1);
	PORTC_PCR6 |= PORT_PCR_MUX(1);
	PORTA_PCR4 &= ~(PORT_PCR_MUX_MASK);
	PORTA_PCR4 |= PORT_PCR_MUX(1);
	
	GPIOB_PDDR |= (1 << 3) | (1 << 2);
	GPIOB_PSOR = (1 << 3) | (1 << 2);
	
	// Configure the Signal Multiplexer for GPIO
	PORTB_PCR9 |= PORT_PCR_MUX(1); //camera clk
	PORTB_PCR23 |= PORT_PCR_MUX(1); //camera SI
	
	// Switch the GPIO pins to output mode
	GPIOB_PDDR |= (1 << 9) | (1 << 23);
	GPIOC_PDDR &= ~(1 << 6);
	GPIOA_PDDR &= ~(1 << 4);
	
	return;
}

/* Set up ADC for capturing camera data */
void InitADC0(void) {
    unsigned int calib;
    // Turn on ADC0
    SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;

		// Single ended 16 bit conversion, no clock divider
		ADC0_CFG1 &= ~ADC_CFG1_ADIV_MASK;
		ADC0_CFG1 |= ADC_CFG1_MODE_MASK;

    // Do ADC Calibration for Singled Ended ADC. Do not touch.
    ADC0_SC3 = ADC_SC3_CAL_MASK;
    while ( (ADC0_SC3 & ADC_SC3_CAL_MASK) != 0 );
    calib = ADC0_CLP0; calib += ADC0_CLP1; calib += ADC0_CLP2;
    calib += ADC0_CLP3; calib += ADC0_CLP4; calib += ADC0_CLPS;
    calib = calib >> 1; calib |= 0x8000;
    ADC0_PG = calib;

    // Select hardware trigger.
    ADC0_SC2 |= ADC_SC2_ADTRG_MASK;

    // Set to single ended mode
		ADC0_SC1A = 0;
		ADC0_SC1A &= ~(ADC_SC1_DIFF_MASK);
		ADC0_SC1A |= ADC_SC1_AIEN_MASK;
		ADC0_SC1A |= ADC_SC1_ADCH(0x00);
		
		// Set up FTM2 trigger on ADC0
		SIM_SOPT7 |= SIM_SOPT7_ADC0TRGSEL(0xA);	// FTM2 select
		SIM_SOPT7 |= SIM_SOPT7_ADC0ALTTRGEN_MASK; // Alternative trigger en.
		SIM_SOPT7 &= ~(SIM_SOPT7_ADC0PRETRGSEL_MASK); // Pretrigger A

		// Enable NVIC interrupt
  	NVIC_EnableIRQ(ADC0_IRQn);
}

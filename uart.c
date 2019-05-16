/*
 * File:        uart.c
 * Purpose:     Provide UART routines for serial IO
 *
 * Notes:		
 *
 */

#include "uart.h"

void InitUART() {
	uint16_t ubd, brfa;

	SIM_SCGC5 |= (SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK);
	SIM_SCGC4 |= (SIM_SCGC4_UART0_MASK | SIM_SCGC4_UART3_MASK);
 
	PORTB_PCR16 |= PORT_PCR_MUX(3); //set transmitter pcr
	PORTB_PCR17 |= PORT_PCR_MUX(3); //set receiver pcr
	
	PORTC_PCR16 |= PORT_PCR_MUX(3);
	PORTC_PCR17 |= PORT_PCR_MUX(3);

	UART0_C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
	UART0_C1 = 0x00;
	
	UART3_C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
	UART3_C1 = 0x00;

	ubd = (uint16_t)((SYS_CLOCK)/(BAUD_RATE * 16));
	UART0_BDH &= ~(UART_BDH_SBR_MASK << UART_BDH_SBR_SHIFT);
	UART0_BDH |= UART_BDH_SBR(ubd >> 8);
	UART0_BDL = UART_BDL_SBR(ubd);
	
	UART3_BDH &= ~(UART_BDH_SBR_MASK << UART_BDH_SBR_SHIFT);
	UART3_BDH |= UART_BDH_SBR(ubd >> 8);
	UART3_BDL = UART_BDL_SBR(ubd);

	brfa = (((SYS_CLOCK*32)/(BAUD_RATE * 16)) - (ubd * 32));
	UART0_C4 |= UART_C4_BRFA(brfa);
	UART0_C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);
	
	UART3_C4 |= UART_C4_BRFA(brfa);
	UART3_C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);
}

char GetChar() {
	while(!(UART0_S1 & UART_S1_RDRF_MASK));

	return UART0_D;
}

void PutChar(char ch) {
	while(!(UART0_S1 & UART_S1_TDRE_MASK));
	
	UART0_D = ch;
}

void Put(char *ptr_str)
{
	while(*ptr_str)
		PutChar(*ptr_str++);
}

void PutNumU(unsigned long int i) {
	static char str[32] = {0};
	int x = 30;
	for(; i && x; --x, i /= 10) {
		str[x] = "0123456789"[i % 10];
	}
	Put(&str[x+1]);
}

void BTSendChar(char ch) {
	while(!(UART3_S1 & UART_S1_TDRE_MASK));
	
	UART3_D = ch;
}

void BTSend(char* str) {
	while(*str) 
		BTSendChar(*str++);
}

char BTRecieve() {
	while (!(UART3_S1 & UART_S1_RDRF_MASK));

	return UART3_D;
}

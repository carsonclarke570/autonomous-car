#ifndef UART_H
#define UART_H

#include <stdint.h>
#include "MK64F12.h"
#define BAUD_RATE 9600      //default baud rate 
#define SYS_CLOCK 20485760 //default system clock (see DEFAULT_SYSTEM_CLOCK  in system_MK64F12.c)

void InitUART(void);

void Put(char *ptr_str );
void PutNumU(unsigned long int i);

char GetChar(void);
void PutChar(char ch);

void BTSendChar(char ch);
void BTSend(char* str);
char BTRecieve(void);

#endif  /*  ifndef  UART_H  */

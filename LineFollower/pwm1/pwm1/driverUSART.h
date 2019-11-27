#include <avr/io.h>
#include <avr/interrupt.h>

extern void USART_Init();
extern void sendStringUSART (char *s);
extern void transmitUSART(unsigned char data);
extern char receiveUSART();
extern void sendIntUSART(int digit);
extern void sendFloatUSART(float f);
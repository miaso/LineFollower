#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

#define F_CPU 16000000UL

#define USART_BAUDRATE 9600     // Baud Rate value

#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)





char receiveUSART(void)
{
	char received;
	/* Wait for data to be received */
	while ( !(UCSRA & (1<<RXC)));
	/* Get and return received data from buffer */
	
	received = UDR;

	return received;
}
/* -----------------------------------------------------
void usart_transmit( char data)
Waits for UDRE flag and writes to data parameter value
to UDR register. Sends data byte.
-----------------------------------------------------*/
void transmitUSART( char data)
{
	/* Wait for data to be transmitted */
	while ( !(UCSRA & (1<<UDRE)));

	UDR = data;
}
/* -----------------------------------------------------
void sendStringUSART (char *s)
Sends a string through USART. Shifts bytes of array
passed by *s pointer until terminator.
-----------------------------------------------------*/
void sendStringUSART (char *s)
{
	char c = 0;
	for (;(( c=*s)!=0);s++){
		transmitUSART(*s); 
	}
}

// Send an integer value converted to string through USART
void sendIntUSART(int digit){
	char str[15];
	sprintf(str, "%d", digit);
	sendStringUSART(str);
	sendStringUSART("\n");
}

/* -----------------------------------------------------
void sendStringUSART (char *s)
Enables USART receiver and transmitter, sets up frame
format: 1 stop bit, 8 data bits, no parity, full duplex.
Uses int baud parameter to set baud rate while casted.
-----------------------------------------------------*/
void USART_Init()
{
	UCSRB |= (1 << RXEN) | (1 << TXEN);   // Turn on the transmission and reception circuitry
	UCSRC |= (1 << URSEL) | (1<<USBS) | (1 << UCSZ0) | (1 << UCSZ1);
	// Use 8-bit character sizes
	
	UBRRL = BAUD_PRESCALE;
	// Load lower 8-bits of the baud rate value into the low byte of the UBRR register
	UBRRH = (BAUD_PRESCALE >> 8); // Load upper 8-bits of the baud rate value..
	// into the high byte of the UBRR register
	UCSRB |= (1 << RXCIE );
	
/*	UCSRA = (1<<U2X);

	UBRRL = 12;

	UBRRH = 0;
*/
}
void sendFloatUSART(float f){
	char st[15];
	float dd=(1.22);
	sprintf(st, "Value = %d", dd);
	//sendIntUSART(dd);
	//sendStringUSART(st);
}
/*
 * driverADC.c
 *
 * Created: 08-02-2015 18:32:39
 *  Author: IdeaPad
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "driverUSART.h"

void ADC_Init()
{

	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescaler to 128 - 72 KHz sample rate @ 10MHz

	ADMUX |= (1 << REFS0); // Set ADC reference to AVCC
	ADMUX |= (1 << ADLAR); // Left adjust ADC result to allow easy 8 bit reading

	// No MUX values needed to be changed to use ADC0

	ADCSRA |= (1 << ADATE);  // Set ADC to Free-Running Mode
	ADCSRA |= (1 << ADEN);  // Enable ADC

	ADCSRA |= (1 << ADIE);  // Enable ADC Interrupt
	sei();	// Enable Global Interrupts

	ADCSRA |= (1 << ADSC);  // Start A2D Conversions


}


ISR(ADC_vect)
{
			//char str[15];
			//sprintf(str, "%d", ADCH);
			
			//sendStringUSART(str);
			//sendStringUSART("\n");
	if(ADCH < 20)
	{
		

		
		OCR1A = 400;
		//OCR1A = 2000;
		OCR1B = 400;
	}
	
	else
	{
		//sendStringUSART(ADCH);
		OCR1A = 0;
		OCR1B = 0;
	}
}
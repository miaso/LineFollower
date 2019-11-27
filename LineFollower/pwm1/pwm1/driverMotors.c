

#define INPUT1 PC3
#define INPUT2 PC2
#define INPUT3 PC0
#define INPUT4 PC1
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "driverUSART.h"

void forwardAB()
{
	//A
	PORTC |= (1 << INPUT1);    // Pin 6 goes high = input 2=1
	PORTC &= ~(1 << INPUT2);  // Pin pd6 goes low = input 1=0
	//B
	PORTC |= (1 << INPUT3);    // Pin 6 goes high = input 2=1
	PORTC &= ~(1 << INPUT4);  // Pin pd6 goes low = input 1=0
	//sendStringUSART("both forward\n");
}
void backwardsAB()
{
	//Motor A Left
	PORTC |= (1 << INPUT2);    // Pin 6 goes high = input 2=1
	PORTC &= ~(1 << INPUT1);  // Pin pd6 goes low = input 1=0
	//Motor B right
	PORTC |= (1 << INPUT4);    // Pin 6 goes high = input 2=1
	PORTC &= ~(1 << INPUT3);  // Pin pd6 goes low = input 1=0
	//sendStringUSART("both back\n");  // Pin pd6 goes low = input 1=0
}

void rightOnly(){
	PORTC |=  (1 << INPUT3);    // Pin 6 goes high = input 2=1
	PORTC &= ~(1 << INPUT4);  // Pin pd6 goes low = input 1=0
	PORTC &= ~(1 << INPUT1);
	PORTC &= ~(1 << INPUT2);
	//sendStringUSART("right only\n");
}
void leftOnly(){
	PORTC |= (1 << INPUT1);    // Pin 6 goes high = input 2=1
	PORTC &= ~(1 << INPUT2);  // Pin pd6 goes low = input 1=0
	PORTC &= ~(1 << INPUT3);
	PORTC &= ~(1 << INPUT4);
	//sendStringUSART("left only\n");
}
void stopAB(){
	PORTC &= ~(1 << INPUT1);
	PORTC &= ~(1 << INPUT2);
	PORTC &= ~(1 << INPUT3);
	PORTC &= ~(1 << INPUT4);
	//sendStringUSART("both stop\n");
}

void stopB(){
	PORTC |= (1 << INPUT1); 
	PORTC |= (1 << INPUT2); 
}
void stopA(){
	PORTC |= (1 << INPUT3);
	PORTC |= (1 << INPUT4);
}
void forwardB(){
	PORTC |= (1 << INPUT1);    // Pin 6 goes high = input 2=1
	PORTC &= ~(1 << INPUT2);
}
void forwardA(){
	PORTC |= (1 << INPUT3);    // Pin 6 goes high = input 2=1
	PORTC &= ~(1 << INPUT4);
}
// S134616
#define kAccelStep 10
#define F_CPU 16000000UL
#include <avr/io.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "driverADC.h"
#include "driverUSART.h"
#include "driverMotors.h"

/*
* PIN name corresponding to the led
*/
#define LED_PIN PB2
/**
* \brief Toggles the LED that is present on port b pin 0.
*/
#define toggle_led()        {PORTB^=(1<<LED_PIN);}
/*
* Sets the LED on.
*/
#define led_on()            {PORTB&=~(1<<LED_PIN);}
/*
* Sets the LED off.
*/
#define led_off()            {PORTB|=(1<<LED_PIN);}
/*
* Inits the LED pin.
*/
#define LED_PIN_init()            {DDRB|=(1<<LED_PIN);}

enum{start, findLine, loop, junction, lost, stop,circle, follow,follow2,speed,wall,test,test2 }stage;

double P_GAIN=1;
double I_GAIN=0;
double D_GAIN=0;
////////////////////////////////////////////////////////////////////////////
double pGain = 4;   //Proportional Gain
double iGain =  0;  //Integral Gain                      SENSORS
double dGain = 0;  //Differential Gain
//////////////////////////////////////junction/////////////////////////////////////
volatile uint8_t count;

double previous_error ,previous_errorB = 0;
double set_PulsesRef=0,errorA,errorB,feedback_PulsesB, feedback_PulsesA ,set_PulsesA = 0, set_PulsesB = 0 ;
double D_error = 0,D_errorB=0,I_errorB, I_error = 0;
int output=0,outputB=0;

float sprev=0;
short c = 0;
double pulsesA;
double pulsesB;

double readSensors ();
float sensPID(double cur_value,int req_value);
void motorsPID();

//float  P_GAIN  = 0.5 ;

int delay = 10;

char lastPos ='s';

int32_t eInteg = 0;  //Integral accumulator
int32_t ePrev  =0;      //Previous Error

float control;
int s;
int flagA=0;
int flagB=0;
double pulsesBturn=0;
double pulsesAturn=0;
int counter=0;
int fifm=0;

int sensFull(){
	int sensor1 = ((PINA & (1<<PA1))/2);
	int sensor2 = ((PINA & (1<<PA2))/4);
	int sensor3 = ((PINA & (1<<PA3))/8);
	int sensor4 = ((PINA & (1<<PA4))/16);
	int sensor5 = ((PINA & (1<<PA5))/32);
	int sensor6 = ((PINA & (1<<PA6))/64);
	int sensor7 = ((PINA & (1<<PA7))/128);
	
	if     ((sensor1==0)&&(sensor2==0)&&(sensor3==0)&&(sensor4==0)&&(sensor5==0)&&(sensor6==0)&&(sensor7==0))
	{return 1;}
	else{
		return 0;
	}
	
}
int sensSpeed(){
	int sensor1 = ((PINA & (1<<PA1))/2);
	int sensor2 = ((PINA & (1<<PA2))/4);
	int sensor3 = ((PINA & (1<<PA3))/8);
	int sensor4 = ((PINA & (1<<PA4))/16);
	int sensor5 = ((PINA & (1<<PA5))/32);
	int sensor6 = ((PINA & (1<<PA6))/64);
	int sensor7 = ((PINA & (1<<PA7))/128);
	
	if     ((sensor1==0)&&(sensor2==0)&&(sensor3==0)&&(sensor4==0)&&(sensor5==0)&&(sensor6==1)&&(sensor7==1))
	{return 1;}
	else{
		return 0;
	}
	
}
void setBothMotorsRpm(int rpmToSet){
	set_PulsesRef=rpmToSet;
	set_PulsesA=set_PulsesRef;
	set_PulsesB=set_PulsesRef;
}

void btMode(){
	set_PulsesRef=10;
	set_PulsesA=0;
	set_PulsesB=set_PulsesRef;
}
/********************************/
void init_interrupts(){
	MCUCR |= (1 << ISC01)|(1 << ISC00);	// Rising Edge
	GICR |= (1 << INT0);			   // INT0 Enabled
	DDRD = 1<<PD2;					  // Set PD2 as input (Using for interupt INT0)
	PORTD = 1<<PD2;					 // Enable PD2 pull-up resistor
	sei();
}
void init_interrupts2(){
	MCUCR |= (1 << ISC11)|(1 << ISC10);  // Rising Edge
	GICR |= (1 << INT1);				// INT1 Enabled
	DDRD = 1<<PD3;					   // Set PD3 as input (Using for interrupt INT1)
	PORTD = 1<<PD3;					  // Enable PD3 pull-up resistor
	sei();
}
void init_io(){
	// Fast PWM mode - 16bit , Fixed frequency , Non- Inverting mode , TOP = ICR1 = 399
	// Frequency = F_CPU/(N*(1+TOP) =40000 Hz , N = 1
	// Duty cycle set by OCR1A and OCR1B
	TCCR1A = (1 << COM1A1) | (1 << WGM11 | (1<<COM1B1) )  ;
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10) ;
	ICR1 = 399;
	TCNT1L = 0;
	TCNT1H = 0;
	sendStringUSART("PWM Initialized");
}
void init_sensors(){
	DDRA &= ~(1 << PA1);        // set as inputs
	DDRA &= ~(1 << PA2);       //seredinka 3 , 4 pravyj, 5 levyj
	DDRA &= ~(1 << PA3);
	DDRA &= ~(1 << PA4);
	DDRA &= ~(1 << PA5);
	DDRA &= ~(1 << PA6);
	DDRA &= ~(1 << PA7);
	
}
void init_timer0(){
	TCCR0|=(1<<CS01)|(1<<CS00);       // Prescaler = FCPU/64=156250
	TIMSK|=(1<<TOIE0);				//Enable Overflow Interrupt Enable
	TCNT0=0;					//Initialize Counter
	count=0;				//Initialize the variable varriable
	sendStringUSART("timer 0 was initialized");
}
void init_timer2(){
	// Timer2 CTC mode Frequency = F_CPU/(2*N*(1+OCR2)) = 100 Hz
	// Set to interrupt after
	OCR2 = 78;									   // Set OCR2
	TCCR2 |= (1 << WGM21);						  // Set to CTC Mode
	TIMSK |= (1 << OCIE2);						 //Set interrupt on compare match
	TCCR2 |= ( 1<< CS20 |1 << CS21 | 1<< CS22); // set prescaler to 1024 and starts PWM
	sei();									   // enable interrupts
	sendStringUSART("\n Timer 2 was initialised");
	
}

void make90R(){

	flagB=1;
	while(pulsesBturn < 350){
		stopA();
		set_PulsesRef=10;
		set_PulsesA=0;
		set_PulsesB=set_PulsesRef;
	}
	flagB=0;
	pulsesBturn=0;
	stopAB();
	setBothMotorsRpm(0);
	//_delay_ms(500);
}
void make90L(){
	flagA=1;
	while(pulsesAturn < 330){
		stopB();
		set_PulsesRef=10;
		set_PulsesB=0;
		set_PulsesA=set_PulsesRef;
	}
	flagA=0;
	pulsesAturn=0;
	stopAB();
	setBothMotorsRpm(0);
}
void motorsPID (){
	//int k=OCR1A;
	//int b=OCR1B;
	static int16_t oca=0;
	static int16_t ocb=0;
	errorA = set_PulsesA - feedback_PulsesA;
	errorB = set_PulsesB - feedback_PulsesB;
	
	oca+=errorA*P_GAIN;
	ocb+=errorB*P_GAIN;
	// stopA();
	
	
	if (oca<0)
	{
		oca=0;
	}
	else if(oca>399)
	{
		oca=399;
	}
	
	if (ocb<0)
	{
		ocb=0;
	}
	else if(ocb>399)
	{
		ocb=399;
	}
	
	OCR1A=oca;
	OCR1B=ocb;
	if (errorB<0)
	{
		stopB();
		OCR1B=399;
		
		}else {
		forwardB();
	}

	if (errorA<0)
	{
		stopA();
		OCR1A=399;
		
		}else {
		forwardA();
	}

}


double readSensors (){
	
	int sensor1 = !((PINA & (1<<PA1))/2);
	int sensor2 = !((PINA & (1<<PA2))/4);
	int sensor3 = !((PINA & (1<<PA3))/8);
	int sensor4 = !((PINA & (1<<PA4))/16);
	int sensor5 = !((PINA & (1<<PA5))/32);
	int sensor6 = !((PINA & (1<<PA6))/64);
	int sensor7 = !((PINA & (1<<PA7))/128);
	
	double sum=  (sensor1*1) + (sensor2*2) + (sensor3*3) + (sensor4*4) + (sensor5*5) + (sensor6*6) + (sensor7*7);
	sum=sum/((sensor1) + (sensor2) + (sensor3) + (sensor4) + (sensor5) + (sensor6) + (sensor7));
	return sum;
}

//********************************//
float sensPID(double cur_value,int req_value)
{
	
	float pid;
	double errorSens;

	errorSens=cur_value-4;
	pid = (pGain * errorSens)  + (iGain * eInteg) + (dGain * (errorSens - ePrev));

	eInteg += errorSens;                  // integral is simply a summation over time
	ePrev = errorSens;                    // save previous for derivative

	
	if (pid<-25){
		pid=-25;
		}else if (pid>25){
		pid=25;
	}
	
	if(pid<0){
		set_PulsesA=set_PulsesRef;
		set_PulsesB=set_PulsesRef-(abs(pid));
	}
	else{
		set_PulsesB=set_PulsesRef;
		set_PulsesA=set_PulsesRef-(abs(pid));
	}
	return pid;
}

void followLine(){
	
	s=readSensors();
	sensPID(s,0);
	//motorsPID();
	//sprev=s;
	//_delay_ms(10);
}
void backManeuvr(){


}
void continueForward(){
	setBothMotorsRpm(set_PulsesRef);
	//motorsPID();
	
}
ISR (TIMER2_COMP_vect){
	
	TCNT2=0;  //reset timer
	c++;    // count up
	//

	// reset the countUp value
	if( 3 == c){
		feedback_PulsesA=pulsesA;
		feedback_PulsesB=pulsesB;
		//sendIntUSART(pulsesBturn);

		pulsesA=0;
		pulsesB=0;
		c=0;
		motorsPID();
		
		if(counter==1){
			fifm++;
		}
	}
	
	
}

ISR (INT0_vect){
	pulsesB++;
	if (flagB==1){
		pulsesBturn++;
	}
}
ISR (INT1_vect){
	pulsesA++;
	if (flagA==1){
		pulsesAturn++;
	}
}
ISR (USART_RXC_vect)
{
	char ReceivedByte ;
	ReceivedByte = UDR ; // Fetch the received byte value into the variable " ByteReceived "
	//UDR = ReceivedByte ; // Echo back the received byte back to the computer
	/* local variable definition */
	switch(ReceivedByte)
	{
		case 'w' :
		forwardAB();
		
		set_PulsesRef=10;
		set_PulsesA=set_PulsesRef;
		set_PulsesB=set_PulsesRef;
		//set_rpmA=set_rpmB;
		break;
		
		case 'z' :
		backwardsAB();
		break;
		
		case 'a' :
		leftOnly();
		break;
		
		case 'd' :
		rightOnly();
		break;
		
		case 's' :
		stopAB();
		break;
		
		case 'e' :

		set_PulsesB+=1;
		break;

		case 'q' :
		set_PulsesA+=1;
		
		break;
		
		case 't' :
		sendIntUSART(set_PulsesRef);
		sendStringUSART("L:");
		sendIntUSART(feedback_PulsesA);
		sendStringUSART("R:");
		sendIntUSART(feedback_PulsesB);
		
		break;
		
		case 'k' :
		
		set_PulsesRef+=kAccelStep;
		break;
		case 'l' :
		
		set_PulsesRef-=kAccelStep;
		break;
		////////////////////
		//pid tuning
		
		
		case '1' :
		P_GAIN=P_GAIN+1;
		break;
		case '2' :
		I_GAIN=I_GAIN+0.5;
		break;
		case '3' :
		D_GAIN=D_GAIN+0.01;
		break;
		
		case '4' :
		pGain=pGain-1;
		break;
		case '5' :
		iGain=iGain-0.5;
		break;
		case '6' :
		dGain=dGain-1;
		break;
		
		case '0' :
		sendIntUSART(P_GAIN);
		sendIntUSART(I_GAIN);
		sendIntUSART(D_GAIN);
		break;
		////////////////
		default :
		sendStringUSART("xunia");
		
		
		
	}
	UDR = ReceivedByte ;
	
	
}


void getToCirclePos()
{
	flagA=1;
	flagB=1;
}
void makeCircle()
{
	set_PulsesA=7;
	set_PulsesB=11;
}
int main(void){
	LED_PIN_init();
	USART_Init();
	//ADC_Init();
	init_sensors();
	init_io();
	init_interrupts2();
	init_interrupts();
	init_timer2();
	DDRD   = (1<<5)|(1<<4);  // output pins OC1A, OC1B   Mega16,32
	forwardAB();
	OCR1A = 0;
	OCR1B = 0;
	set_PulsesA=set_PulsesRef;
	set_PulsesB=set_PulsesRef;
	previous_error = set_PulsesA - feedback_PulsesA;
	previous_errorB= set_PulsesB - feedback_PulsesB;
	sendStringUSART("\n START ADJUST \n");
	stage=findLine;
	
	
	while (1)
	{
		switch(stage)
		{
			//////////////////////////////////////////////
			case start:
			if(readSensors()==4)
			{
				sendStringUSART("Lets find the line");
				forwardAB();
				
				stage=findLine;
				} else {stage=start;}
				break;
				//////////////////////////////////////////////
				case findLine:

				if(readSensors()==4){
					stopA();
					stopB();
					make90R();
					//backManeuvr();
					sendStringUSART("Found it!");
					stage=loop;
					}else{		set_PulsesRef=10;
					setBothMotorsRpm(set_PulsesRef);
				stage=findLine;}
				break;
				//////////////////////////////////////////////////////
				case loop:
				if (sensFull()==1)
				{

					stage=stop;
				}
				else
				{
					set_PulsesRef=10;
					followLine();
					stage=loop;
				}
				break;
				//////////////////////////////////////////////////////
				case stop:
				pulsesAturn=0;
				flagA=1;
				while (pulsesAturn<300)
				{
					forwardAB();
					set_PulsesRef=10;
					followLine();
					//set_PulsesRef=10;
					//setBothMotorsRpm(set_PulsesRef);
				}
				pulsesAturn=0;
				counter=1;
				while(fifm<128){
					stopA();stopB();
				}
				counter=0;
				fifm=0;
				

				make90R();
				counter=1;
				while(fifm<66){
					stopA();stopB();
				}
				counter=0;
				fifm=0;
				flagA=1;
				pulsesAturn=0;
				while (pulsesAturn<100)
				{
					backwardsAB();
					set_PulsesRef=5;
					setBothMotorsRpm(set_PulsesRef);
				}
				pulsesAturn=0;
				flagA=0;
				stage= circle;
				counter=1;
				break;
				//////////////////////////////////////////////////////
				case circle:
				//forwardAB();
				
				if(fifm<402){
					makeCircle();
				}else
				{
					makeCircle();
					if(sensFull()==1){
						make90L();
						counter=0;
						fifm=0;
						stage=follow2;
						
					}
				}
				
				
				//stage=follow;
				break;
				//////////////////////////////////////////////////////
				case follow:
				if (sensFull()==1)
				{
					
					stage=wall;
					}else{
					set_PulsesRef=10;
					followLine();
					
					stage=follow;
				}
				break;
				//////////////////////////////////////////////////////
						case follow2:
						if (sensFull()==1)
						{
											counter=1;
											while(fifm<126){
												set_PulsesRef=10;
												followLine();
											}
											counter=0;
											fifm=0;	
									
									
							stage=follow;
							}else{
							set_PulsesRef=10;
							followLine();
							
							stage=follow2;
							
						}
						break;
				////////////////////////////////////////////////////
				case wall:
				/////// wait 3 sec
				counter=1;
				while(fifm<198){
					stopA();stopB();
				}
				counter=0;
				fifm=0;
				///first forward
				pulsesAturn=0;
				flagA=1;
				while (pulsesAturn<500)
				{
					forwardAB();
					set_PulsesRef=10;
					setBothMotorsRpm(set_PulsesRef);
				}
				flagA=0;
				pulsesAturn=0;
				
				// first left
				pulsesAturn=0;
				counter=1;
				while(fifm<128){
					stopA();stopB();
				}
				counter=0;
				fifm=0;
				make90R();
				counter=1;
				while(fifm<128){
					stopA();stopB();
				}
				counter=0;
				fifm=0;
				/////// forward 2
								pulsesAturn=0;
								flagA=1;
								while (pulsesAturn<500)
								{
									forwardAB();
									set_PulsesRef=10;
									setBothMotorsRpm(set_PulsesRef);
								}
								flagA=0;
								pulsesAturn=0;
				/////// left turn
								pulsesAturn=0;
								counter=1;
								while(fifm<128){
									stopA();stopB();
								}
								counter=0;
								fifm=0;
								make90L();
								counter=1;
								while(fifm<128){
									stopA();stopB();
								}
								counter=0;
								fifm=0;
				////// more forward 
										pulsesAturn=0;
										flagA=1;
										while (pulsesAturn<500)
										{
											forwardAB();
											set_PulsesRef=10;
											setBothMotorsRpm(set_PulsesRef);
										}
										flagA=0;
										pulsesAturn=0;
										
			////////  LEFT MORE
										counter=0;
										fifm=0;
										

										make90L();
										counter=1;
										while(fifm<66){
											stopA();stopB();
										}
										counter=0;
										fifm=0;
				
				//////    	more forward
				pulsesAturn=0;
				flagA=1;
				while (pulsesAturn<300)
				{
					forwardAB();
					set_PulsesRef=10;	
					setBothMotorsRpm(set_PulsesRef);
				}
				pulsesAturn=0;		
				//// right  last time 
				pulsesAturn=0;
				counter=1;
				while(fifm<128){
					stopA();stopB();
				}
				counter=0;
				fifm=0;
				make90R();
				counter=1;
				while(fifm<66){
					stopA();stopB();
				}
				counter=0;
				fifm=0;
				
					//////    	more forward last time
					pulsesAturn=0;
					flagA=1;
					while (pulsesAturn<300)
					{
						forwardAB();
						set_PulsesRef=10;
						setBothMotorsRpm(set_PulsesRef);
					}
					pulsesAturn=0;
						
				stage=test;
				break;
				//////////////////////////////////////////////////////
				case test:

				counter=1;
				while(fifm<66){
					stopA();stopB();
				}
				counter=0;
				fifm=0;
				
				
				stage=test;
				break;
				//////////////////////////////////////////////
				case test2:
				btMode();
				stage=test;
				break;
				//////////////////////////////////////////////////////
				default:stage=loop; break;
				//////////////////////////////////////////////////////
			}
		}
	}
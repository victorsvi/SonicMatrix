
#include <Arduino.h>
#include "Timer3.h"

void setTimer3 (uint32_t interval) {

	/*
				
		Timer 3 16bits
		
		Prescaler x resolution x max value (values in microseconds)
			CLOCK	PRESCALER	RESOLUTION	MAX TIME
			16MHz	1        	0,0625us   	4096us
			16MHz	8        	0,5us      	32768us
			16MHz	64       	4us       	262144us
			16MHz	1024     	64us      	4194304us

		With prescaler = 1 and target = 2.5us we need do count 40 times
	*/

	noInterrupts(); // disable all interrupts
	
	TCCR3A = 0;
	TCCR3B = 0;
	TCNT3  = 0;

	OCR3A = (uint16_t) (interval/64) - 1; // compare match register 16MHz clock, 1 prescaler, 400kHz frequency (the timer stats at zero hence the -1)
 
	// CTC mode
	TCCR3B |= (1 << WGM12); 
	
	// 1024 prescaler
	TCCR3B |= (1 << CS10);  
	TCCR3B |= (1 << CS12);

	interrupts(); // enable all interrupts

}

void enableTimer3 () {
	
	noInterrupts(); // disable all interrupts
	
	TIMSK3 |= (1 << OCIE1A); // enable timer compare interrupt
	
	interrupts(); // enable all interrupts
	
}

void disableTimer3 () {
	
	//noInterrupts(); // disable all interrupts
	
	TIMSK3 = 0; // disable timer interrupts
	
	//interrupts(); // enable all interrupts
	
}


#include <Arduino.h>
#include "Timer4.h"

void setTimer4 () {

	/*
		400kHz, 2.5us
		
		Timer 4
		
		Prescaler x resolution x max value (values in microseconds)
			CLOCK	PRESCALER	RESOLUTION	MAX TIME
			16MHz	1        	0,0625us   	4096us
			16MHz	8        	0,5us      	32768us
			16MHz	64       	4us       	262144us
			16MHz	1024     	64us      	4194304us

		With prescaler = 1 and target = 2.5us we need do count 40 times
	*/

	noInterrupts(); // disable all interrupts
	
	TCCR4A = 0;
	TCCR4B = 0;
	TCNT4  = 0;

	OCR4A = 40 - 1; // compare match register 16MHz clock, 1 prescaler, 400kHz frequency (the timer stats at zero hence the -1)
	
	TCCR4B |= (1 << WGM12); // CTC mode
	TCCR4B |= (1 << CS10); // 1 prescaler 
	//TIMSK4 |= (1 << OCIE1A); // enable timer compare interrupt

	interrupts(); // enable all interrupts

}

void enableTimer4 () {
	
	noInterrupts(); // disable all interrupts
	
	TIMSK4 |= (1 << OCIE1A); // enable timer compare interrupt
	
	interrupts(); // enable all interrupts
	
}

void disableTimer4 () {
	
	noInterrupts(); // disable all interrupts
	
	TIMSK4 = 0; // disable timer interrupts
	
	interrupts(); // enable all interrupts
	
}

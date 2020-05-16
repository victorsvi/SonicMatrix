/*
 * An abstraction layer for operation of the timer hardware from Arduino.
 * 
 * by Victor Salvi (victorsvi@gmail.com), 2020.
 */
 
#include <Arduino.h>
#include "Timer3.h"

/**
 * Sets up the Timer 3 of Arduino Mega to raise a Compare Match Register interrupt on channel A with a constant period of "interval".
 * The timer will be disabled in this function.
 * Create a funcion to be called at the interrupt with the name "ISR( TIMER3_COMPA_vect )".
 * The minimum interval is 64us and the maximum is 4194304us. The interval resolution is 64us.
 * @param interval Interval of time between interrupts in microseconds.
 */
void setTimer3 (uint32_t interval) {

	/*
				
		Timer 3 16bits
		
		Prescaler x resolution x max value (values in microseconds)
			CLOCK	PRESCALER	RESOLUTION	MAX TIME
			16MHz	1        	0,0625us   	4096us
			16MHz	8        	0,5us      	32768us
			16MHz	64       	4us       	262144us
			16MHz	1024     	64us      	4194304us

	*/

	noInterrupts(); // disable all interrupts
	
	TCCR3A = 0;
	TCCR3B = 0;
	TCNT3  = 0;

	// Compare and match register
	OCR3A = (uint16_t) (interval/64) - 1; // as the counter increments every 64us, the number of counts to reach a time interval in us is that time divided by 64us (the timer stats at zero hence the -1)
 
	// CTC mode
	TCCR3B |= (1 << WGM12); 
	
	// 1024 prescaler (15625Hz)
	TCCR3B |= (1 << CS10);  
	TCCR3B |= (1 << CS12);
	
	TIMSK3 = 0; // disable timer interrupts

	interrupts(); // enable all interrupts

}

/**
 * Enables the Timer 3 Compare and Match interrupt on channel A.
 * Be sure to set up the Timer first.
 */
void enableTimer3 () {
	
	noInterrupts(); // disable all interrupts
	
	TIMSK3 |= (1 << OCIE1A); // enable timer compare interrupt
	
	interrupts(); // enable all interrupts
	
}

/**
 * Disables the Timer 3 Compare and Match interrupt on channel A.
 */
void disableTimer3 () {
	
	//noInterrupts(); // disable all interrupts
	
	TIMSK3 = 0; // disable timer interrupts
	
	//interrupts(); // enable all interrupts
	
}

/*
 * Generates a signal to test the driver circuit and the transducer itself.
 * A define statement allows fine tunning the frequency
 * A define statement allows setting the wave format
 *
 * by Victor Salvi (victorsvi@gmail.com), 2020.
 */
 
#include <Arduino.h>
#include <stdint.h>

//#define UNO
#define MEGA

#define ARRAY_PHASERES 10 //number of "slices" or "steps" that a period is divided into. It defines the resolution of the phase control (max 16 bits)
#define PERIOD 2500 //period of the signal in picoseconds (max 4096000ps)
#define PATTERN 0x001F //binary unsigned representing the pattern to be outputted (0xAAAA = #_#_#_#_#_#_#_#_)
#define PIN 13 //pin for the output signal

#define MSK(b) (1 << b) //creates a mask with the bth bit high

struct s_pin { // represents a pin mask
	volatile uint8_t *bank_ptr; // pin's bank address
	uint8_t bit_msk; // pin's bit mask for the bank
};

void setTimer1 ();
void enableTimer1 ();
void disableTimer1 ();
ISR( TIMER1_COMPA_vect );

volatile uint8_t array_phase_idx = 0; // current phase index of the signal (which bit of the pattern is being outputted)
volatile uint8_t *bank_ptr; // pin's bank address
uint8_t bit_msk; // pin's bit mask for the bank
volatile uint8_t out = 0;

#ifdef MEGA
const struct s_pin PINS[70] = { // Port bank and bit mask for each Arduino Mega pin indexed by pin number
	{&PORTE, MSK(0)},
	{&PORTE, MSK(1)},
	{&PORTE, MSK(4)},
	{&PORTE, MSK(5)},
	{&PORTG, MSK(5)},
	{&PORTE, MSK(3)},
	{&PORTH, MSK(3)},
	{&PORTH, MSK(4)},
	{&PORTH, MSK(5)},
	{&PORTH, MSK(6)},
	{&PORTB, MSK(4)},
	{&PORTB, MSK(5)},
	{&PORTB, MSK(6)},
	{&PORTB, MSK(7)},
	{&PORTJ, MSK(1)},
	{&PORTJ, MSK(0)},
	{&PORTH, MSK(1)},
	{&PORTH, MSK(0)},
	{&PORTD, MSK(3)},
	{&PORTD, MSK(2)},
	{&PORTD, MSK(1)},
	{&PORTD, MSK(0)},
	{&PORTA, MSK(0)},
	{&PORTA, MSK(1)},
	{&PORTA, MSK(2)},
	{&PORTA, MSK(3)},
	{&PORTA, MSK(4)},
	{&PORTA, MSK(5)},
	{&PORTA, MSK(6)},
	{&PORTA, MSK(7)},
	{&PORTC, MSK(7)},
	{&PORTC, MSK(6)},
	{&PORTC, MSK(5)},
	{&PORTC, MSK(4)},
	{&PORTC, MSK(3)},
	{&PORTC, MSK(2)},
	{&PORTC, MSK(1)},
	{&PORTC, MSK(0)},
	{&PORTD, MSK(7)},
	{&PORTG, MSK(2)},
	{&PORTG, MSK(1)},
	{&PORTG, MSK(0)},
	{&PORTL, MSK(7)},
	{&PORTL, MSK(6)},
	{&PORTL, MSK(5)},
	{&PORTL, MSK(4)},
	{&PORTL, MSK(3)},
	{&PORTL, MSK(2)},
	{&PORTL, MSK(1)},
	{&PORTL, MSK(0)},
	{&PORTB, MSK(3)},
	{&PORTB, MSK(2)},
	{&PORTB, MSK(1)},
	{&PORTB, MSK(0)},
	{&PORTF, MSK(0)},
	{&PORTF, MSK(1)},
	{&PORTF, MSK(2)},
	{&PORTF, MSK(3)},
	{&PORTF, MSK(4)},
	{&PORTF, MSK(5)},
	{&PORTF, MSK(6)},
	{&PORTF, MSK(7)},
	{&PORTK, MSK(0)},
	{&PORTK, MSK(1)},
	{&PORTK, MSK(2)},
	{&PORTK, MSK(3)},
	{&PORTK, MSK(4)},
	{&PORTK, MSK(5)},
	{&PORTK, MSK(6)},
	{&PORTK, MSK(7)}
};
#endif

#ifdef UNO
const struct s_pin PINS[20] = { // Port bank and bit mask for each Arduino Mega pin indexed by pin number
	{&PORTD, MSK(0)},
	{&PORTD, MSK(1)},
	{&PORTD, MSK(2)},
	{&PORTD, MSK(3)},
	{&PORTD, MSK(4)},
	{&PORTD, MSK(5)},
	{&PORTD, MSK(6)},
	{&PORTD, MSK(7)},
	{&PORTB, MSK(0)},
	{&PORTB, MSK(1)},
	{&PORTB, MSK(2)},
	{&PORTB, MSK(3)},
	{&PORTB, MSK(4)},
	{&PORTB, MSK(5)},
	{&PORTC, MSK(0)},
	{&PORTC, MSK(1)},
	{&PORTC, MSK(2)},
	{&PORTC, MSK(3)},
	{&PORTC, MSK(4)},
	{&PORTC, MSK(5)}
};
#endif

void setup () {
	
	pinMode(PIN, OUTPUT);
	
	bank_ptr = PINS[PIN].bank_ptr;
	bit_msk = PINS[PIN].bit_msk;
	
	array_phase_idx = 0;


  *bank_ptr |= bit_msk;

  noInterrupts(); // disable all interrupts
  
  TCCR0A = 0;
  TCCR0B = 0;
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR3A = 0;
  TCCR3B = 0;
  TCCR3C = 0;
  TCCR4A = 0;
  TCCR4B = 0;
  TCCR4C = 0;
  TCCR5A = 0;
  TCCR5B = 0;
  TCCR5C = 0;
 TIMSK0 = 0; // disable timer compare interrupt
 TIMSK1 = 0; // disable timer compare interrupt
 TIMSK2 = 0; // disable timer compare interrupt
 TIMSK3 = 0; // disable timer compare interrupt
 TIMSK4 = 0; // disable timer compare interrupt
 TIMSK5 = 0; // disable timer compare interrupt
interrupts();
  
	setTimer1();
	enableTimer1();
	
} //setup

void loop () {
	
/*
  if(PATTERN & (1 << (array_phase_idx))){ //gets the current bit of the pattern
    *bank_ptr |= bit_msk; //sets pin to 1
  }
  else {
    *bank_ptr &= ~bit_msk; //sets pin to 0
  }
  
  if( array_phase_idx < ARRAY_PHASERES ) { //increments the current phase index. It will cycle between 0 and (ARRAY_PHASERES - 1)
    array_phase_idx++;
  }
  else {
    array_phase_idx = 0; 
  }
*/
/*
  if(out){ //gets the current bit of the pattern
    *bank_ptr |= bit_msk; //sets pin to 1
  }
  else {
    *bank_ptr &= ~bit_msk; //sets pin to 0
  }
  out = ~out;
*/


} //loop


ISR( TIMER1_COMPA_vect ) {

	if(PATTERN & (1 << (array_phase_idx))){ //gets the current bit of the pattern
		*bank_ptr |= bit_msk; //sets pin to 1
	}
	else {
		*bank_ptr &= ~bit_msk; //sets pin to 0
	}
	
	if( array_phase_idx < ARRAY_PHASERES ) { //increments the current phase index. It will cycle between 0 and (ARRAY_PHASERES - 1)
		array_phase_idx++;
	}
	else {
		array_phase_idx = 0; 
	}

/*
  if(out){ //gets the current bit of the pattern
    *bank_ptr |= bit_msk; //sets pin to 1
  }
  else {
    *bank_ptr &= ~bit_msk; //sets pin to 0
  }
  out = ~out;

  *bank_ptr = ~(*bank_ptr);
*/
} //ISR T1

/**
 * Sets up the Timer 1 of Arduino Mega to run at 400kHz and raise a Compare Match Register interrupt on channel A.
 * The timer will be disabled in this function.
 * Create a function to be called at the interrupt with the name "ISR( TIMER1_COMPA_vect )".
 */
void setTimer1 () {

	/*
		400kHz, 2.5us
		
		Timer 1 16bits
		
		Prescaler x resolution x max value (values in microseconds)
			CLOCK	PRESCALER	RESOLUTION	MAX TIME
			16MHz	1        	0,0625us   	4096us
			16MHz	8        	0,5us      	32768us
			16MHz	64       	4us       	262144us
			16MHz	1024     	64us      	4194304us

		With prescaler = 1 and target = 2.5us we need to count 40 times
	*/

	noInterrupts(); // disable all interrupts
	
	TCCR1A = 0;
	TCCR1B = 0;
  TCCR1C = 0;
	TCNT1  = 0;

	//compare match register
	OCR1A = ((PERIOD*10)/625) - 1; // value for 16MHz clock, 1 prescaler (the timer stats at zero hence the -1)
	
  TCNT1  = 0;
  
	TCCR1B |= (1 << WGM12); // CTC mode
	TCCR1B |= (1 << CS10); // 1 prescaler 
	
	TIMSK1 = 0; // disable timer compare interrupt

	interrupts(); // enable all interrupts

}

/**
 * Enables the Timer 1 Compare and Match interrupt on channel A.
 * Be sure to set up the Timer first.
 */
void enableTimer1 () {
	
	noInterrupts(); // disable all interrupts
	
	TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
	
	interrupts(); // enable all interrupts
	
}

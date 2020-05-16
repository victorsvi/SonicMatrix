
#include "Timer4.h"
#include "Timer5.h"
#include <stdint.h>

ISR( TIMER3_COMPA_vect );
ISR( TIMER4_COMPA_vect );

volatile uint8_t traj_step_idx = 0, traj_step_num = 64, led = LOW, t4 = 0;
volatile uint8_t array_phase_idx = 0;
#define ARRAY_PHASERES 10

void setup () {
	
	Serial.begin(115200);
	
	pinMode(LED_BUILTIN,OUTPUT);
	
  setTimer5 (512000);
  enableTimer5 ();
  setTimer4();
  enableTimer4 ();
	
	

} //setup

void loop () {
	
	//Serial.println(traj_step_idx);
	delay(256);
	
} //loop

ISR( TIMER3_COMPA_vect ) {

	if(traj_step_idx < traj_step_num ) { //increments the step until reaches the last step
		traj_step_idx++;
	}
	else { //then stay in the last step and disables the timer
		disableTimer5 ();
	}
	
	led = ~led;
	digitalWrite(LED_BUILTIN,led);

} //ISR T5

ISR( TIMER4_COMPA_vect ) {

  //buffer the indexes to optimize access as they're volatile
  uint8_t phase_idx = array_phase_idx;
  uint8_t step_idx = traj_step_idx;
  
  // Just copy the port state from the buffer
  //PORTA = traj_port_buffer[step_idx][phase_idx][0];
  //PORTB = traj_port_buffer[step_idx][phase_idx][1];
  //PORTC = traj_port_buffer[step_idx][phase_idx][2];
  //PORTD = traj_port_buffer[step_idx][phase_idx][3];
  //PORTE = 0;
  //PORTF = traj_port_buffer[step_idx][phase_idx][4];
  //PORTG = traj_port_buffer[step_idx][phase_idx][5];
  //PORTH = traj_port_buffer[step_idx][phase_idx][6];
  //PORTI = 0;
  //PORTJ = traj_port_buffer[step_idx][phase_idx][7];
  //PORTK = traj_port_buffer[step_idx][phase_idx][8];
  //PORTL = traj_port_buffer[step_idx][phase_idx][9];

  //Serial.println(array_phase_idx);
  array_phase_idx < 10 ? array_phase_idx++ : array_phase_idx = 0;
} //ISR T4

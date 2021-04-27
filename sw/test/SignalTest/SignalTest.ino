/* TEST PROGRAM TO CHECK THE STABILITY OF THE GENERATED SIGNAL */


//#include <stdlib.h>
#include <stdint.h>
//#include <EEPROM.h>
#include <Arduino.h>
#include "Ultrasonic.h"
#include "Debug.h"
#include "Timer4.h"

#define TRANS_DIAMETER 16 //diameter of the element in millimeters
#define TRANS_SEPARATION 2 //distance between two consecutive elements in the array in millimeters
#define ARRAY_SIZE_X 8 //number of transducers of the array in the x dimension
#define ARRAY_SIZE_Y 8 //number of transducers of the array in the y dimension
#define ARRAY_PHASERES 10 //number of transducers of the array in the y dimension
#define TRAJ_RES 1 //trajectory resolution in millimeters
#define TRAJ_MAXSTEPS 100 //maximum steps of the trajectory

/* PROTOTYPES */

void trand_array_load ( t_transd_array *transd_array );
ISR( TIMER4_COMPA_vect );

/* DATA DEFINITION */
struct s_state {
	uint8_t isActiveMove = 0, isActiveStatic = 0, isStop = 0, isFlat = 0;
	uint8_t x = 0, y = 0, z = 0, d = 0, s = 0;
	uint8_t currStep = 0;
	uint8_t moveCurrStep = 0, moveNSteps = 0;
	uint8_t lastx = 0, lasty = 0, lastz = 0;
};

struct s_pin {
	volatile uint8_t *bank_ptr;
	uint8_t bit_idx;
};

/* GLOBAL VARIABLES */

const uint8_t ARRAY_CALIBRATION[ARRAY_SIZE_X][ARRAY_SIZE_Y][2] = {
	//pin numer, phase compensation
	4 , 0,//x0, y0
	6 , 0,//x0, y1
	7 , 0,
	8 , 0,
	9 , 0,
	10, 0,
	11, 0,
	12, 0,
	13, 0,
	14, 0,
	15, 0,
	16, 0,
	17, 0,
	18, 0,
	19, 0,
	20, 0,
	21, 0,
	22, 0,
	23, 0,
	24, 0,
	25, 0,
	26, 0,
	27, 0,
	28, 0,
	29, 0,
	30, 0,
	31, 0,
	32, 0,
	33, 0,
	34, 0,
	35, 0,
	36, 0,
	37, 0,
	38, 0,
	39, 0,
	40, 0,
	41, 0,
	42, 0,
	43, 0,
	44, 0,
	45, 0,
	46, 0,
	47, 0,
	48, 0,
	49, 0,
	50, 0,
	51, 0,
	52, 0,
	53, 0,
	55, 0,
	56, 0,
	57, 0,
	58, 0,
	59, 0,
	60, 0,
	61, 0,
	62, 0,
	63, 0,
	64, 0,
	65, 0,
	66, 0,
	67, 0,
	68, 0,
	69, 0
};

const struct s_pin PINS[70] = {
	{&PORTE, 0},
	{&PORTE, 1},
	{&PORTE, 4},
	{&PORTE, 5},
	{&PORTG, 5},
	{&PORTE, 3},
	{&PORTH, 3},
	{&PORTH, 4},
	{&PORTH, 5},
	{&PORTH, 6},
	{&PORTB, 4},
	{&PORTB, 5},
	{&PORTB, 6},
	{&PORTB, 7},
	{&PORTJ, 1},
	{&PORTJ, 0},
	{&PORTH, 1},
	{&PORTH, 0},
	{&PORTD, 3},
	{&PORTD, 2},
	{&PORTD, 1},
	{&PORTD, 0},
	{&PORTA, 0},
	{&PORTA, 1},
	{&PORTA, 2},
	{&PORTA, 3},
	{&PORTA, 4},
	{&PORTA, 5},
	{&PORTA, 6},
	{&PORTA, 7},
	{&PORTC, 7},
	{&PORTC, 6},
	{&PORTC, 5},
	{&PORTC, 4},
	{&PORTC, 3},
	{&PORTC, 2},
	{&PORTC, 1},
	{&PORTC, 0},
	{&PORTD, 7},
	{&PORTG, 2},
	{&PORTG, 1},
	{&PORTG, 0},
	{&PORTL, 7},
	{&PORTL, 6},
	{&PORTL, 5},
	{&PORTL, 4},
	{&PORTL, 3},
	{&PORTL, 2},
	{&PORTL, 1},
	{&PORTL, 0},
	{&PORTB, 3},
	{&PORTB, 2},
	{&PORTB, 1},
	{&PORTB, 0},
	{&PORTF, 0},
	{&PORTF, 1},
	{&PORTF, 2},
	{&PORTF, 3},
	{&PORTF, 4},
	{&PORTF, 5},
	{&PORTF, 6},
	{&PORTF, 7},
	{&PORTK, 0},
	{&PORTK, 1},
	{&PORTK, 2},
	{&PORTK, 3},
	{&PORTK, 4},
	{&PORTK, 5},
	{&PORTK, 6},
	{&PORTK, 7}
};

struct s_state state;

t_transd_array *transd_array = NULL;

//uint8_t traj_steps[TRAJ_MAXSTEPS][ARRAY_SIZE_X][ARRAY_SIZE_Y];
//uint8_t traj_steps[TRAJ_MAXSTEPS][ARRAY_PHASERES][10];

uint32_t cnt, start;

void setup () {
	
	Serial.begin(115200);
	
	DDRA = 0xFF;
	DDRB = 0xFF;
	DDRC = 0xFF;
	DDRD = 0xFF;
	//DDRE = 0xFF;
	DDRF = 0xFF;
	DDRG = 0xFF;
	DDRH = 0xFF;
	//DDRI = 0xFF;
	DDRJ = 0xFF;
	DDRK = 0xFF;
	DDRL = 0xFF;
	
	setTimer4();
	
	transd_array = transd_array_init( ARRAY_SIZE_X, ARRAY_SIZE_Y, TRANS_DIAMETER, TRANS_SEPARATION, ARRAY_PHASERES );
	if( transd_array == NULL ) {
		#ifdef DEBUG
		DEBUG_MSG("The transducer array pointer is null")
		#endif
	}
	trand_array_load (transd_array);
	
	transd_array_calcfocus( transd_array, 128, 71, 71, 50 );
	
	cnt = 0;
	start = micros();
	
	enableTimer4 ();

  Serial.println("ready");
}

void loop () {
	
  uint8_t x, y, bit, pin;
    
  PORTA = 0;
  PORTB = 0;
  PORTC = 0;
  PORTD = 0;
  //PORTE = 0;
  PORTF = 0;
  PORTG = 0;
  PORTH = 0;
  //PORTI = 0;
  PORTJ = 0;
  PORTK = 0;
  PORTL = 0;
  
  //if(state.isActiveStatic && !state.isActiveMove) {
      
    
    for(x = 0; x < ARRAY_SIZE_X; x++){
      for(y = 0; y < ARRAY_SIZE_Y; y++){
        
        //access the pattern and gets the value for the bit representing the current step
        bit = (transd_array->transd_ptr + x * ARRAY_SIZE_Y + y)->pattern & (1 << state.currStep);
        //gets the pin that the transducer is connected to
        pin = (transd_array->transd_ptr + x * ARRAY_SIZE_Y + y)->port_pin;
        
        //updates only the current pin
        //the bank and bit of each pin is available on the PINS constant indexed by pin number
        *(PINS[pin].bank_ptr) |= bit << *(PINS[pin].bank_ptr);
              
      }
    }
    
    state.currStep < ARRAY_PHASERES ? state.currStep = 0 : state.currStep++;
  //}
  
  //DEBUG
  cnt++;
  if(cnt > 40) {
    cnt = 0;
    Serial.println(micros() - start);
    start = micros();
  }
  //DEBUG
  
}

ISR( TIMER4_COMPA_vect ) {
	
  uint8_t x, y, bit, pin;
    
	PORTA = 0;
	PORTB = 0;
	PORTC = 0;
	PORTD = 0;
	//PORTE = 0;
	PORTF = 0;
	PORTG = 0;
	PORTH = 0;
	//PORTI = 0;
	PORTJ = 0;
	PORTK = 0;
	PORTL = 0;
	
	//if(state.isActiveStatic && !state.isActiveMove) {
			
		
		for(x = 0; x < ARRAY_SIZE_X; x++){
			for(y = 0; y < ARRAY_SIZE_Y; y++){
				
				//access the pattern and gets the value for the bit representing the current step
				bit = (transd_array->transd_ptr + x * ARRAY_SIZE_Y + y)->pattern & (1 << state.currStep);
				//gets the pin that the transducer is connected to
				pin = (transd_array->transd_ptr + x * ARRAY_SIZE_Y + y)->port_pin;
				
				//updates only the current pin
				//the bank and bit of each pin is available on the PINS constant indexed by pin number
				*(PINS[pin].bank_ptr) |= bit << *(PINS[pin].bank_ptr);
							
			}
		}
		
		state.currStep < ARRAY_PHASERES ? state.currStep = 0 : state.currStep++;
	//}
	
	//DEBUG
	cnt++;
	if(cnt > 400000) {
		cnt = 0;
		Serial.println(micros() - start);
		start = micros();
	}
	//DEBUG
	
}

void trand_array_load ( t_transd_array *transd_array ){
	
	uint8_t x, y;
	
	for(x = 0; x < ARRAY_SIZE_X; x++){
		for(y = 0; y < ARRAY_SIZE_Y; y++){

			transd_array_set( transd_array, x, y, ARRAY_CALIBRATION[x][y][0], ARRAY_CALIBRATION[x][y][1] );
		}
	}
}









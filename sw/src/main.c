
#include <stdlib.h>
#include <stdint.h>
#include <EEPROM.h>
#include <Arduino.h>
#include "Ultrasonic.h"

#define TRANS_DIAMETER 16 //diameter of the element in milimeters
#define TRANS_SEPARATION 2 //distance between two consecutive elements in the array in milimeters
#define ARRAY_SIZE_X 8 //number of transducers of the array in the x dimension
#define ARRAY_SIZE_Y 8 //number of transducers of the array in the y dimension

struct s_memory {
	uint8_t id;
	uint8_t pin;
	uint8_t phase;
}

union u_memory {
	struct s_memory memory;
	uint8_t *buf;
}

t_array *array;

//pinToBank() //converte pino para porta
//criar um timer para gerar o sinal de 40khz. com resolução de 1/8, o timer deve ter frequência de 320kHz (3.125ns)


void setup () {
	
	array = Array_init( ARRAY_SIZE_X, ARRAY_SIZE_Y, TRANS_DIAMETER, TRANS_SEPARATION );
	if( array == NULL ) {
		Serial.println("E001 - Error creating the array");
	}
	
}

void loop () {
	
	
}
/*
void Array_load ( t_array *array ) {
	
	if( array != NULL ) {
		
		
	}
}
*/











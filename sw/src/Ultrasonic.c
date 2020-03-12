/*
	TODO:
		rotina de calibração: assigment dos pinos de io para cada posição da matriz (salvar na eeprom)
		rotina de calibração: compensação de fase (salvar na eeprom)
		
	
	SCOPE:
		array quadrado 8x8
		recebe uma coordenada e cria um ponto focal nessa coordenada (tridimensional)
		move o ponto focal com uma velocidade parametrizavel
	
	UPGRADES:
		use potentiometer to fine tune the frequency
		use analog joystick to move the focal point

*/

#include <stdlib.h>
#include <stdint.h>
//#include <EEPROM.h>
//#include <Arduino.h>


#define SOUND_SPEED 343600 //speed of the sound in the medium in milimeters per second (at 20°C dry air)
#define SOUND_FREQ 40000 //frequency of the signal
//#define RESOLUTION 8 //in how many discrete steps the period of the signal is divided. It defines the phase delay resolution and the amplitude resolution

uint8_t transd_array_init( t_transd_array *transd_array, const uint8_t size_x, const uint8_t size_y, const uint8_t elem_diameter, const uint8_t elem_separation, const uint8_t phase_res ){
	
	uint8_t ret;
	uint8_t x,y;
	
	if(transd_array == NULL) {
		#IFDEF DEBUG DEBUG_MSG("The transducer array pointer is null") #ENDIF
		return 40;	
	}
	
	if(size_x == 0 || size_y == 0) {
		#IFDEF DEBUG DEBUG_MSG("Invalid size for the transducer array") #ENDIF
		return 41;
	}

	if(elem_diameter == 0) {
		#IFDEF DEBUG DEBUG_MSG("Invalid value for transducer diameter") #ENDIF
		return 42;
	}
	
	if(phase_res == 0) {
		#IFDEF DEBUG DEBUG_MSG("Invalid value for phase resolution") #ENDIF
		return 43;
	}
	
	transd_array->size_x = size_x;
	transd_array->size_y = size_y;
	transd_array->elem_diameter = elem_diameter;
	transd_array->elem_separation = elem_separation;
	transd_array->phase_res = phase_res;
	
	/* transd_array->curr_step = 0; */
	
	transd_array->transd_ptr = malloc(sizeof(t_transd) * size_x * size_y);
	if(transd_array->transd_ptr == NULL){
		#IFDEF DEBUG DEBUG_MSG("Memory allocation error") #ENDIF
		return 1;
	}
	
	for(x = 0; x < size_x; x++){
		for(y = 0; y < size_y; y++){
			
			ret = transd_init( (transd_array->transd_ptr + x * size_y + y) /* &(transd_array->transd_ptr[x][y]) */, x, y, 0, 0, elem_diameter, elem_separation );
			if(ret != 0) {
				return ret;
			}
		}
	}
	
	return 0;
} //transd_array_init

uint8_t transd_array_set( t_transd_array *transd_array, const uint8_t index_x, const uint8_t index_y, const uint8_t port_pin, const uint8_t phase_comp ){
	
	uint8_t ret;
	
	if(transd_array == NULL) {
		#IFDEF DEBUG DEBUG_MSG("The transducer array pointer is null") #ENDIF
		return 40;	
	}
	
	ret = transd_set( (transd_array->transd_ptr + x * size_y + y), port_pin, phase_comp );
	
	return ret;
} //transd_array_set

uint8_t transd_array_calcfocus( t_transd_array *transd_array, const uint8_t focus_x, const uint8_t focus_y, const uint8_t focus_z ) {
	
	uint8_t ret;
	uint8_t x,y;
	
	if(transd_array == NULL) {
		#IFDEF DEBUG DEBUG_MSG("The transducer array pointer is null") #ENDIF
		return 40;	
	}
	
	for(x = 0; x < size_x; x++){
		for(y = 0; y < size_y; y++){
			
			ret = transd_calcfocus( (transd_array->transd_ptr + x * size_y + y), /*transd_array->phase_res, transd_array->curr_step,*/ focus_x, focus_y, focus_z );
			if(ret != 0) {
				return ret;
			}
		}
	}
	
	return 0;
} //transd_array_calcfocus

uint8_t transd_array_calcflat( t_transd_array *transd_array, const uint8_t amplitude ) {
	
	uint8_t ret;
	uint8_t x,y;
	
	if(transd_array == NULL) {
		#IFDEF DEBUG DEBUG_MSG("The transducer array pointer is null") #ENDIF
		return 40;	
	}
	
	for(x = 0; x < size_x; x++){
		for(y = 0; y < size_y; y++){
			
			ret = transd_calcflat( (transd_array->transd_ptr + x * size_y + y), /*transd_array->phase_res, transd_array->curr_step,*/ amplitude );
			if(ret != 0) {
				return ret;
			}
		}
	}
	
	return 0;
} //transd_array_calcflat




/* PRIVATE */

uint8_t transd_getPattern( const uint8_t phase_res, const uint8_t phase_comp, const uint8_t phase, const uint8_t amplitude ){
	/*
	RESOLUTION: Define em quantos steps um período será dividido (quantos bits terá o padrão e cara transdutor). Para uma coordenada, armazenar o padrão em um inteiro para cada transd
	para gerar o sinal, basta ler os bits sequencialmente
	fazer a resolução variável, de no máximo 16 (16bits)
	
	duty -> define a quantidade de 1s e 0s (bits on = duty/resolução; 3 = 30/10) (se o duty for de 0 a 255, fazer bits on = (((duty*100)/ 255)/resolução))
	30% -> _______###
	50% -> _____#####

	phase -> define quanto deslocar (bits deslocados = fase / (360/resolução); 2 = 72 / 360 / 10) (se a fase for de 0 a 255, fazer bits deslocados = (fase/(255)/resolução))
	0   -> _____##### (0°)
	2   -> ___#####__ (72°)
	8   -> ##_____### (288°) (5 amp, 8 ph)
	*/
	
	uint8_t bits_duty, bits_phase;
	uint16_t pattern = 0;
	
	//sets the (Nth + 1) bit = 1 (00001000b for n = 3) then subtracts one to make all the lesser bits = 1 (00000111b for n = 3) 
	pattern = (1 << bits_duty) - 1; 
	
	
	
	return pattern;
}

uint8_t transd_init( t_transd *transd, const uint8_t index_x, const uint8_t index_y, const uint8_t port_pin, const uint8_t phase_comp, const uint8_t elem_diameter, const uint8_t elem_separation ) {
	
	if(transd == NULL) {
		#IFDEF DEBUG DEBUG_MSG("The transducer pointer is null") #ENDIF
		return 44;	
	}
	
	if(elem_diameter == 0) {
		#IFDEF DEBUG DEBUG_MSG("Invalid value for transducer diameter") #ENDIF
		return 42;
	}
			
	transd->port_pin = port_pin;
	transd->phase_comp = phase_comp;
	transd->x = ((elem_diameter / 2) + (elem_diameter * index_x) + (elem_separation * index_x)); //calculates the position of the transducer on the array
	transd->y = ((elem_diameter / 2) + (elem_diameter * index_y) + (elem_separation * index_y)); //calculates the position of the transducer on the array
	transd->phase = 0;
	transd->amplitude = 0;
	/* here is an example of the calculation of the position in one axys (elem_diameter = 16, elem_separation = 2)
	 
	  diameter           separation
	 |-------|          |--|
	   _____      _____      _____  
	  /     \    /     \    /     \ 
	 |   -   |  |   -   |  |   -   |
	  \_____/    \_____/    \_____/ 

	 |   |   |  |   |   |  |   |   |
	 0   8  16  18  26 34  36  44 52
	 -------------------------------> 
	 */
} //transd_init

uint8_t transd_set( t_transd *transd, uint8_t port_pin, uint8_t phase_comp );

uint8_t transd_calcfocus( t_transd *transd, /*const uint8_t phase_res, const uint8_t curr_step,*/ const uint8_t focus_x, const uint8_t focus_y, const uint8_t focus_z );

uint8_t transd_calcflat( t_transd *transd, /*const uint8_t phase_res, const uint8_t curr_step,*/ const uint8_t amplitude ) {
	
	if(transd == NULL) {
		#IFDEF DEBUG DEBUG_MSG("The transducer pointer is null") #ENDIF
		return 44;	
	}
	
	transd->phase = 0;
	transd->amplitude = amplitude;
	
	
	
}

















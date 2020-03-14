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
#define SOUND_WAVELEN SOUND_SPEED/SOUND_FREQ //wave length (8.6 mm)

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

uint8_t transd_array_calcfocus( t_transd_array *transd_array, const uint8_t duty_cycle, const uint8_t focus_x, const uint8_t focus_y, const uint8_t focus_z ) {
	
	uint8_t ret;
	uint8_t x,y;
	
	if(transd_array == NULL) {
		#IFDEF DEBUG DEBUG_MSG("The transducer array pointer is null") #ENDIF
		return 40;	
	}
	
	for(x = 0; x < size_x; x++){
		for(y = 0; y < size_y; y++){
			
			ret = transd_calcfocus( (transd_array->transd_ptr + x * size_y + y), transd_array->phase_res, /*transd_array->curr_step,*/ focus_x, focus_y, focus_z );
			if(ret != 0) {
				return ret;
			}
		}
	}
	
	return 0;
} //transd_array_calcfocus

uint8_t transd_array_calcflat( t_transd_array *transd_array, const uint8_t duty_cycle ) {
	
	uint8_t ret;
	uint8_t x,y;
	
	if(transd_array == NULL) {
		#IFDEF DEBUG DEBUG_MSG("The transducer array pointer is null") #ENDIF
		return 40;	
	}
	
	for(x = 0; x < size_x; x++){
		for(y = 0; y < size_y; y++){
			
			ret = transd_calcflat( (transd_array->transd_ptr + x * size_y + y), transd_array->phase_res, /*transd_array->curr_step,*/ duty_cycle );
			if(ret != 0) {
				return ret;
			}
		}
	}
	
	return 0;
} //transd_array_calcflat

#IFDEF DEBUG
/* Dumps the array structure to JSON */
void transd_dumpToFile ( FILE *f,  t_transd_array *transd_array ){
	
	char buff[1024];
	char patt[16];
	t_transd *transd_ptr;
	uint8_t i,x,y;
	
	if(f == NULL) return;
	
	sprintf(buff, "{\r\n\t\"size_x\": %u,\r\n\t\"size_y\": %u,\r\n\t\"elem_diameter\": %u,\r\n\t\"elem_separation\": %u,\r\n\t\"phase_res\": %u,\r\n\t\"transd_ptr\": [\r\n", transd_array->size_x, transd_array->size_y, transd_array->elem_diameter, transd_array->elem_separation, transd_array->phase_res);
	fputs(buff, f);
	
	for(x = 0; x < size_x; x++){
		for(y = 0; y < size_y; y++){
			
			if( x == 0 && y == 0 ) {
				fputc(',',f);
			}
			
			transd_ptr = (transd_array->transd_ptr + x * size_y + y);
			
			for(i = 0; i < 16; i++) {
				
				patt[i] = transd_ptr->pattern & ((1 << (i+1)) - 1) ? '#' : '_';
			}
			
			sprintf(buff, "\t{\r\n\t\t\"port_pin\": %u,\r\n\t\t\"phase_comp\": %u,\r\n\t\t\"x\": %u,\r\n\t\t\"y\": %u,\r\n\t\t\"phase\": %u,\r\n\t\t\"duty_cycle\": %u,\r\n\t\t\"pattern\": \"%s\"\r\n\t}", transd_ptr->port_pin, transd_ptr->phase_comp, transd_ptr->x, transd_ptr->y, transd_ptr->phase, transd_ptr->duty_cycle, patt);
			fputs(buff, f);

		}
	}
	
	sprintf(buff, "\t]\r\n}\r\n", );
	fputs(buff, f);
}
#ENDIF



/* PRIVATE */

uint32_t transd_sqrt_int (const uint32_t x) {

    uint32_t l = 1;
    uint32_t h = x;
    uint32_t n = (l + h)/2;

    while ( l < n ){

        if( (n * n) <= x ){
            l = n;
        }
        else if ( (n * n) > x ) {
            h = n;
        }
        n = (l + h)/2;

    };

    return l;
}

uint16_t transd_getPattern( const uint8_t phase_res, const uint8_t phase_comp, const uint8_t phase, const uint8_t duty ){
	/*
	RESOLUTION: Define em quantos steps um período será dividido (quantos bits terá o padrão e cara transdutor). Para uma coordenada, armazenar o padrão em um inteiro para cada transd
	para gerar o sinal, basta ler os bits sequencialmente
	fazer a resolução variável, de no máximo 16 (16bits)
	*/
	
	uint8_t bits_duty, bits_phase;
	uint16_t pattern = 0;
	
	/*
		duty -> define a quantidade de 1s e 0s (bits on = duty/resolução; 3 = 30/10) (se o duty for de 0 a 255, fazer bits on = (((duty*100)/ 255)/resolução))
		30% -> _______###
		50% -> _____#####
	*/
	bits_duty = ((duty * 100) / 255) / phase_res);
	
	/*
		phase -> define quanto deslocar (bits deslocados = fase / (360/resolução); 2 = 72 / 360 / 10) (se a fase for de 0 a 255, fazer bits deslocados = (fase/(255)/resolução))
		0   -> _____##### (0°)
		2   -> ___#####__ (72°)
		8   -> ##_____### (288°) (5 amp, 8 ph)
	*/
	bits_phase = ((phase + phase_comp) / (255 / phase_res));
	
	//sets the (Nth + 1) bit = 1 (00001000b for n = 3) then subtracts one to make all the lesser bits = 1 (00000111b for n = 3) 
	pattern = (1 << bits_duty) - 1; 
	
	//shifts the duty cycle pattern to change phase
	pattern <<= bits_phase;
	
	//if shifting bit for phase "overflows" the number of bits of the pattern (defined by the resolution), the pattern lost some duty cycle information (the bits that overflow when shifting)
	if(bits_phase + bits_duty > phase_res) {
		//generates the bits that were lost on phase shitfing at the LSB side
		pattern |= (1 << (bits_duty + bits_phase - phase_res)) - 1;
	}
		
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
	transd->duty_cycle = 0;
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

uint8_t transd_set( t_transd *transd, uint8_t port_pin, uint8_t phase_comp ) {
	
	if(transd == NULL) {
		#IFDEF DEBUG DEBUG_MSG("The transducer pointer is null") #ENDIF
		return 44;	
	}
	
	transd->port_pin = port_pin;
	transd->phase_comp = phase_comp;
	
	return 0;	
} //transd_set

uint8_t transd_calcfocus( t_transd *transd, const uint8_t phase_res, /*const uint8_t curr_step,*/ const uint8_t duty_cycle, const uint8_t focus_x, const uint8_t focus_y, const uint8_t focus_z ){
	
	uint32_t distance;
	int8_t Dx,Dy; //may be smaller than zero
	
	if(transd == NULL) {
		#IFDEF DEBUG DEBUG_MSG("The transducer pointer is null") #ENDIF
		return 44;	
	}
	
	/*
		W = S / f (wave length = sound speed / frequency)

		Dij = ( (Xij - x)^2 + (Yij - y)^2 + (-z)^2 )^1/2 (distance from the ij transducer to the focus point)

		Pij = ( Dij * 256 ) / W  (phase of the ij transducer to have 0° of phase at the focus point) (testar o que acontece se Dij > W, já que Pij é uint8_t)(no pc funcionou!)
	*/
	Dx = transd->x - focus_x;
	Dy = transd->y - focus_y;
	distance = (Dx * Dx) + (Dy * Dy) + (focus_z * focus_z);
	distance = transd_sqrt_int(distance);
	distance = distance % SOUND_WAVELEN;
	transd->phase = (uint8_t) (( distance * 256) / SOUND_WAVELEN); 
	
	transd->duty_cycle = duty_cycle;
	transd->pattern = transd_getPattern( phase_res, transd->phase_comp, transd->phase, transd->duty_cycle ); 
	
	return 0;	
} //transd_calcfocus

uint8_t transd_calcflat( t_transd *transd, const uint8_t phase_res, /*const uint8_t curr_step,*/ const uint8_t duty_cycle ) {
	
	if(transd == NULL) {
		#IFDEF DEBUG DEBUG_MSG("The transducer pointer is null") #ENDIF
		return 44;	
	}
	
	transd->phase = 0;
	transd->duty_cycle = duty_cycle;
	transd->pattern = transd_getPattern( phase_res, transd->phase_comp, transd->phase, transd->duty_cycle ); 
	
	return 0;	
} //transd_calcflat


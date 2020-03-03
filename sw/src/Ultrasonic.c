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
#include <EEPROM.h>
#include <Arduino.h>


#define SOUND_SPEED 343000 //speed of the sound in the medium in milimeters per second
#define SOUND_FREQ 40000 //frequency of the signal
//#define RESOLUTION 8 //in how many discrete steps the period of the signal is divided. It defines the phase delay resolution and the amplitude resolution



/*
	Initializes the array, creating a list of transducers.
	The list is ordered by the position of the trasducer, frist by the x axys then by the y axys. An example of a array 2x2 is {(0,0);(0,1);(1,0);(1,1)}, where the ids are 1, 2, 3 and 4, respectvely.
	
*/
t_array *Array_init( uint8_t size_x, uint8_t size_y, uint8_t elem_diameter, uint8_t elem_separation ){
	
	t_array *array = NULL;
	t_transd *transd = NULL;
	t_transd *transd_last = NULL;
	uint8_t id_curr = 1;
	uint8_t count_x;
	uint8_t count_y;
	
	//vaidates the input parameters. If any of then is smaller or equal to zero, the calculations will be invalid
	if( size_x == 0 || size_y == 0 || elem_diameter == 0 ) {
		return NULL;
	}
	
	array = malloc(sizeof(t_array));
	if( array == NULL ) {
		return NULL;
	}
	
	array->size_x = size_x;
	array->size_y = size_y;
	array->elem_diameter = elem_diameter;
	array->elem_separation = elem_separation;
	
	for(count_y = 0; count_y < size_y; count_y++) {
		for(count_x = 0; count_x < size_x; count_x++) { //for each element of the array
		
			/// passar a usar um array. mais rápido. criar o array ordenado por pino!
			transd = malloc(sizeof(t_transd));
			if( transd == NULL ) {
				return NULL;
			}
			
			transd->id = id_curr;
			transd->pin = 0;
			transd->phase_comp = 0;
			transd->x = ((elem_diameter / 2) + (elem_diameter * count_x) + (elem_separation * count_x)); //calculates the position of the transducer on the array
			transd->y = ((elem_diameter / 2) + (elem_diameter * count_y) + (elem_separation * count_y));//calculates the position of the transducer on the array
			transd->phase = 0;
			transd->amplitude = 0;
			transd->next = NULL;
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
			
			if( transd_last == NULL ){ //the frist node is pointed by the root pointer
				array->array = transd;
			}
			else { //the other nodes are pointed by the last node
				transd_last->next = transd;
			}
			transd_last=transd;
			
			id_curr++;
		}
	}
	
	return array;	
} //Array_init


t_transd *Transd_get( t_array *array, uint8_t id ) {
	
} //Transd_get

void Transd_set( t_array *array, uint8_t id, uint8_t pin, int8_t phase_comp ) {
	
} //Transd_set



















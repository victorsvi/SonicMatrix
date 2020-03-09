
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>


///RESOLUTION: Define em quantos steps um período será dividido (quantos bits terá o padrão e cara transdutor). Para uma coordenada, armazenar o padrão em um inteiro para cada transd
//para gerar o sinal, basta ler os bits sequencialmente
//fazer a resolução variável, de no máximo 16 (16bits)
/*
duty -> define a quantidade de 1s e 0s (bits on = duty/resolução; 3 = 30/10) (se o duty for de 0 a 255, fazer bits on = (((duty*100)/ 255)/resolução))
30% -> ###_______
50% -> #####_____

phase -> define quanto deslocar (bits deslocados = fase / (360/resolução); 2 = 72 / 360 / 10) (se a fase for de 0 a 255, fazer bits deslocados = (fase/(255)/resolução))
0   -> #####_____
2   -> __#####___ (72°)

W = S / f (wave length = sound speed / frequency)

Dij = ( (Xij - x)^2 + (Zij - z)^2 + (-z)^2 )^1/2

Pij = ( Dij * 256 ) / W   (testar o que acontece se Dij > W, já que Pij é uint8_t)(no pc funcionou!)


*/

typedef struct s_transd {
	uint8_t port_pin; //pin on which the transducer is connected
	//uint8_t *port_bank; //pin's port bank
	//uint8_t port_bankidx; //pin's index on port bank
	int8_t phase_comp; //physical phase delay  of the transducer
	uint8_t x; //x position of the center of the transducer in milimeters
	uint8_t y; //y position of the center of the transducer in milimeters
	uint8_t phase; //current discrete phase delay of the transducer (how many steps the signal will delay. Depends on the signal resolution)
	uint8_t amplitude; //current discrete amplitude of the transducer (how many steps the signal will be on, like a duty cycle. Depends on the signal resolution)
	uint8_t pattern;
} t_transd;

typedef struct s_transd_array {
	uint8_t size_x; //number of transducers in the x axys
	uint8_t size_y; //number of transducers in the y axys
	uint8_t elem_diameter; //diameter of each transducer in milimeters
	uint8_t elem_separation; //separation between transducers in milimeters
	t_transd *transd_ptr; //pointer to the transducer list 
	uint8_t phase_res;
	uint8_t curr_step;
} t_transd_array;

/* PUBLIC */

uint8_t transd_array_init( t_transd_array *transd_array, const uint8_t size_x, const uint8_t size_y, const uint8_t elem_diameter, const uint8_t elem_separation, const uint8_t phase_res );

uint8_t transd_array_set( t_transd_array *transd_array, const uint8_t index_x, const uint8_t index_y, const uint8_t port_pin, const int8_t phase_comp );

uint8_t transd_array_calcfocus( t_transd_array *transd_array, const uint8_t focus_x, const uint8_t focus_y, const uint8_t focus_z );

uint8_t transd_array_calcflat( t_transd_array *transd_array, const uint8_t amplitude );

/* PRIVATE */

//uint8_t transd_init( t_transd *transd, const uint8_t index_x, const uint8_t index_y, const uint8_t port_pin, const int8_t phase_comp, const uint8_t elem_diameter, const uint8_t elem_separation );

//uint8_t transd_set( t_transd *transd, uint8_t port_pin, int8_t phase_comp );

//uint8_t transd_calcfocus( t_transd *transd, const uint8_t phase_res, const uint8_t curr_step, const uint8_t focus_x, const uint8_t focus_y, const uint8_t focus_z );

//uint8_t transd_calcflat( t_transd *transd, const uint8_t phase_res, const uint8_t curr_step, const uint8_t amplitude );


#ifdef __cplusplus
	}
#endif

#endif
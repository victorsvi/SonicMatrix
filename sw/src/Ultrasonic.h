
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>

typedef struct s_transd {
	uint8_t id; 
	uint8_t pin; 
	int8_t phase_comp; //physical phase delay  of the transducer
	uint8_t x; //x position of the center of the transducer in milimeters
	uint8_t y; //y position of the center of the transducer in milimeters
	uint8_t phase; //current discrete phase delay of the transducer
	uint8_t amplitude; //current discrete amplitude of the transducer
	struct s_transd *next; ///não usar lista?
} t_transd;

typedef struct s_array {
	uint8_t size_x; //number of transducers in the x axys
	uint8_t size_y; //number of transducers in the y axys
	uint8_t elem_diameter; //diameter of each transducer in milimeters
	uint8_t elem_separation; //separation between transducers in milimeters
	t_array *array; //pointer to the transducer list 
} t_array;

t_array *Array_init( uint8_t size_x, uint8_t size_y, uint8_t elem_diameter, uint8_t elem_separation );

t_transd *Transd_get( t_array *array, uint8_t id );

void Transd_set( t_array *array, uint8_t id, uint8_t pin, int8_t phase_comp );


#ifdef __cplusplus
	}
#endif

#endif
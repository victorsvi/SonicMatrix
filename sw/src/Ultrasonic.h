
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>

typedef struct s_transd {
	uint8_t port_pin; //pin on which the transducer is connected
	//uint8_t *port_bank; //pin's port bank
	//uint8_t port_bankidx; //pin's index on port bank
	int8_t phase_comp; //physical phase delay  of the transducer
	uint8_t x; //x position of the center of the transducer in milimeters
	uint8_t y; //y position of the center of the transducer in milimeters
	uint8_t phase; //current discrete phase delay of the transducer (how many steps the signal will delay. Depends on the signal resolution)
	uint8_t amplitude; //current discrete amplitude of the transducer (how many steps the signal will be on, like a duty cycle. Depends on the signal resolution)
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

uint8_t transd_array_set( t_transd_array *transd_array, const uint8_t index_x, const uint8_t index_y, const uint8_t port_pin, const int8_t phase_comp );

uint8_t transd_array_init( t_transd_array *transd_array, const uint8_t size_x, const uint8_t size_y, const uint8_t elem_diameter, const uint8_t elem_separation, const uint8_t phase_res );

uint8_t transd_array_calcfocus( t_transd_array *transd_array, const uint8_t focus_x, const uint8_t focus_y, const uint8_t focus_z );

uint8_t transd_array_calcflat( t_transd_array *transd_array, const uint8_t amplitude );

/* PRIVATE */

uint8_t transd_init( t_transd *transd, const uint8_t index_x, const uint8_t index_y, const uint8_t port_pin, const int8_t phase_comp, const uint8_t elem_diameter, const uint8_t elem_separation );

uint8_t transd_set( t_transd *transd, uint8_t port_pin, int8_t phase_comp );

uint8_t transd_calcfocus( t_transd *transd, const uint8_t phase_res, const uint8_t curr_step, const uint8_t focus_x, const uint8_t focus_y, const uint8_t focus_z );

uint8_t transd_calcflat( t_transd *transd, const uint8_t phase_res, const uint8_t curr_step, const uint8_t amplitude );


#ifdef __cplusplus
	}
#endif

#endif
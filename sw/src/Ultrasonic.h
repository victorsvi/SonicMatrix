
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>

#ifdef DEBUG
#include <stdio.h>
#endif

/* CONSTANTS */
#define TRANS_DIAMETER 16 //diameter of the element in millimeters (total length of the array cant exceed 255 millimeters)
#define TRANS_SEPARATION 2 //distance between two consecutive elements in the array in millimeters (total length of the array cant exceed 255 millimeters)
#define ARRAY_SIZE_X 8 //number of transducers of the array in the x dimension
#define ARRAY_SIZE_Y 8 //number of transducers of the array in the y dimension
#define ARRAY_PHASERES 10 //number of "slices" or "steps" that a period is divided into. It defines the resolution of the phase control (max 16 bits)

/* DATA TYPES */
/* represents a transducer of the array, grouping it's properties and state variables */
typedef struct s_transd { 
	uint8_t port_pin; //pin on which the transducer is connected.
	uint8_t phase_comp; //physical phase delay of the transducer. Use to compensate for manufacturing imperfections of the transducers.
	//uint8_t x; //x position of the center of the transducer in millimeters.
	//uint8_t y; //y position of the center of the transducer in millimeters.
	//uint8_t phase; //current discrete phase delay of the transducer. It ranges from 0 to 255 and equates to 0 to 2pi.
	//uint8_t duty_cycle; //current discrete duty cycle of the transducer. It ranges from 0 to 255 and equates to 0% to 100%.
	uint16_t pattern; //current pattern that represents the wave format. Each bit is a step of the waveform. The number of bits is limited by the phase_res. 
} t_transd;

/* represents the array of transducers */
/*
typedef struct s_transd_array {
	uint8_t size_x; //number of transducers in the x axis
	uint8_t size_y; //number of transducers in the y axis
	uint8_t elem_diameter; //diameter of each transducer in millimeters
	uint8_t elem_separation; //separation between transducers in millimeters
	TROCAR POR VETOR t_transd *transd_ptr; //pointer to the transducer array
	uint8_t phase_res; //Number of steps that a wave period is divided. Increase the phase_res to obtain a more accurate waveform. Decrease the phase_res if the uC can't update the outputs fast enough to support the 40kHz frequency.
} t_transd_array;
*/

/* VARIABLES */
extern t_transd transd_array[ARRAY_SIZE_X][ARRAY_SIZE_Y]; //defined in the Ultrasonic.c file. Should be read to access the resulting patterns

/* FUNCTIONS */
/*t_transd_array * transd_array_init( t_transd_array *transd_array, const uint8_t size_x, const uint8_t size_y, const uint8_t elem_diameter, const uint8_t elem_separation, const uint8_t phase_res );*/

uint8_t transd_array_set( /*t_transd_array *transd_array,*/ const uint8_t index_x, const uint8_t index_y, const uint8_t port_pin, const uint8_t phase_comp );

uint8_t transd_array_calcfocus( /*t_transd_array *transd_array,*/ const uint8_t duty_cycle, const uint8_t focus_x, const uint8_t focus_y, const uint8_t focus_z );

uint8_t transd_array_calcflat( /*t_transd_array *transd_array,*/ const uint8_t duty_cycle );

/*uint8_t transd_array_kill( t_transd_array *transd_array );*/

#ifdef DEBUG
uint8_t transd_dumpJSON ( FILE *f/*,  t_transd_array *transd_array*/ );

uint8_t transd_dumpCSV ( FILE *f/*,  t_transd_array *transd_array*/ );
#endif

#ifdef __cplusplus
	}
#endif

#endif

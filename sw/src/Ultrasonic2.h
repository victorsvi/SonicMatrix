
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>

#ifdef DEBUG
#include <stdio.h>
#endif

/* represents a transducer of the array, grouping it's properties and state variables */
typedef struct s_transd { 
	uint8_t port_pin; //pin on which the transducer is connected.
	uint8_t phase_comp; //physical phase delay of the transducer. Use to compensate for manufacturing imperfections of the transducers.
	uint8_t x; //x position of the center of the transducer in milimeters.
	uint8_t y; //y position of the center of the transducer in milimeters.
	uint8_t phase; //current discrete phase delay of the transducer. It ranges from 0 to 255 and equates to 0 to 2pi.
	uint8_t duty_cycle; //current discrete duty cycle of the transducer. It ranges from 0 to 255 and equates to 0% to 100%.
	uint16_t pattern; //current pattern thar represents the wave format. Each bit is a step of the waveform. The number of bits is limite by the phase_res. 
} t_transd;

uint8_t transd_array_init( t_transd *transd_array );

uint8_t transd_array_set(t_transd *transd_array, const uint8_t index_x, const uint8_t index_y, const uint8_t port_pin, const uint8_t phase_comp );

uint8_t transd_array_calcfocus( t_transd *transd_array, const uint8_t duty_cycle, const uint8_t focus_x, const uint8_t focus_y, const uint8_t focus_z );

uint8_t transd_array_calcflat( t_transd *transd_array, const uint8_t duty_cycle );

#ifdef DEBUG
uint8_t transd_dumpJSON ( FILE *f,  t_transd *transd_array );

uint8_t transd_dumpCSV ( FILE *f,  t_transd *transd_array );
#endif

#ifdef __cplusplus
	}
#endif

#endif

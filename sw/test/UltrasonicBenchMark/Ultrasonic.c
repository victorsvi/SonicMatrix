/* 
 * This file implements functions to manage a ultrasonic matrix.
 * The main objetive is to create a focal point near the matrix
 * where the sound pressure will peak.
 * 
 * Once the calculations to archive a focal point have been made
 * the driving signals of the transducers can be generated without
 * new calculations. The math is done only when the focal point
 * changes.
 * 
 * The principle to generate a focal point is calculate the
 * distance of each transducer and adjust the individual phases
 * so that all the signals arrive in phase at the focal point.
 * 
 * Each transducer will be assigned a Pattern that represents the
 * it's driving waveform. The driving signal is assumed to be a
 * square wave of 40kHz. If no phase control were needed, the 
 * pins would need to be fliped at 80kHz (two flips each period).
 * With phase control, the pins CAN be switched with a much higher
 * frequency, determined by the phase resolution. If phase
 * resolution is 10 the pin must be updated at 400kHz and the phase
 * can be set in pi/5 increments.
 * 
 * Each bit of the pattern represents the output of that transducer
 * pin at the equivalent "slice" of the period. The "slice" 0 is 
 * the lsb bit.
 * 
 * by Victor Salvi, 2020.
 */
#include <stdlib.h>
#include <stdint.h>

#include "Debug.h"
#include "Ultrasonic.h"

/*
#define SOUND_SPEED 343600 //speed of the sound in the medium in milimeters per second (at 20°C dry air)
#define SOUND_FREQ 40000  //frequency of the signal
#define SOUND_WAVELEN 8  //(SOUND_SPEED/SOUND_FREQ) - the wave length is 8.6 mm
#define TRANSD_SIZE_X 8 //number of transducers in the x axys
#define TRANSD_SIZE_Y 8 //number of transducers in the y axys
#define TRANSD_DIAM 16 //diameter of each transducer in milimeters
#define TRANSD_SEPAR 2 //separation between transducers in milimeters
#define TRANSD_PHRES 10 //Number of steps that a wave period is divided. Increase the phase_res to obtain a more accurate wavefrom. Decrease the phase_res if the uC can't update the outputs fast enough to support the 40kHz frequency.
*/

uint32_t transd_sqrt_int (const uint32_t x);
uint16_t transd_getPattern( const uint8_t phase_res, const uint8_t phase, const uint8_t duty );

uint8_t transd_init( t_transd *transd, const uint8_t index_x, const uint8_t index_y, const uint8_t port_pin, const uint8_t phase_comp, const uint8_t elem_diameter, const uint8_t elem_separation );
uint8_t transd_set( t_transd *transd, uint8_t port_pin, uint8_t phase_comp );
uint8_t transd_calcfocus( t_transd *transd, const uint8_t phase_res, const uint8_t duty_cycle, const uint8_t focus_x, const uint8_t focus_y, const uint8_t focus_z );
uint8_t transd_calcflat( t_transd *transd, const uint8_t phase_res, const uint8_t duty_cycle );

/* PUBLIC */

/**
 * Creates and initializes an transducer array.
 * @param size_x number of transducers in the x axys.
 * @param size_y number of transducers in the y axys.
 * @param elem_diameter diameter of each transducer in milimeters.
 * @param elem_separation separation between transducers in milimeters
 * @param phase_res Number of steps that a wave period is divided. Increase the phase_res to obtain a more accurate wavefrom. Decrease the phase_res if the uC can't update the outputs fast enough to support the 40kHz frequency.
 * @return A pointer to the new instance of a transducer array.
 */
t_transd_array * transd_array_init( /*t_transd_array *transd_array,*/ const uint8_t size_x, const uint8_t size_y, const uint8_t elem_diameter, const uint8_t elem_separation, const uint8_t phase_res ) {	
	/*uint8_t ret; deprecated because the function now returns a pointer */
	uint8_t x,y;
	t_transd_array *transd_array;
		
	if(size_x == 0 || size_y == 0) {
		#ifdef DEBUG 
		DEBUG_MSG("Invalid size for the transducer array") 
		#endif
		return NULL;
	}

	if(elem_diameter == 0) {
		#ifdef DEBUG 
		DEBUG_MSG("Invalid value for transducer diameter") 
		#endif
		return NULL;
	}
	
	if(phase_res < 2 || phase_res > 16) { //minimum of 2 and maximum of 16 bits for resolution
		#ifdef DEBUG 
		DEBUG_MSG("Invalid value for phase resolution") 
		#endif
		return NULL;
	}
	
	transd_array = (t_transd_array*) malloc(sizeof(t_transd_array));
	if(transd_array == NULL){
		#ifdef DEBUG 
		DEBUG_MSG("Memory allocation error") 
		#endif
		return NULL;
	}
	
	transd_array->size_x = size_x;
	transd_array->size_y = size_y;
	transd_array->elem_diameter = elem_diameter;
	transd_array->elem_separation = elem_separation;
	transd_array->phase_res = phase_res;
	
	//the number of transducers of the array equals size_x*size_y
	transd_array->transd_ptr = (t_transd*) malloc(sizeof(t_transd) * size_x * size_y);
	if(transd_array->transd_ptr == NULL){
		#ifdef DEBUG 
		DEBUG_MSG("Memory allocation error") 
		#endif
		return NULL;
	}
	
	for(x = 0; x < size_x; x++){
		for(y = 0; y < size_y; y++){
			
			/*ret =*/ transd_init( (transd_array->transd_ptr + x * size_y + y), x, y, 0, 0, elem_diameter, elem_separation );
			/*if(ret != 0) { deprecated because the function now returns a pointer 
				return ret;
			}*/
		}
	}
	
	return transd_array;
} //transd_array_init

/**
 * Sets the port and phase compensation of a transducer in the array.
 * @param transd_array The pointer to the array.
 * @param index_x The index of the transducer in the array on the x direction.
 * @param index_y The index of the transducer in the array on the y direction.
 * @param port_pin The number of the pin on which the transducer is connected.
 * @param phase_comp The discrete phase offset of the transducer. It ranges from 0 to 255 and equates to 0 to 2pi.
 * @return 0 for sucess, an error code otherwise.
 */
uint8_t transd_array_set( t_transd_array *transd_array, const uint8_t index_x, const uint8_t index_y, const uint8_t port_pin, const uint8_t phase_comp ){
	
	uint8_t ret;
	
	if(transd_array == NULL) {
		#ifdef DEBUG 
		DEBUG_MSG("The transducer array pointer is null") 
		#endif
		return 40;	
	}
	
	if(transd_array->transd_ptr == NULL) {
		#ifdef DEBUG 
		DEBUG_MSG("The transducer pointer is null") 
		#endif
		return 44;	
	}
	
	ret = transd_set( (transd_array->transd_ptr + index_x * transd_array->size_y + index_y), port_pin, phase_comp );
	
	return ret;
} //transd_array_set

/** 
 * Calculates the parameters for each transducer to generate a focal point at a three-dimensional coordinate.
 * @param transd_array The pointer to the array.
 * @param duty_cycle Discrete duty cycle of the transducer. It ranges from 0 to 255 and equates to 0% to 100%.
 * @param focus_x The x coordinate of the focal point in milimeters.
 * @param focus_y The y coordinate of the focal point in milimeters.
 * @param focus_z The z coordinate of the focal point in milimeters.
 * @return 0 for sucess, an error code otherwise.
 */
uint8_t transd_array_calcfocus( t_transd_array *transd_array, const uint8_t duty_cycle, const uint8_t focus_x, const uint8_t focus_y, const uint8_t focus_z ) {
	
	uint8_t ret;
	uint8_t x,y;
	
	if(transd_array == NULL) {
		#ifdef DEBUG 
		DEBUG_MSG("The transducer array pointer is null") 
		#endif
		return 40;	
	}
	
	if(transd_array->transd_ptr == NULL) {
		#ifdef DEBUG 
		DEBUG_MSG("The transducer pointer is null") 
		#endif
		return 44;	
	}
	
	for(x = 0; x < transd_array->size_x; x++){
		for(y = 0; y < transd_array->size_y; y++){
			
			ret = transd_calcfocus( (transd_array->transd_ptr + x * transd_array->size_y + y), transd_array->phase_res, duty_cycle, focus_x, focus_y, focus_z );
			if(ret != 0) {
				return ret;
			}
		}
	}
	
	return 0;
} //transd_array_calcfocus

/** 
 * Calculates the parameters for each transducer to generate a synchronized in phase signal for all the transducers.
 * @param transd_array The pointer to the array.
 * @param duty_cycle Discrete duty cycle of the transducer. It ranges from 0 to 255 and equates to 0% to 100%.
 * @return 0 for sucess, an error code otherwise.
 */
uint8_t transd_array_calcflat( t_transd_array *transd_array, const uint8_t duty_cycle ) {
	
	uint8_t ret;
	uint8_t x,y;
	
	if(transd_array == NULL) {
		#ifdef DEBUG 
		DEBUG_MSG("The transducer array pointer is null") 
		#endif
		return 40;	
	}
	
	if(transd_array->transd_ptr == NULL) {
		#ifdef DEBUG 
		DEBUG_MSG("The transducer pointer is null") 
		#endif
		return 44;	
	}
	
	for(x = 0; x < transd_array->size_x; x++){
		for(y = 0; y < transd_array->size_y; y++){
			
			ret = transd_calcflat( (transd_array->transd_ptr + x * transd_array->size_y + y), transd_array->phase_res, duty_cycle );
			if(ret != 0) {
				return ret;
			}
		}
	}
	
	return 0;
} //transd_array_calcflat

/** 
 * Deallocate a transducer array.
 * @param transd_array The pointer to the array.
 * @return 0 for sucess, an error code otherwise.
 */
uint8_t transd_array_kill( t_transd_array *transd_array ){
	
	if(transd_array == NULL) {
		#ifdef DEBUG 
		DEBUG_MSG("The transducer array pointer is null") 
		#endif
		return 40;	
	}
	
	if(transd_array->transd_ptr == NULL) {
		#ifdef DEBUG 
		DEBUG_MSG("The transducer pointer is null") 
		#endif
		return 44;	
	}
	
	free(transd_array->transd_ptr);
	
	free(transd_array);
	
	return 0;
} //transd_array_kill

#ifdef DEBUG

/**
 * Dumps the array structure to a JSON file.
 * @param f The file pointer.
 * @param transd_array The pointer to the array.
 * @return 0 for sucess, an error code otherwise.
 */
uint8_t transd_dumpJSON ( FILE *f,  t_transd_array *transd_array ){
	
	char buff[1024];
	char patt[17];
	t_transd *transd_ptr;
	uint8_t i,x,y;
	uint16_t mask;
	
	if(f == NULL) {
		#ifdef DEBUG 
		DEBUG_MSG("Unable to open file") 
		#endif
		return 2;
	}
	
	//the array constants data; Opens the array of transducers
	sprintf(buff, "{\n\t\"size_x\": %u,\n\t\"size_y\": %u,\n\t\"elem_diameter\": %u,\n\t\"elem_separation\": %u,\n\t\"phase_res\": %u,\n\t\"transd_ptr\": [", transd_array->size_x, transd_array->size_y, transd_array->elem_diameter, transd_array->elem_separation, transd_array->phase_res);
	fputs(buff, f);
	
	for(x = 0; x < transd_array->size_x; x++){
		for(y = 0; y < transd_array->size_y; y++){
			
			if( x != 0 || y != 0 ) {
				fputc(',',f);
			}
			
			transd_ptr = (transd_array->transd_ptr + x * transd_array->size_y + y);
			
			//converts the 16 bit pattern to a 16 characters string
			//the '#' represents a active output and the '_' represents a inactive output
			for(i = 0; i < 16; i++) {
				
				mask = (1 << (i)); //creates a mask with the current bit = 1
				if(transd_ptr->pattern & mask) { //if the pattern have the current bit = 1
					//patt[15 - i] = '#'; //the signal starts at the lsb of the pattern
					patt[transd_array->phase_res - 1 - i] = '#'; //upgraded to show only the used bits of the pattern
				}
				else {
					//patt[15 - i] = '_';
					patt[transd_array->phase_res - 1 - i] = '_'; //upgraded to show only the used bits of the pattern
				}
			}
			//patt[16] = '\0';
			patt[transd_array->phase_res] = '\0'; //upgraded to show only the used bits of the pattern
			
			//the current transducer data
			sprintf(buff, "\n\t{\n\t\t\"port_pin\": %u,\n\t\t\"phase_comp\": %u,\n\t\t\"x\": %u,\n\t\t\"y\": %u,\n\t\t\"phase\": %u,\n\t\t\"duty_cycle\": %u,\n\t\t\"pattern\": \"%s\"\n\t}", transd_ptr->port_pin, transd_ptr->phase_comp, transd_ptr->x, transd_ptr->y, transd_ptr->phase, transd_ptr->duty_cycle, patt);
			fputs(buff, f);

		}
	}
	//Closes the array of transducers
	sprintf(buff, "]\n}\n");
	fputs(buff, f);
	
	return 0;
}

/**
 * Dumps the transducers state of the array to a CSV file.
 * @param f The file pointer.
 * @param transd_array The pointer to the array.
 * @return 0 for sucess, an error code otherwise.
 */
uint8_t transd_dumpCSV ( FILE *f,  t_transd_array *transd_array ){
	
	char buff[1024];
	char patt[17];
	t_transd *transd_ptr;
	uint8_t i,x,y;
	uint16_t mask;
	
	if(f == NULL) {
		#ifdef DEBUG 
		DEBUG_MSG("Unable to open file") 
		#endif
		return 2;
	}
	
	//prints the header of the collumns
	sprintf(buff, "idx_x;idx_y;port_pin;phase_comp;dist_x;dist_y;phase;duty_cycle;pattern\n");
	fputs(buff, f);
	
	for(x = 0; x < transd_array->size_x; x++){
		for(y = 0; y < transd_array->size_y; y++){
			
			transd_ptr = (transd_array->transd_ptr + x * transd_array->size_y + y);
			
			//converts the 16 bit pattern to a 16 characters string
			//the '#' represents a active output and the '_' represents a inactive output
			for(i = 0; i < 16; i++) {
				
				mask = (1 << (i)); //creates a mask with the current bit = 1
				if(transd_ptr->pattern & mask) { //if the pattern have the current bit = 1
					//patt[15 - i] = '#'; //the signal starts at the lsb of the pattern
					patt[transd_array->phase_res - 1 - i] = '#'; //upgraded to show only the used bits of the pattern
				}
				else {
					//patt[15 - i] = '_';
					patt[transd_array->phase_res - 1 - i] = '_'; //upgraded to show only the used bits of the pattern
				}
			}
			//patt[16] = '\0';
			patt[transd_array->phase_res] = '\0'; //upgraded to show only the used bits of the pattern
			
			//the current transducer data
			sprintf(buff, "%u;%u;%u;%u;%u;%u;%u;%u;%s\n", x, y, transd_ptr->port_pin, transd_ptr->phase_comp, transd_ptr->x, transd_ptr->y, transd_ptr->phase, transd_ptr->duty_cycle, patt);
			fputs(buff, f);

		}
	}
		
	return 0;
}
#endif



/* PRIVATE */

/** 
 * Performs the integer square root (truncated). Uses an iterative method
 * @parm x Number to take the sqare root.
 * @return The square root of x.
 */
uint16_t transd_sqrt_int (const uint16_t x) {

    uint16_t l = 1; // the lowest guess number that can be the root of x
    uint16_t h = x; // the highest guess number that can be the root of x
    uint16_t n = (l + h)/2; //the current guess being tested

    while ( l < n ){ //the stop condition is when the lowest possible guess is equal or greater the hisghest guess.

        if( (n * n) <= x ){ //if the guess is too small, updates increases the lowest guess
            l = n;
        }
        else if ( (n * n) > x ) { //if the guess is too big, updates decreases the hisghest guess
            h = n;
        }
        n = (l + h)/2; //the new guess is a midpoint

    };

    return l;
}

/**
 * Calculates the 16 bit pattern that represents the signal based on the phase and duty cycle.
 * The pattern is a sequence of bits that represents one period of the output signal. Each bit defines if the output is high (1) or low (0) at each step of the period.
 * The phase_res defines how many steps one period have (how many bits the pattern will have). The higher the resolution, the more accurate the wave will be. The lsb is the frist bit of the period and the msb is the last.
 * @parm phase_res Number of steps that a wave period is divided into.
 * @param phase The discrete phase of the transducer. It ranges from 0 to 255 and equates to 0 to 2pi.
 * @param duty Discrete duty cycle of the transducer. It ranges from 0 to 255 and equates to 0% to 100%.
 * @return The pattern.
 */
uint16_t transd_getPattern( const uint8_t phase_res, const uint8_t phase, const uint8_t duty ){
	
	uint8_t bits_duty, bits_phase;
	uint16_t pattern = 0;
	
	/*
		duty -> defines the quantity of 1s over 0s (if duty is percentage -> bits on = duty/resolution; 3 = 30/10)
		30% -> _______###
		50% -> _____#####
	*/
	bits_duty = (((duty * 100) / 255) / phase_res); //equation for duty domain [0,255]
	//the odrer of the operations is important to avoid truncate or overflow
	
	/*
		phase -> defines how much delay the signal have (if phase if degree - > displaced bits = phase / (360/resolution); 2 = 72 / 360 / 10)
		0   -> _____##### (0°)
		2   -> ___#####__ (72°)
		8   -> sssssssssss (5 amp, 8 ph)
	*/
	bits_phase = (phase * phase_res) / 255; //equation for phase domain [0,255]
	
	/* DEPRECATED
	//sets the (Nth + 1) bit = 1 (00001000b for n = 3) then subtracts one to make all the lesser bits = 1 (00000111b for n = 3) 
	pattern = (1 << bits_duty) - 1; 
	
	//shifts the duty cycle pattern to change phase
	pattern <<= bits_phase;
	
	//if shifting bit for phase "overflows" the number of bits of the pattern (defined by the resolution), the pattern lost some duty cycle information (the bits that overflow when shifting)
	if(bits_phase + bits_duty > phase_res) {
		//generates the bits that were lost on phase shitfing at the LSB side
		pattern |= (1 << (bits_duty + bits_phase - phase_res)) - 1;
	}
	*/
	if(bits_phase + bits_duty > phase_res) { //if if the high bits will overflow outside the bits defined by the resolution
		//inverted case: the high bits overflow: ##_____### (50% 288°; duty = 128 phase = 204; bits_duty = 5 bits_phase = 8)
		//generate the inverse pattern then invert it
		
		bits_duty = phase_res - bits_duty;
		bits_phase = bits_duty + bits_phase - phase_res;
		pattern = (1 << bits_duty) - 1; 
		pattern <<= bits_phase;
		pattern = ~pattern;
	}
	else {
		//normal case: the high bits do not overflow: ___#####__ (50% 72°; duty = 128 phase = 51; bits_duty = 5 bits_phase = 2)
		
		//sets the (Nth + 1) bit = 1 (00001000b for n = 3) then subtracts one to make all the lesser bits = 1 (00000111b for n = 3)  
		pattern = (1 << bits_duty) - 1; 
		
		//shifts the duty cycle pattern to change phase
		pattern <<= bits_phase;
	}
		
	return pattern;
}

/**
 * Initializes an transducer of the array. Saves the pin and phase compensation. Calculates the physical position of the transducer on the array.
 * @param transd The pointer to the transducer on the array.
 * @param index_x The index of the transducer in the array on the x direction.
 * @param index_y The index of the transducer in the array on the y direction.
 * @param port_pin The number of the pin on which the transducer is connected.
 * @param phase_comp The discrete phase offset of the transducer. It ranges from 0 to 255 and equates to 0 to 2pi.
 * @param elem_diameter diameter of each transducer in milimeters.
 * @param elem_separation separation between transducers in milimeters
 * @return 0 for sucess, an error code otherwise.
 */
uint8_t transd_init( t_transd *transd, const uint8_t index_x, const uint8_t index_y, const uint8_t port_pin, const uint8_t phase_comp, const uint8_t elem_diameter, const uint8_t elem_separation ) {
	
	if(transd == NULL) {
		#ifdef DEBUG 
		DEBUG_MSG("The transducer pointer is null") 
		#endif
		return 44;	
	}
	
	if(elem_diameter == 0) {
		#ifdef DEBUG 
		DEBUG_MSG("Invalid value for transducer diameter") 
		#endif
		return 42;
	}
			
	transd->port_pin = port_pin;
	transd->phase_comp = phase_comp;
	transd->x = ((elem_diameter / 2) + (elem_diameter * index_x) + (elem_separation * index_x)); //calculates the position of the transducer on the array
	transd->y = ((elem_diameter / 2) + (elem_diameter * index_y) + (elem_separation * index_y)); //calculates the position of the transducer on the array
	transd->phase = 0;
	transd->duty_cycle = 0;
	transd->pattern = 0;
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
	 
	 return 0;
} //transd_init

/**
 * Updates the pin and phase compensation attributes of an transducer of the array.
 * @param transd The pointer to the transducer on the array.
 * @param port_pin The number of the pin on which the transducer is connected.
 * @param phase_comp The discrete phase offset of the transducer. It ranges from 0 to 255 and equates to 0 to 2pi.
 * @return 0 for sucess, an error code otherwise.
 */
uint8_t transd_set( t_transd *transd, uint8_t port_pin, uint8_t phase_comp ) {
	
	if(transd == NULL) {
		#ifdef DEBUG 
		DEBUG_MSG("The transducer pointer is null") 
		#endif
		return 44;	
	}
	
	transd->port_pin = port_pin;
	transd->phase_comp = phase_comp;
	
	return 0;	
} //transd_set

/** 
 * Calculates the parameters for an transducer so its signal arrive in phase at a three-dimensional coordinate.
 * @param transd The pointer to the transducer on the array.
 * @parm phase_res Number of steps that a wave period is divided into.
 * @param duty_cycle Discrete duty cycle of the transducer. It ranges from 0 to 255 and equates to 0% to 100%.
 * @param focus_x The x coordinate of the focal point in milimeters.
 * @param focus_y The y coordinate of the focal point in milimeters.
 * @param focus_z The z coordinate of the focal point in milimeters.
 * @return 0 for sucess, an error code otherwise.
 */
uint8_t transd_calcfocus( t_transd *transd, const uint8_t phase_res, const uint8_t duty_cycle, const uint8_t focus_x, const uint8_t focus_y, const uint8_t focus_z ){
	
	uint16_t distance; //type long because it can overflow for some big distances
	uint16_t phase;
	int8_t Dx,Dy; //may be smaller than zero
	
	if(transd == NULL) {
		#ifdef DEBUG 
		DEBUG_MSG("The transducer pointer is null") 
		#endif
		return 44;	
	}
	
	/*
		W = S / f (wave length = sound speed / frequency)

		Dij = ( (Xij - x)^2 + (Yij - y)^2 + (-z)^2 )^1/2 (distance from the ij transducer to the focus point)

		Pij = ( Dij * 255 ) / W  (phase of the ij transducer to have 0° of phase at the focus point)
	*/
	Dx = transd->x - focus_x; //distance on the x axys
	Dy = transd->y - focus_y; //distance on the y axys
	distance = (Dx * Dx) + (Dy * Dy) + (focus_z * focus_z); //squared distance
	distance *= 100; //multiplies by 10 squared to "keep" one decimal place after the square root.
	distance = transd_sqrt_int(distance); //x10. After taking the square root, the distance is not in milimeters
	phase = (( distance * 2550) / 860); //(dist * 255 / 8.6) //As the distance is multiplied by 10, this equation does an extra by 10 division
	phase += transd->phase_comp; //adds the phase compensation. This is the total phase displacement that the ware does between the transducer and the focal point.
	transd->phase = (uint8_t) (phase % 255); //As the phase is periodic, gets the mod 255 value (255 = 360 degrees)
	
	transd->duty_cycle = duty_cycle;
	transd->pattern = transd_getPattern( phase_res, transd->phase, transd->duty_cycle ); 
	
	return 0;	
} //transd_calcfocus

/** 
 * Calculates the parameters for an transducer so its signal is in phase with all the other transducers.
 * @param transd The pointer to the transducer on the array.
 * @parm phase_res Number of steps that a wave period is divided into.
 * @param duty_cycle Discrete duty cycle of the transducer. It ranges from 0 to 255 and equates to 0% to 100%.
 * @return 0 for sucess, an error code otherwise.
 */
uint8_t transd_calcflat( t_transd *transd, const uint8_t phase_res, const uint8_t duty_cycle ) {
	
	if(transd == NULL) {
		#ifdef DEBUG 
		DEBUG_MSG("The transducer pointer is null")
		#endif
		return 44;	
	}
	
	transd->phase = transd->phase_comp; //the phase is set to zero but the pattern can be delayed because of the phase compensation
	transd->duty_cycle = duty_cycle;
	transd->pattern = transd_getPattern( phase_res, transd->phase, transd->duty_cycle ); 
	
	return 0;	
} //transd_calcflat


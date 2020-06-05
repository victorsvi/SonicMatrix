/*
 * This file implements functions to manage a ultrasonic matrix.
 * The main objective is to create a focal point near the matrix
 * where the sound pressure will peak.
 *
 * Once the calculations to archive a focal point have been made
 * the driving signals of the transducers can be generated without
 * new calculations (the result is buffered). The math is done
 * only when the focal point changes.
 *
 * The principle to generate a focal point is calculate the
 * distance of each transducer and adjust the individual phases
 * so that all the signals arrive in phase at the focal point.
 *
 * Each transducer will be assigned a Pattern that represents the
 * it's driving waveform. The driving signal is assumed to be a
 * square wave of 40kHz. If no phase control were needed, the
 * pins would need to be flipped at 80kHz (two flips each period).
 * With phase control, the pins CAN be switched with a much higher
 * frequency, determined by the phase resolution. If phase
 * resolution is 10 the pin must be updated at 400kHz and the phase
 * can be set in pi/5 increments (phase resolution).
 *
 * Each bit of the pattern represents the output of that transducer
 * pin at the equivalent "slice" of the period. The "slice" 0 is
 * the lsb bit.
 *
 * As an example of how to interpret the pattern and create an output
 * signal, lets analyze a pattern represented in binary for a phase
 * resolution of 10 bits. The ones means output active and zeros means
 * output inactive:
 * (msb) 0000111100 (lsb)
 * As only 4/10 of the period is active, the duty cycle is 40%
 * As it has 2 zeros by the lsb, the phase is 2/10 of 2pi (2pi/5)
 * At this resolution, the output needs to be updated at 400kHz 
 * (10 * 40kHz) as 10 updates will represent 1 period of the 40kHz
 * resonant frequency of the transducers.
 * 
 * There are four parameters that defines the geometry of the
 * matrix, which is a rectangle filled with transducers.
 * An additional parameters defines the phase resolution. Mind that
 * a bigger phase resolution will increase the focusing capabilities
 * but will demand more hardware capacity.
 * These parameters are defined constants that can be adjusted in the 
 * header file.
 *
 * Down is a illustration of an 8x4 matrix. Each O is a transducer.
 * There are the x and y axis and the o is the origin of the coordinates
 * the (0,0) transducer.
 *
 *  y|
 *   |   ---------------
 *   |  |O O O O O O O O|
 *   |  |O O O O O O O O|
 *   |  |O O O O O O O O|
 *   |  |O O O O O O O O|
 *   |  o---------------
 *   |______________________________
 *                                 x
 *  
 * by Victor Salvi (victorsvi@gmail.com), 2020.
 */
 
#include <stdlib.h>
#include <stdint.h>

#include "Debug.h"
#include "Ultrasonic.h"
#include "Math.h"

/* CONSTANTS 
#define SOUND_SPEED 343600 //speed of the sound in the medium in millimeters per second (at 20°C dry air)
#define SOUND_FREQ 40000  //frequency of the signal
#define SOUND_WAVELEN 8  //(SOUND_SPEED/SOUND_FREQ) - the wave length is 8.6 mm
*/

/* DATA TYPES */
#ifdef DEBUG
typedef struct s_transd_debug {
	uint8_t dist_x; //x position of the center of the transducer in millimeters.
	uint8_t dist_y; //y position of the center of the transducer in millimeters.
	uint8_t dist; //dist between the center of the transducer and the focal point in millimeters.
	uint8_t phase; //current discrete phase delay of the transducer. It ranges from 0 to 255 and equates to 0 to 2pi.
	uint8_t duty_cycle; //current discrete duty cycle of the transducer. It ranges from 0 to 255 and equates to 0% to 100%.
} t_transd_debug;
#endif

/* VARIABLES */
t_transd transd_array[ARRAY_SIZE_X][ARRAY_SIZE_Y]; //This array represents the matrix of transducers and the elements are indexed by their x and y ordering in the matrix
#ifdef DEBUG
t_transd_debug transd_array_debug[ARRAY_SIZE_X][ARRAY_SIZE_Y]; //to debug the pattern calculations
#endif

/* FUNCTIONS */
uint16_t transd_getPattern( /*const uint8_t phase_res,*/ const uint8_t phase, const uint8_t duty );
// uint8_t transd_init( t_transd *transd, const uint8_t index_x, const uint8_t index_y, const uint8_t port_pin, const uint8_t phase_comp, const uint8_t elem_diameter, const uint8_t elem_separation );
// uint8_t transd_set( t_transd *transd, uint8_t port_pin, uint8_t phase_comp );
// uint8_t transd_calcfocus( t_transd *transd, const uint8_t phase_res, const uint8_t duty_cycle, const uint8_t focus_x, const uint8_t focus_y, const uint8_t focus_z );
// uint8_t transd_calcflat( t_transd *transd, const uint8_t phase_res, const uint8_t duty_cycle );

/* PUBLIC */

/*
 * Creates and initializes an transducer array.
 * @param size_x number of transducers in the x axis.
 * @param size_y number of transducers in the y axis.
 * @param elem_diameter diameter of each transducer in millimeters.
 * @param elem_separation separation between transducers in millimeters
 * @param phase_res Number of steps that a wave period is divided. Increase the phase_res to obtain a more accurate waveform. Decrease the phase_res if the uC can't update the outputs fast enough to support the 40kHz frequency.
 * @return A pointer to the new instance of a transducer array.
 */
/*t_transd_array * transd_array_init( t_transd_array *transd_array, const uint8_t size_x, const uint8_t size_y, const uint8_t elem_diameter, const uint8_t elem_separation, const uint8_t phase_res ) {
	//uint8_t ret; deprecated because the function now returns a pointer
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

			transd_init( (transd_array->transd_ptr + x * size_y + y), x, y, 0, 0, elem_diameter, elem_separation );
			//ret = transd_init( (transd_array->transd_ptr + x * size_y + y), x, y, 0, 0, elem_diameter, elem_separation );
			//if(ret != 0) { deprecated because the function now returns a pointer
				//return ret;
			//}
		}
	}

	return transd_array;
}*/ //transd_array_init

/**
 * Sets the port and phase compensation of a transducer in the array.
 * @param index_x The index of the transducer in the array on the x direction.
 * @param index_y The index of the transducer in the array on the y direction.
 * @param port_pin The number of the pin on which the transducer is connected.
 * @param phase_comp The discrete phase offset of the transducer. It ranges from 0 to 255 and equates to 0 to 2pi.
 * @return 0 for success, an error code otherwise.
 */
uint8_t transd_array_set( /*t_transd_array *transd_array,*/ const uint8_t index_x, const uint8_t index_y, const uint8_t port_pin, const uint8_t phase_comp ){

	//uint8_t ret;

	/*if(transd_array == NULL) {
		#ifdef DEBUG
		DEBUG_MSG("The transducer array pointer is null")
		#endif
		return 40;
	}*/

	/*if(transd_array->transd_ptr == NULL) {
		#ifdef DEBUG
		DEBUG_MSG("The transducer pointer is null")
		#endif
		return 44;
	}*/
	
	if(index_x >= ARRAY_SIZE_X) {
		#ifdef DEBUG
		DEBUG_MSG("index_x out of bounds")
		#endif
		return 4;
	}
	
	if(index_y >= ARRAY_SIZE_Y) {
		#ifdef DEBUG
		DEBUG_MSG("index_y out of bounds")
		#endif
		return 4;
	}

	/*ret = transd_set( (transd_array->transd_ptr + index_x * transd_array->size_y + index_y), port_pin, phase_comp );*/

	transd_array[index_x][index_y].port_pin = port_pin;
	transd_array[index_x][index_y].phase_comp = phase_comp;

	//return ret;
	return 0;
} //transd_array_set

/**
 * Calculates the parameters for each transducer to generate a focal point at a three-dimensional coordinate.
 * @param duty_cycle Discrete duty cycle of the transducer. It ranges from 0 to 255 and equates to 0% to 100%.
 * @param focus_x The x coordinate of the focal point in millimeters.
 * @param focus_y The y coordinate of the focal point in millimeters.
 * @param focus_z The z coordinate of the focal point in millimeters.
 * @return 0 for sucess, an error code otherwise.
 */
uint8_t transd_array_calcfocus( /*t_transd_array *transd_array,*/ const uint8_t duty_cycle, const uint8_t focus_x, const uint8_t focus_y, const uint8_t focus_z ) {

	//uint8_t ret;
	uint8_t x,y;
	t_transd *transd = NULL;
	
	uint32_t distance; //type long because it can overflow for some big distances
	uint8_t phase;
	int16_t dist_x,dist_y; //may be smaller than zero

	/*if(transd_array == NULL) {
		#ifdef DEBUG
		DEBUG_MSG("The transducer array pointer is null")
		#endif
		return 40;
	}*/

	/*if(transd_array->transd_ptr == NULL) {
		#ifdef DEBUG
		DEBUG_MSG("The transducer pointer is null")
		#endif
		return 44;
	}*/

	transd = &transd_array[0][0]; //current element being iterated (note that the order of the for loops cant be changed because this pointer is incremented)
	for(x = 0; x < ARRAY_SIZE_X; x++){
		for(y = 0; y < ARRAY_SIZE_Y; y++){

			// ret = transd_calcfocus( (transd_array->transd_ptr + x * transd_array->size_y + y), transd_array->phase_res, duty_cycle, focus_x, focus_y, focus_z );
			// if(ret != 0) {
				// return ret;
			// }
			
			// here is an example of the calculation of the position in one axis (elem_diameter = 16, elem_separation = 2)

			/* diameter           separation
			  |-------|          |--|
			    _____      _____      _____
			   /     \    /     \    /     \
			  |   -   |  |   -   |  |   -   |
			   \_____/    \_____/    \_____/

			  |   |   |  |   |   |  |   |   |
			  0   8  16  18  26 34  36  44 52
			  ------------------------------->
       */
			dist_x = (TRANS_DIAMETER / 2) + ((TRANS_DIAMETER+TRANS_SEPARATION) * x); //calculates the x position of the center of the transducer on the array
			dist_y = (TRANS_DIAMETER / 2) + ((TRANS_DIAMETER+TRANS_SEPARATION) * y); //calculates the y position of the center of the transducer on the array
			#ifdef DEBUG
			transd_array_debug[x][y].dist_x = dist_x;
			transd_array_debug[x][y].dist_y = dist_y;
			#endif
			/*
				W = S / f (wave length = sound speed / frequency)

				Dij = ( (Xij - x)^2 + (Yij - y)^2 + (-z)^2 )^1/2 (distance from the ij transducer to the focus point)

				Pij = ( Dij * 255 ) / W  (phase of the ij transducer to have 0° of phase at the focus point)
			*/
			dist_x = dist_x - focus_x; //distance on the x axis
			dist_y = dist_y - focus_y; //distance on the y axis
			distance = (dist_x * dist_x) + (dist_y * dist_y) + (focus_z * focus_z); //squared distance
			distance *= 100; //multiplies by 10 squared to "keep" one decimal place after the square root.
			distance = transd_sqrt_int(distance); //x10. After taking the square root, the distance is not in millimeters
			phase = (( distance * 2550) / 860); //(dist * 255 / 8.6) //As the distance is multiplied by 10, this equation does an extra by 10 division
			phase += transd->phase_comp; //adds the phase compensation. This is the total phase displacement that the ware does between the transducer and the focal point.

			transd->pattern = transd_getPattern( /*phase_res,*/ phase, duty_cycle );

			#ifdef DEBUG
			transd_array_debug[x][y].dist=distance/10;
			transd_array_debug[x][y].phase=phase;
			transd_array_debug[x][y].duty_cycle=duty_cycle;
			#endif
			
			transd++; //if you change the order of iteration, this line needs to change (when iterating the first dimension then the second dimension, we replicate the order that the compiler puts the elements in linear memory)
		
		}
	}
		
	return 0;
} //transd_array_calcfocus

/**
 * Calculates the parameters for each transducer to generate a synchronized in phase signal for all the transducers (focal point at infinity on z axis).
 * @param duty_cycle Discrete duty cycle of the transducer. It ranges from 0 to 255 and equates to 0% to 100%.
 * @return 0 for success, an error code otherwise.
 */
uint8_t transd_array_calcflat( /*t_transd_array *transd_array,*/ const uint8_t duty_cycle ) {

	//uint8_t ret;
	uint8_t x,y;
	t_transd *transd = NULL;

	/*if(transd_array == NULL) {
		#ifdef DEBUG
		DEBUG_MSG("The transducer array pointer is null")
		#endif
		return 40;
	}*/

	/*if(transd_array->transd_ptr == NULL) {
		#ifdef DEBUG
		DEBUG_MSG("The transducer pointer is null")
		#endif
		return 44;
	}*/
			
	transd = &transd_array[0][0]; //current element being iterated (note that the order of the for loops cant be changed because this pointer is incremented)
	for(x = 0; x < ARRAY_SIZE_X; x++){
		for(y = 0; y < ARRAY_SIZE_Y; y++){

			// ret = transd_calcflat( (transd_array->transd_ptr + x * transd_array->size_y + y), transd_array->phase_res, duty_cycle );
			// if(ret != 0) {
				// return ret;
			// }
			
			#ifdef DEBUG
			transd_array_debug[x][y].dist_x = ((TRANS_DIAMETER / 2) + ((TRANS_DIAMETER+TRANS_SEPARATION) * x); //calculates the x position of the center of the transducer on the array
			transd_array_debug[x][y].dist_y = ((TRANS_DIAMETER / 2) + ((TRANS_DIAMETER+TRANS_SEPARATION) * y); //calculates the y position of the center of the transducer on the array
			#endif

			transd->pattern = transd_getPattern( /*phase_res,*/ transd->phase_comp, duty_cycle );

			#ifdef DEBUG
			transd_array_debug[x][y].dist=0;
			transd_array_debug[x][y].phase=transd->phase_comp;
			transd_array_debug[x][y].duty_cycle=duty_cycle;
			#endif
			
			transd++; //if you change the order of iteration, this line needs to change (when iterating the first dimension then the second dimension, we replicate the order that the compiler puts the elements in linear memory)
		
		}
	}

	return 0;
} //transd_array_calcflat

/*
 * Deallocate a transducer array.
 * @param transd_array The pointer to the array.
 * @return 0 for success, an error code otherwise.
 */
/*uint8_t transd_array_kill( t_transd_array *transd_array ){

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
}*/ //transd_array_kill

#ifdef DEBUG
/**
 * Dumps the array structure to a JSON file.
 * Usable if compiled for Windows/Linux. Doesn't work if compiled for Arduino.
 * @param f The file pointer.
 * @return 0 for success, an error code otherwise.
 */
uint8_t transd_dumpJSON ( FILE *f/*,  t_transd_array *transd_array*/ ){

	char buff[1024];
	char patt[17];
	t_transd *transd_ptr;
	t_transd_debug *transd_ptr_debug;
	uint8_t i,x,y;
	uint16_t mask;

	if(f == NULL) {
		#ifdef DEBUG
		DEBUG_MSG("Unable to open file")
		#endif
		return 2;
	}

	//the array constants data; Opens the array of transducers
	sprintf(buff, "{\n\t\"size_x\": %u,\n\t\"size_y\": %u,\n\t\"elem_diameter\": %u,\n\t\"elem_separation\": %u,\n\t\"phase_res\": %u,\n\t\"transd_array\": [", ARRAY_SIZE_X, ARRAY_SIZE_Y, TRANS_DIAMETER, TRANS_SEPARATION, ARRAY_PHASERES);
	fputs(buff, f);

	for(x = 0; x < ARRAY_SIZE_X; x++){
		for(y = 0; y < ARRAY_SIZE_Y; y++){

			if( x != 0 || y != 0 ) {
				fputc(',',f);
			}

			transd_ptr = &transd_array[x][y];
			transd_ptr_debug = &transd_array_debug[x][y];

			//converts the 16 bit pattern to a 16 characters string
			//the '#' represents a active output and the '_' represents a inactive output
			for(i = 0; i < 16; i++) {

				mask = (1 << (i)); //creates a mask with the current bit = 1
				if(transd_ptr->pattern & mask) { //if the pattern have the current bit = 1
					//patt[15 - i] = '#'; //the signal starts at the lsb of the pattern
					patt[ARRAY_PHASERES - 1 - i] = '#'; //upgraded to show only the used bits of the pattern
				}
				else {
					//patt[15 - i] = '_';
					patt[ARRAY_PHASERES - 1 - i] = '_'; //upgraded to show only the used bits of the pattern
				}
			}
			//patt[16] = '\0';
			patt[ARRAY_PHASERES] = '\0'; //upgraded to show only the used bits of the pattern

			//the current transducer data
			sprintf(buff, "\n\t{\n\t\t\"port_pin\": %u,\n\t\t\"phase_comp\": %u,\n\t\t\"dist_x\": %u,\n\t\t\"dist_y\": %u,\n\t\t\"dist\": %u,\n\t\t\"phase\": %u,\n\t\t\"duty_cycle\": %u,\n\t\t\"pattern\": \"%s\"\n\t}", transd_ptr->port_pin, transd_ptr->phase_comp, transd_ptr_debug->dist_x, transd_ptr_debug->dist, transd_ptr_debug->dist_y, transd_ptr_debug->phase, transd_ptr_debug->duty_cycle, patt);
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
 * Usable if compiled for Windows/Linux. Doesn't work if compiled for Arduino.
 * @param f The file pointer.
 * @return 0 for success, an error code otherwise.
 */
uint8_t transd_dumpCSV ( FILE *f/*,  t_transd_array *transd_array*/ ){

	char buff[1024];
	char patt[17];
	t_transd *transd_ptr;
	t_transd_debug *transd_ptr_debug;
	uint8_t i,x,y;
	uint16_t mask;

	if(f == NULL) {
		#ifdef DEBUG
		DEBUG_MSG("Unable to open file")
		#endif
		return 2;
	}

	//prints the header of the columns
	sprintf(buff, "idx_x;idx_y;port_pin;phase_comp;dist_x;dist_y;dist;phase;duty_cycle;pattern\n");
	fputs(buff, f);

	for(x = 0; x < ARRAY_SIZE_X; x++){
		for(y = 0; y < ARRAY_SIZE_Y; y++){

			transd_ptr = &transd_array[x][y];
			transd_ptr_debug = &transd_array_debug[x][y];

			//converts the 16 bit pattern to a 16 characters string
			//the '#' represents a active output and the '_' represents a inactive output
			for(i = 0; i < 16; i++) {

				mask = (1 << (i)); //creates a mask with the current bit = 1
				if(transd_ptr->pattern & mask) { //if the pattern have the current bit = 1
					//patt[15 - i] = '#'; //the signal starts at the lsb of the pattern
					patt[ARRAY_PHASERES - 1 - i] = '#'; //upgraded to show only the used bits of the pattern
				}
				else {
					//patt[15 - i] = '_';
					patt[ARRAY_PHASERES - 1 - i] = '_'; //upgraded to show only the used bits of the pattern
				}
			}
			//patt[16] = '\0';
			patt[ARRAY_PHASERES] = '\0'; //upgraded to show only the used bits of the pattern

			//the current transducer data
			sprintf(buff, "%u;%u;%u;%u;%u;%u;%u;%u;%u;%s\n", x, y, transd_ptr->port_pin, transd_ptr->phase_comp, transd_ptr_debug->dist_x, transd_ptr_debug->dist_y, transd_ptr_debug->dist, transd_ptr_debug->phase, transd_ptr_debug->duty_cycle, patt);
			fputs(buff, f);

		}
	}

	return 0;
}
#endif



/* PRIVATE */

/**
 * Calculates the 16 bit pattern that represents the signal based on the phase and duty cycle.
 * The pattern is a sequence of bits that represents one period of the output signal. Each bit defines if the output is high (1) or low (0) at each step of the period.
 * The phase_res defines how many steps one period have (how many bits the pattern will have). The higher the resolution, the more accurate the wave will be. The lsb is the first bit of the period and the msb is the last.
 * @param phase The discrete phase of the transducer. It ranges from 0 to 255 and equates to 0 to 2pi.
 * @param duty Discrete duty cycle of the transducer. It ranges from 0 to 255 and equates to 0% to 100%.
 * @return The pattern.
 */
uint16_t transd_getPattern( /*const uint8_t phase_res,*/ const uint8_t phase, const uint8_t duty ){

	uint8_t bits_duty, bits_phase;
	uint16_t pattern = 0;

	//examples based on 10bit phase resolution

	/*
		duty -> defines the quantity of 1s over 0s (if duty is percentage -> bits on = duty/resolution; 3 = 30/10)
		30% -> _______###
		50% -> _____#####
	*/
	bits_duty = ((duty * ARRAY_PHASERES) / 255); //equation for duty domain [0,255]
	//the odrer of the operations is important to avoid truncate or overflow

	/*
		phase -> defines how much delay the signal have (if phase if degree - > displaced bits = phase / (360/resolution); 2 = 72 / 360 / 10)
		0   -> _____##### (0°)
		2   -> ___#####__ (72°)
		8   -> ##_____### (5 amp, 8 ph)
	*/
	bits_phase = (phase * ARRAY_PHASERES) / 255; //equation for phase domain [0,255]

	/* DEPRECATED
	//sets the (Nth + 1) bit = 1 (00001000b for n = 3) then subtracts one to make all the lesser bits = 1 (00000111b for n = 3)
	pattern = (1 << bits_duty) - 1;

	//shifts the duty cycle pattern to change phase
	pattern <<= bits_phase;

	//if shifting bit for phase "overflows" the number of bits of the pattern (defined by the resolution), the pattern lost some duty cycle information (the bits that overflow when shifting)
	if(bits_phase + bits_duty > phase_res) {
		//generates the bits that were lost on phase shifting at the LSB side
		pattern |= (1 << (bits_duty + bits_phase - phase_res)) - 1;
	}
	*/
	if(bits_phase + bits_duty > ARRAY_PHASERES) { //if if the high bits will overflow outside the bits defined by the resolution
		//inverted case: the high bits overflow: ##_____### (50% 288°; duty = 128 phase = 204; bits_duty = 5 bits_phase = 8)
		//generate the inverse pattern then invert it

		bits_duty = ARRAY_PHASERES - bits_duty;
		bits_phase = bits_duty + bits_phase - ARRAY_PHASERES;
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

/*
 * Initializes an transducer of the array. Saves the pin and phase compensation. Calculates the physical position of the transducer on the array.
 * @param transd The pointer to the transducer on the array.
 * @param index_x The index of the transducer in the array on the x direction.
 * @param index_y The index of the transducer in the array on the y direction.
 * @param port_pin The number of the pin on which the transducer is connected.
 * @param phase_comp The discrete phase offset of the transducer. It ranges from 0 to 255 and equates to 0 to 2pi.
 * @param elem_diameter diameter of each transducer in millimeters.
 * @param elem_separation separation between transducers in millimeters
 * @return 0 for success, an error code otherwise.
 */
/*uint8_t transd_init( t_transd *transd, const uint8_t index_x, const uint8_t index_y, const uint8_t port_pin, const uint8_t phase_comp, const uint8_t elem_diameter, const uint8_t elem_separation ) {

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
	// here is an example of the calculation of the position in one axis (elem_diameter = 16, elem_separation = 2)

	  // diameter           separation
	 // |-------|          |--|
	   // _____      _____      _____
	  // /     \    /     \    /     \
	 // |   -   |  |   -   |  |   -   |
	  // \_____/    \_____/    \_____/

	 // |   |   |  |   |   |  |   |   |
	 // 0   8  16  18  26 34  36  44 52
	 // ------------------------------->
	

	 return 0;
} *///transd_init

/*
 * Updates the pin and phase compensation attributes of an transducer of the array.
 * @param transd The pointer to the transducer on the array.
 * @param port_pin The number of the pin on which the transducer is connected.
 * @param phase_comp The discrete phase offset of the transducer. It ranges from 0 to 255 and equates to 0 to 2pi.
 * @return 0 for success, an error code otherwise.
 */
/*uint8_t transd_set( t_transd *transd, uint8_t port_pin, uint8_t phase_comp ) {

	if(transd == NULL) {
		#ifdef DEBUG
		DEBUG_MSG("The transducer pointer is null")
		#endif
		return 44;
	}

	transd->port_pin = port_pin;
	transd->phase_comp = phase_comp;

	return 0;
}*/ //transd_set

/*
 * Calculates the parameters for an transducer so its signal arrive in phase at a three-dimensional coordinate.
 * @param transd The pointer to the transducer on the array.
 * @param phase_res Number of steps that a wave period is divided into.
 * @param duty_cycle Discrete duty cycle of the transducer. It ranges from 0 to 255 and equates to 0% to 100%.
 * @param focus_x The x coordinate of the focal point in millimeters.
 * @param focus_y The y coordinate of the focal point in millimeters.
 * @param focus_z The z coordinate of the focal point in millimeters.
 * @return 0 for success, an error code otherwise.
 */
/*uint8_t transd_calcfocus( t_transd *transd, const uint8_t phase_res, const uint8_t duty_cycle, const uint8_t focus_x, const uint8_t focus_y, const uint8_t focus_z ){

	uint32_t distance; //type long because it can overflow for some big distances
	uint8_t phase;
	int8_t Dx,Dy; //may be smaller than zero

	if(transd == NULL) {
		#ifdef DEBUG
		DEBUG_MSG("The transducer pointer is null")
		#endif
		return 44;
	}

	
		// W = S / f (wave length = sound speed / frequency)

		// Dij = ( (Xij - x)^2 + (Yij - y)^2 + (-z)^2 )^1/2 (distance from the ij transducer to the focus point)

		// Pij = ( Dij * 255 ) / W  (phase of the ij transducer to have 0° of phase at the focus point)
	
	Dx = transd->x - focus_x; //distance on the x axis
	Dy = transd->y - focus_y; //distance on the y axis
	distance = (Dx * Dx) + (Dy * Dy) + (focus_z * focus_z); //squared distance
	distance *= 100; //multiplies by 10 squared to "keep" one decimal place after the square root.
	distance = transd_sqrt_int(distance); //x10. After taking the square root, the distance is not in millimeters
	phase = (( distance * 2550) / 860); //(dist * 255 / 8.6) //As the distance is multiplied by 10, this equation does an extra by 10 division
	phase += transd->phase_comp; //adds the phase compensation. This is the total phase displacement that the ware does between the transducer and the focal point.
	transd->phase = phase;//(uint8_t) (phase % 255); //As the phase is periodic, gets the mod 255 value (255 = 360 degrees)

	transd->duty_cycle = duty_cycle;
	transd->pattern = transd_getPattern( phase_res, transd->phase, transd->duty_cycle );

	return 0;
}*/ //transd_calcfocus

/*
 * Calculates the parameters for an transducer so its signal is in phase with all the other transducers.
 * @param transd The pointer to the transducer on the array.
 * @param phase_res Number of steps that a wave period is divided into.
 * @param duty_cycle Discrete duty cycle of the transducer. It ranges from 0 to 255 and equates to 0% to 100%.
 * @return 0 for success, an error code otherwise.
 */
/*uint8_t transd_calcflat( t_transd *transd, const uint8_t phase_res, const uint8_t duty_cycle ) {

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
}*/ //transd_calcflat

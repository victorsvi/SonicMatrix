

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include "../../src/Debug.h"
#include "../../src/Ultrasonic.h"


#define TESTLEVEL 4

#define MATRIX_X 4U
#define MATRIX_Y 6U
#define MATRIX_DIAMETER 12U
#define MATRIX_SEPARATION 3U
#define MATRIX_RESOLUTION 10U

int main (int argc, const char* argv[]) {

	uint8_t x,y;
	uint8_t ret = 0;
	int phcomp = 0;
	FILE *f = NULL;
	t_transd_array *transd_array = NULL;

	f = fopen("UltrasonicDump.json","w");

	transd_array = transd_array_init( MATRIX_X, MATRIX_Y, MATRIX_DIAMETER, MATRIX_SEPARATION, MATRIX_RESOLUTION );
	if( transd_array == NULL) goto EXIT;

#if TESTLEVEL == 1
	goto DUMP;
#endif

	for(x = 0; x < MATRIX_X; x++){
		for(y = 0; y < MATRIX_Y; y++){

            //phcomp = rand() % 255;
            phcomp = ((y+1) * 20) - ((x+1) * 5);
			ret = transd_array_set( transd_array, x, y, (x*MATRIX_Y) + y, phcomp );
			if( ret != 0) return ret;
		}
	}

#if TESTLEVEL == 2
	goto DUMP;
#endif

    ret = transd_array_calcflat( transd_array, /*duty_cycle*/ 128 );
    if( ret != 0) return ret;

#if TESTLEVEL == 3
	goto DUMP;
#endif

    ret = transd_array_calcfocus( transd_array, /*duty_cycle*/ 128, /*focus_x*/ 25, /*focus_y*/ 40, /*focus_z*/ 50);
    if( ret != 0) return ret;

#if TESTLEVEL == 4
	goto DUMP;
#endif

DUMP:
	transd_dumpToFile ( f, transd_array );

	return 0;

EXIT:
	printf("\r\n\r\nTest completed with return value %d",ret);
	getchar();
	return ret;
}

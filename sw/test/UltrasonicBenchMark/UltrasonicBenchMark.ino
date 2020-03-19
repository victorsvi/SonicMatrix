
#include <stdint.h>
#include "Ultrasonic.h"

#define N 1000

uint32_t start, end, time;
uint16_t cnt;

t_transd_array *transd_array;


void setup () {

	Serial.begin(9600);
	
}

void loop () {

	transd_array = transd_array_init( 8, 8, 16, 2, 12 );
	start = micros();
	for(cnt = 0; cnt < N; cnt++) {
		transd_array_calcfocus( transd_array, 128, cnt % 160, cnt % 140, cnt % 100 );
	}
	end = micros();
  time = (end - start) / N;
	Serial.print("12 bit resolution: ");
	Serial.print(time);
	Serial.println("us");
	transd_array_kill( transd_array );
	
	
	transd_array = transd_array_init( 8, 8, 16, 2, 10 );
	start = micros();
	for(cnt = 0; cnt < N; cnt++) {
		transd_array_calcfocus( transd_array, 128, cnt % 160, cnt % 140, cnt % 100 );
	}
	end = micros();
  time = (end - start) / N;
	Serial.print("10 bit resolution: ");
	Serial.print(time);
	Serial.println("us");
	transd_array_kill( transd_array );


	transd_array = transd_array_init( 8, 8, 16, 2, 8 );
	start = micros();
	for(cnt = 0; cnt < N; cnt++) {
		transd_array_calcfocus( transd_array, 128, cnt % 160, cnt % 140, cnt % 100 );
	}
	end = micros();
  time = (end - start) / N;
	Serial.print("8 bit resolution: ");
	Serial.print(time);
	Serial.println("us");
	transd_array_kill( transd_array );
	
}

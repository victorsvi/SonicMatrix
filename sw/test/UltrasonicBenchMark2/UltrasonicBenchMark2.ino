
#include <stdint.h>
#include "Ultrasonic2.h"

#define N 1000

uint32_t start, end, time;
uint16_t cnt;

t_transd transd_array[64];

byte buff[128];
uint16_t pattern;
uint8_t x,y,idx;


void setup () {

	Serial.begin(9600);
	
  transd_array_init( transd_array );
}

void loop () {
	
	start = micros();
	for(cnt = 0; cnt < N; cnt++) {
		transd_array_calcfocus( transd_array, 128, cnt % 160, cnt % 140, cnt % 100 );
	}
	end = micros();
  time = (end - start) / N;
	Serial.print("10 bit resolution: ");
	Serial.print(time);
	Serial.println("us");
	
  start = micros();
  for(cnt = 0; cnt < N; cnt++) {
    
    for(x = 0; x < 8; x++){
      for(y = 0; y < 8; y++){
         pattern = (buff[2 * (x * y + y)] << 8) | buff[(2 * (x * y + y)) + 1];
         transd_array_set(transd_array, x, y, pattern, pattern );
      }
    }
  }
  end = micros();
  time = (end - start) / N;
  Serial.print("recieve serial pattern: ");
  Serial.print(time);
  Serial.println("us");

    
  start = micros();
  for(cnt = 0; cnt < N; cnt++) {
    
    for(x = 0; x < 8; x++){
      for(y = 0; y < 8; y++){
         idx = 2 * (x * y + y);
         transd_array_set2(transd_array, x, y, buff[idx], buff[idx + 1] );
      }
    }
  }
  end = micros();
  time = (end - start) / N;
  Serial.print("recieve serial: ");
  Serial.print(time);
  Serial.println("us");
}

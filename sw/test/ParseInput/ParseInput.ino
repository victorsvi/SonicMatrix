#include <stdint.h>

struct s_state {
  uint8_t isActiveMove = 0, isActiveStatic = 0, isStop = 0, isFlat = 0;
  uint8_t x = 0, y = 0, z = 0, d = 0, s = 0;
  uint8_t currStep = 0;
  uint8_t moveCurrStep = 0, moveNSteps = 0;
  uint8_t lastx = 0, lasty = 0, lastz = 0;
} state;

uint8_t parseInput ();
void parseInputToken (char *token);
void clearState ();
void printCommand ();

void setup() {
  
  Serial.begin(115200);

}

void loop() { 

	clearState();
	
	if(parseInput()) {
		
		printCommand();
	}

}

uint8_t parseInput () {
	
	char buffer[64];
	uint8_t buffSize, start, end;
	/*
	char *buffPtr;
	uint8_t n;
	*/
	
	if(Serial.available()) {
		
		buffSize =  Serial.readBytes(buffer,63);
		buffer[buffSize] = '\0';
		buffSize++;
		
		start = end = 0;
		while (start < buffSize) {
						
			while( buffer[start + end] != ' ' && buffer[start + end] != '\0' ) {
				end++;
			}
			buffer[start + end] = '\0';
			parseInputToken(&buffer[start]);
			start += end + 1; end = 0;
		}
		
		return 1;
	}
	
	return 0;
	
		/*
		start = 0; end = start; 
		while( buffer[start + end] != ' ' && buffer[start + end] != '\0' ) {
			end++;
		}
		buffer[start + end] = '\0';
		focusTarget.x = atoi(&buffer[start]);
		
		if( start + end < buffSize ) {
			start += end + 1; end = 0;
			while( buffer[start + end] != ' ' && buffer[start + end] != '\0' ) {
				end++;
			}
			buffer[start + end] = '\0';
			focusTarget.y = atoi(&buffer[start]);
		}
		
		if( start + end < buffSize ) {
			start += end + 1; end = 0;
			while( buffer[start + end] != ' ' && buffer[start + end] != '\0' ) {
				end++;
			}
			buffer[start + end] = '\0';
			focusTarget.z = atoi(&buffer[start]);
		}
		*/
		/*
		buffPtr = &buffer; n = 0; 
		while( *(buffPtr + n) != ' ' && *(buffPtr + n) != '\0' ) {
			n++;
		}
		*(buffPtr + n) = '\0';
		focusTarget.x = atoi(buffPtr);
		
		buffPtr = (buffPtr + ++n); n = 0; 
		if( buffPtr - &buffer < buffSize ) {
			while( *(buffPtr + n) != ' ' && *(buffPtr + n) != '\0' ) {
				n++;
			}
			*(buffPtr + n) = '\0';
			focusTarget.y = atoi(buffPtr);
		}
		
		buffPtr = (buffPtr + ++n); n = 0; 
		if( buffPtr - &buffer < buffSize ) {
			while( *(buffPtr + n) != ' ' && *(buffPtr + n) != '\0' ) {
				n++;
			}
			*(buffPtr + n) = '\0';
			focusTarget.z = atoi(buffPtr);
		}
		*/
	
}

/*

*/
void parseInputToken (char *token) {
	
	if(token == NULL) return;
	if(token[0] == '\0') return;
	
  /*
    valid tokens:
    syntax  action  mandatory_pars
    m Set the move mode (moves from the current position to the desired position then deactivates)  x, y, z, d, s
    r Set the realocate mode (moves from the current position to the desired position and keep active)  x, y, z, d, s
    a Activates the focal point at the desired position x, y, z, d
    f Activates the flat mode d
    i Deactivates the current mode
    x[integer]  Sets the x focus coordinate. Value in milimeters
    y[integer]  Sets the y focus coordinate. Value in milimeters
    z[integer]  Sets the z focus coordinate. Value in milimeters
    d[integer]  Sets the duty cycle. Values from 0 to 255, equating to 0% to 100%
    s[integer]  Sets the speed of the displacement. Value in milimeters per second
  */
  switch (token[0]) {
    case 'i':
      state.isStop = 1;
      break;
    case 'a':
      state.isActiveStatic = 1;
      break;
    case 'f':
      state.isActiveStatic = 1;
      state.isFlat = 1;
      break;
    case 'm':
      state.isActiveMove = 1;
      break;
    case 'r':
      state.isActiveStatic = 1;
      state.isActiveMove = 1;
      break;
    case 'x':
      state.x = atoi(&token[1]);
      break;
    case 'y':
      state.y = atoi(&token[1]);
      break;
    case 'z':
      state.z = atoi(&token[1]);
      break;
    case 'd':
      state.d = atoi(&token[1]);
      break;
    case 's':
      state.s = atoi(&token[1]);
      break;
  }
}

void clearState () {
	
	state.lastx = state.x; state.lasty = state.y; state.lastz = state.z;
	
	state.isActiveMove = 0, state.isActiveStatic = 0, state.isStop = 0, state.isFlat = 0;
	state.x = 0, state.y = 0, state.z = 0, state.d = 0, state.s = 0;
	state.currStep = 0;
	state.moveCurrStep = 0, state.moveNSteps = 0;
	
}

void printCommand () {
	
	if(state.isActiveMove){
		Serial.print("Move from ("); Serial.print(state.lastx); Serial.print(","); Serial.print(state.lasty); Serial.print(","); Serial.print(state.lastz); Serial.print(") to ("); Serial.print(state.x); Serial.print(","); Serial.print(state.y); Serial.print(","); Serial.print(state.z); Serial.print(") at "); Serial.print(state.s); Serial.print("mm/s with duty cycle of "); Serial.print(state.d);
		if(state.isActiveStatic) {
			 Serial.print(" then keep ative");
		}
		else {
			 Serial.print(" then deactivate");
		}
	}
	else if(state.isActiveStatic) {
		Serial.print("Activate at ("); Serial.print(state.x); Serial.print(","); Serial.print(state.y); Serial.print(","); Serial.print(state.z); Serial.print(") with duty cycle of "); Serial.print(state.d);
	}
	if(state.isFlat) {
		Serial.print("Activate flat mode with duty cycle of "); Serial.print(state.d);
	}
	if(state.isStop) {
		Serial.print("Deactivate");
	}
	Serial.print("\n");
	
}

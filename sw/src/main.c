
//pinToBank() //converte pino para porta
//criar um timer para gerar o sinal de 40khz. com resolução de 1/10, o timer deve ter frequência de 400kHz (2.5us) (criar um lib para operar o timer?)
//algum jeito de efetuar o cálculo do delay mais rápido? SIM! posso fazer, para uma coordenada z, preenher uma matriz com o delay vs distância do foco. Depois, para cada transdutor, colocar o valor relativo à distância do foco em x, y. (será que é mas rápido? irá ter que calcular a d = (dx^2 + dy^2)^1/2
//algum jeito de atribuir as portas de forma rápida?
//interpretar a serial de forma assíncrona? implementar de forma que a geração do sinal execute com prioridade e a interpretação da entrada seja feita "quando sobrar ciclos" (software interrupt?)


/*
	PROGRAMAS DE TESTE
	
	Que códigos precisam ser criados para testar os conceitos uilizados?
	
	# Geração de sinal 40kHz * 8 (resolução) (teste do timer) (checar jitter). Fazer também em 64 canais para verificar o desempenho.
	# Interpretação da entrada serial
	# Dump do cálculo das fases
	# Dump das portas
	# Para testar o mapeamento dos pinos (exibir a coordenada x, y e o numero do pino em display, enquanto gera um sinal no pino. a entrada serial indica qual coordenada checar)
	# Comparar a sqrt() do arduino com a de inteiro (custom), calculando a raiz de vários números e contando o tempo para a conclusão.
*/

/*
	IMPORTANTE! Calcular o foco em tempo real é muito lento (cerca de 18ms por cálculo)
	
	No main, usar a array_calc_focus e copiar as patterns para outro array. Esse outro array deve
	armazenar vários "passos", de forma que o conjunto dos passos forme uma tragetória.
	
	O destino é informado pela serial, o arduino (que sabe a origem) vai fazer uma linha reta até o destino, colocando todos os "passos" nesse array antes de iniciar (deve demorar cerca de 180ms para 100 passos).
*/


//#include <stdlib.h>
#include <stdint.h>
//#include <EEPROM.h>
#include <Arduino.h>
#include "Ultrasonic.h"
#include "Debug.h"
#include "Timer4.h"

#define TRANS_DIAMETER 16 //diameter of the element in milimeters
#define TRANS_SEPARATION 2 //distance between two consecutive elements in the array in milimeters
#define ARRAY_SIZE_X 8 //number of transducers of the array in the x dimension
#define ARRAY_SIZE_Y 8 //number of transducers of the array in the y dimension
#define ARRAY_PHASERES 10 //number of transducers of the array in the y dimension
#define TRAJ_RES 1 //trajectory resolution in milimeters
#define TRAJ_MAXSTEPS 100 //maximum steps of the trajectory

/* PROTOTYPES */

void trand_array_load ( t_transd_array *transd_array );
ISR( TIMER4_COMPA_vect );
uint8_t parseInput ();
void parseInputToken (char *token);
void clearState ();
void printCommand ();

/* GLOBAL VARIABLES */

const uint8_t ARRAY_CALIBRATION[ARRAY_SIZE_X][ARRAY_SIZE_Y][2] = {
	//pin numer, phase compensation
	0, 0, //x0, y0
	0, 0 //x0, y1
}

struct s_state {
	uint8_t isActiveMove = 0, isActiveStatic = 0, isStop = 0, isFlat = 0;
	uint8_t x = 0, y = 0, z = 0, d = 0, s = 0;
	uint8_t currStep = 0;
	uint8_t moveCurrStep = 0, moveNSteps = 0;
	uint8_t lastx = 0, lasty = 0, lastz = 0;
} state;

t_transd_array *transd_array = NULL;

uint8_t traj_steps[TRAJ_MAXSTEPS][ARRAY_SIZE_X][ARRAY_SIZE_Y];

void setup () {
	
	Serial.begin(115200);
	
	setTimer4()
	
	transd_array = transd_array_init( ARRAY_SIZE_X, ARRAY_SIZE_Y, TRANS_DIAMETER, TRANS_SEPARATION, ARRAY_PHASERES );
	if( transd_array == NULL ) {
		#ifdef DEBUG
		DEBUG_MSG("The transducer array pointer is null")
		#endif
	}
	trand_array_load (transd_array);
	
}

void loop () {
	
	
}

ISR( TIMER4_COMPA_vect ) {
	
	
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
		syntax	action	mandatory_pars
		m	Set the move mode (moves from the current position to the desired position then deactivates)	x, y, z, d, s
		r	Set the realocate mode (moves from the current position to the desired position and keep active)	x, y, z, d, s
		a	Activates the focal point at the desired position	x, y, z, d
		f	Activates the flat mode	d
		i	Deactivates the current mode
		x[integer]	Sets the x focus coordinate. Value in milimeters
		y[integer]	Sets the y focus coordinate. Value in milimeters
		z[integer]	Sets the z focus coordinate. Value in milimeters
		d[integer]	Sets the duty cycle. Values from 0 to 255, equating to 0% to 100%
		s[integer]	Sets the speed of the displacement. Value in milimeters per second
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

void trand_array_load ( t_transd_array *transd_array ){
	
	uint8_t x, y;
	
	for(x = 0; x < ARRAY_SIZE_X; x++){
		for(y = 0; y < ARRAY_SIZE_Y; y++){

			transd_array_set( transd_array, x, y, ARRAY_CALIBRATION[x][y][0], ARRAY_CALIBRATION[x][y][1] );
		}
	}
}








